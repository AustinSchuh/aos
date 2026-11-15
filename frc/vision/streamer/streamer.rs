//! Which cameras each viewer watches, and feeding the encoders frames with their overlays.

use std::collections::HashMap;

use tokio::sync::mpsc::UnboundedSender;

use crate::draw::{self, Frame};
use crate::frame;
use crate::join::{Join, JoinOptions, Ready};
use crate::pipeline::{Pipeline, Session, Settings};
use crate::signalling::ToBrowser;
use crate::tag_box::{self, Intrinsics};

/// Who a viewer is: one websocket session.
pub type ViewerId = u64;

/// How frames become streams.
#[derive(Clone, Debug)]
pub struct Options {
    pub settings: Settings,
    /// Halve each side of the camera image before encoding.
    pub half_size: bool,
    /// Send at most this many frames a second of each camera; the rest are skipped.
    pub max_fps: f64,
    /// How long a frame waits for its overlays, and when a detector counts as gone.
    pub join: JoinOptions,
    /// How long a session waits for all its cameras' first frames before starting with the
    /// cameras it has.
    pub start_wait_ns: i64,
    /// The node whose cameras' calibrations to use.
    pub node: String,
    /// How big the AprilTags are, in meters, edge to edge of their black squares.
    pub tag_size: f64,
}

struct Camera {
    last_sent_ns: Option<i64>,
    /// Frames waiting for their overlays.
    join: Join<Frame>,
    /// Stream pixels per camera-image pixel, for drawing the overlays.
    scale: f64,
    /// Its calibration, for drawing the tags' boxes; `None` until the constants have one.
    intrinsics: Option<Intrinsics>,
}

struct Viewer {
    /// The cameras it asked to watch, in the order it asked.
    wanted: Vec<u32>,
    to_browser: UnboundedSender<ToBrowser>,
    session: Option<Session>,
    /// When it started waiting for its cameras' encoders.
    waiting_since_ns: Option<i64>,
}

/// Every camera's encoder, and every viewer's session.
pub struct Streamer {
    options: Options,
    pipeline: Pipeline,
    cameras: HashMap<u32, Camera>,
    viewers: HashMap<ViewerId, Viewer>,
}

impl Streamer {
    pub fn new(options: Options, cameras: &[u32]) -> anyhow::Result<Self> {
        Ok(Self {
            pipeline: Pipeline::new(options.settings.clone())?,
            cameras: cameras
                .iter()
                .map(|&camera| {
                    (
                        camera,
                        Camera {
                            last_sent_ns: None,
                            join: Join::new(options.join),
                            scale: 1.0,
                            intrinsics: None,
                        },
                    )
                })
                .collect(),
            viewers: HashMap::new(),
            options,
        })
    }

    /// The cameras there are to watch.
    pub fn cameras(&self) -> Vec<u32> {
        let mut cameras: Vec<u32> = self.cameras.keys().copied().collect();
        cameras.sort();
        cameras
    }

    fn watched(&self, camera: u32) -> bool {
        self.viewers
            .values()
            .any(|viewer| viewer.wanted.contains(&camera))
    }

    /// Viewer `id` wants to watch `cameras`, replacing whatever it watched.  Its offer comes once
    /// they have frames.
    pub fn watch(
        &mut self,
        id: ViewerId,
        cameras: Vec<u32>,
        to_browser: UnboundedSender<ToBrowser>,
    ) -> anyhow::Result<()> {
        if let Some(missing) = cameras
            .iter()
            .find(|camera| !self.cameras.contains_key(camera))
        {
            anyhow::bail!("There is no camera {missing}");
        }
        self.end_session(id);
        self.viewers.insert(
            id,
            Viewer {
                wanted: cameras,
                to_browser,
                session: None,
                waiting_since_ns: None,
            },
        );
        self.stop_unwatched_encoders();
        Ok(())
    }

    /// A viewer's websocket closed.
    pub fn leave(&mut self, id: ViewerId) {
        self.end_session(id);
        self.viewers.remove(&id);
        self.stop_unwatched_encoders();
    }

    fn end_session(&mut self, id: ViewerId) {
        if let Some(session) = self
            .viewers
            .get_mut(&id)
            .and_then(|viewer| viewer.session.take())
        {
            self.pipeline.end_session(session);
        }
    }

    fn stop_unwatched_encoders(&mut self) {
        for camera in self.cameras() {
            if !self.watched(camera) && self.pipeline.encoder_size(camera).is_some() {
                self.pipeline.stop_encoder(camera);
                if let Some(state) = self.cameras.get_mut(&camera) {
                    state.last_sent_ns = None;
                }
            }
        }
    }

    fn session(&self, id: ViewerId) -> anyhow::Result<&Session> {
        self.viewers
            .get(&id)
            .and_then(|viewer| viewer.session.as_ref())
            .ok_or_else(|| anyhow::anyhow!("No session to answer"))
    }

    pub fn answer(&self, id: ViewerId, sdp: &str) -> anyhow::Result<()> {
        self.pipeline.set_answer(self.session(id)?, sdp)
    }

    pub fn ice(&self, id: ViewerId, mline_index: u32, candidate: &str) -> anyhow::Result<()> {
        self.pipeline
            .add_ice_candidate(self.session(id)?, mline_index, candidate);
        Ok(())
    }

    /// Starts the sessions of viewers whose cameras all have encoders now, or that have waited
    /// long enough for the rest.  A session started without some of its cameras starts again
    /// once they come.
    fn start_waiting_sessions(&mut self, now_ns: i64) {
        let start_wait_ns = self.options.start_wait_ns;
        let ids: Vec<ViewerId> = self.viewers.keys().copied().collect();
        for id in ids {
            let viewer = self.viewers.get(&id).expect("Listed");
            if viewer.wanted.is_empty() {
                continue;
            }
            let ready: Vec<u32> = viewer
                .wanted
                .iter()
                .copied()
                .filter(|&camera| self.pipeline.encoder_size(camera).is_some())
                .collect();
            if let Some(session) = &viewer.session {
                if session.cameras.len() >= ready.len() {
                    continue;
                }
                log::info!("Viewer {id}: more of its cameras have come; starting again");
                self.end_session(id);
            }
            let viewer = self.viewers.get_mut(&id).expect("Listed");
            let since = *viewer.waiting_since_ns.get_or_insert(now_ns);
            if ready.is_empty()
                || (ready.len() < viewer.wanted.len() && now_ns - since < start_wait_ns)
            {
                continue;
            }
            let to_browser = viewer.to_browser.clone();
            match self.pipeline.start_session(ready, to_browser.clone()) {
                Ok(session) => {
                    log::info!("Viewer {id}: streaming cameras {:?}", session.cameras);
                    self.viewers.get_mut(&id).expect("Listed").session = Some(session);
                }
                Err(error) => {
                    log::error!("Viewer {id}: couldn't start its session: {error:#}");
                    let _ = to_browser.send(ToBrowser::Error {
                        message: format!("{error:#}"),
                    });
                    // Don't retry every frame; a new watch tries again.
                    self.viewers.get_mut(&id).expect("Listed").wanted.clear();
                }
            }
        }
    }

    /// Draws each ready frame's overlays and encodes it.
    fn send(&self, camera: u32, ready: Vec<Ready<Frame>>) {
        let Some(size) = self.pipeline.encoder_size(camera) else {
            return;
        };
        let scale = self.cameras.get(&camera).map_or(1.0, |state| state.scale);
        for Ready {
            mut frame,
            capture_ns,
            overlay,
        } in ready
        {
            if (frame.width, frame.height) != size {
                // From before the camera changed size.
                continue;
            }
            if let Some(overlay) = overlay {
                draw::draw(&mut frame, &overlay, scale);
            }
            self.pipeline.push(camera, frame.i420, capture_ns);
        }
    }

    /// One of `camera`'s gray images, arriving at `now_ns`.  Encoded, with its overlays, if
    /// anyone is watching and it isn't too soon after the last one sent.
    pub fn on_image(
        &mut self,
        camera: u32,
        gray: &[u8],
        cols: u32,
        rows: u32,
        capture_ns: i64,
        now_ns: i64,
    ) {
        if !self.cameras.contains_key(&camera) || !self.watched(camera) {
            return;
        }
        let min_interval_ns = (1e9 / self.options.max_fps) as i64;
        if let Some(last) = self.cameras[&camera].last_sent_ns {
            // A little slack, so a camera at exactly max_fps isn't halved by jitter.
            if capture_ns - last < min_interval_ns * 9 / 10 {
                return;
            }
        }
        let size = frame::stream_size(cols, rows, self.options.half_size);
        if size.0 == 0 || size.1 == 0 {
            return;
        }
        if self.pipeline.encoder_size(camera) != Some(size) {
            // First frame, or the camera changed size: a new encoder, and new sessions for
            // everyone watching it.
            let ids: Vec<ViewerId> = self
                .viewers
                .iter()
                .filter(|(_, viewer)| {
                    viewer
                        .session
                        .as_ref()
                        .is_some_and(|session| session.cameras.contains(&camera))
                })
                .map(|(&id, _)| id)
                .collect();
            for id in ids {
                self.end_session(id);
                if let Some(viewer) = self.viewers.get_mut(&id) {
                    viewer.waiting_since_ns = None;
                }
            }
            if let Err(error) = self.pipeline.start_encoder(camera, size) {
                log::error!("Couldn't start camera {camera}'s encoder: {error:#}");
                return;
            }
            log::info!("Encoding camera {camera} at {}x{}", size.0, size.1);
        }

        let mut i420 = Vec::new();
        frame::gray_to_i420(gray, cols, rows, self.options.half_size, &mut i420);
        let state = self.cameras.get_mut(&camera).expect("Checked");
        state.scale = size.0 as f64 / cols as f64;
        state.last_sent_ns = Some(capture_ns);
        let ready = state.join.add_frame(
            Frame {
                i420,
                width: size.0,
                height: size.1,
            },
            capture_ns,
            now_ns,
        );
        self.send(camera, ready);
        self.start_waiting_sessions(now_ns);
    }

    /// The robot's `frc.vision.CameraConstants`: each camera's calibration.
    pub fn on_constants(&mut self, data: &[u8]) {
        for (&camera, state) in self.cameras.iter_mut() {
            let intrinsics = Intrinsics::find(data, &self.options.node, camera);
            if intrinsics.is_none() {
                log::warn!(
                    "No calibration for camera {camera} on {}, so no boxes on its tags",
                    self.options.node
                );
            }
            state.intrinsics = intrinsics;
        }
    }

    /// One of `camera`'s `frc.vision.TargetMap`s, the tags found in one of its images, arriving
    /// at `now_ns`.
    pub fn on_target_map(&mut self, camera: u32, data: &[u8], now_ns: i64) {
        let tag_size = self.options.tag_size;
        let Some(state) = self.cameras.get_mut(&camera) else {
            return;
        };
        let overlay = match tag_box::target_map_overlay(data, state.intrinsics.as_ref(), tag_size) {
            Ok(overlay) => overlay,
            Err(error) => {
                log::warn!("Camera {camera}: bad TargetMap: {error}");
                return;
            }
        };
        let ready = state.join.add_overlay(overlay, now_ns);
        self.send(camera, ready);
    }

    /// When `camera`'s oldest waiting frame stops waiting for its overlays, if one is waiting.
    pub fn next_deadline_ns(&self, camera: u32) -> Option<i64> {
        self.cameras
            .get(&camera)
            .and_then(|state| state.join.next_deadline_ns())
    }

    /// Sends `camera`'s frames whose wait is over by `now_ns`, without their overlays.
    pub fn expire(&mut self, camera: u32, now_ns: i64) {
        if let Some(state) = self.cameras.get_mut(&camera) {
            let ready = state.join.expire(now_ns);
            self.send(camera, ready);
        }
    }

    /// Logs, for each camera, how many frames got their overlays since the last call.
    pub fn log_statistics(&mut self) {
        for camera in self.cameras() {
            let join = &mut self.cameras.get_mut(&camera).expect("Listed").join;
            if join.drawn + join.missed + join.unmatched > 0 {
                log::info!(
                    "Camera {camera}: {} frames with overlays, {} without (none in time), {} \
                     overlays for no waiting frame",
                    join.drawn,
                    join.missed,
                    join.unmatched
                );
            }
            join.drawn = 0;
            join.missed = 0;
            join.unmatched = 0;
        }
    }
}
