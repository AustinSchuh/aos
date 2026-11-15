//! Which cameras each viewer watches, and feeding the encoders their frames.

use std::collections::HashMap;

use tokio::sync::mpsc::UnboundedSender;

use crate::frame;
use crate::pipeline::{Pipeline, Session, Settings};
use crate::signalling::ToBrowser;

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
    /// How long a session waits for all its cameras' first frames before starting with the
    /// cameras it has.
    pub start_wait_ns: i64,
}

struct Camera {
    last_sent_ns: Option<i64>,
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
                .map(|&camera| (camera, Camera { last_sent_ns: None }))
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

    /// One of `camera`'s gray images, arriving at `now_ns`.  Encoded if anyone is watching and it
    /// isn't too soon after the last one sent.
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
        self.cameras.get_mut(&camera).expect("Checked").last_sent_ns = Some(capture_ns);
        self.pipeline.push(camera, i420, capture_ns);
        self.start_waiting_sessions(now_ns);
    }
}
