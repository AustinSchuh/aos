//! The streamer's GStreamer pipeline: one encoder per camera, shared by every viewer, and one
//! WebRTC session per viewer carrying every camera it watches.
//!
//! ```text
//! appsrc (I420) -> x264enc -> h264 caps -> tee --+---------------------------+
//!   ...one encoder per watched camera            |                           |
//!                                     queue -> rtph264pay -> caps --+   (other viewers)
//!                                     ...one branch per camera      +--> webrtcbin (send only)
//! ```
//!
//! Everything is in one pipeline because a viewer's webrtcbin takes every camera it watches, as
//! a video track each, bundled onto one ICE and DTLS transport: one peer connection per viewer
//! rather than one per camera.  An encoder only gets frames while someone watches its camera.
//! A new session forces a keyframe on its cameras so it can start straight away; the browser's
//! PLIs reach an encoder through its rtph264pay as force-key-unit events.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{anyhow, Context};
use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;
use gstreamer_sdp as gst_sdp;
use gstreamer_video as gst_video;
use gstreamer_webrtc as gst_webrtc;
use tokio::sync::mpsc::UnboundedSender;

use crate::signalling::ToBrowser;

/// How the encoders and WebRTC are set up.
#[derive(Clone, Debug)]
pub struct Settings {
    /// x264's target, in kbit/s, for each camera.
    pub bitrate_kbps: u32,
    /// A keyframe at least this often, in frames.
    pub keyframe_interval: u32,
    /// The UDP ports WebRTC may use, inclusive.
    pub min_port: u32,
    pub max_port: u32,
}

fn element(factory: &str) -> anyhow::Result<gst::Element> {
    gst::ElementFactory::make(factory)
        .build()
        .with_context(|| format!("No {factory} element; is its GStreamer plugin installed?"))
}

/// One camera's encoder, from appsrc to the tee its viewers' branches hang off.
struct Encoder {
    appsrc: gst_app::AppSrc,
    encoder: gst::Element,
    tee: gst::Element,
    elements: Vec<gst::Element>,
    size: (u32, u32),
}

/// One camera's branch of a session: from its encoder's tee into the session's webrtcbin.
struct Branch {
    tee: gst::Element,
    tee_pad: gst::Pad,
    /// Its webrtcbin's request pad, which must be released for webrtcbin to be freed.
    sink: gst::Pad,
    elements: Vec<gst::Element>,
}

/// One viewer's peer connection: a webrtcbin carrying `cameras`, one video track each, in order.
pub struct Session {
    pub cameras: Vec<u32>,
    webrtcbin: gst::Element,
    branches: Vec<Branch>,
}

/// The pipeline every encoder and session lives in.
pub struct Pipeline {
    settings: Settings,
    pipeline: gst::Pipeline,
    encoders: HashMap<u32, Encoder>,
}

impl Pipeline {
    pub fn new(settings: Settings) -> anyhow::Result<Self> {
        let pipeline = gst::Pipeline::with_name("camera_streamer");
        // Nothing waits on the bus, so its messages are handled as they're posted: a message
        // left on it keeps its element alive, and every session's webrtcbin with it.
        pipeline
            .bus()
            .expect("A pipeline has a bus")
            .set_sync_handler(|_, message| {
                let source = message
                    .src()
                    .map_or_else(|| "?".into(), |source| source.path_string());
                match message.view() {
                    gst::MessageView::Error(error) => {
                        log::error!("{source}: {} ({:?})", error.error(), error.debug());
                    }
                    gst::MessageView::Warning(warning) => {
                        log::warn!("{source}: {} ({:?})", warning.error(), warning.debug());
                    }
                    _ => {}
                }
                gst::BusSyncReply::Drop
            });
        pipeline
            .set_state(gst::State::Playing)
            .context("Starting the pipeline")?;
        Ok(Self {
            settings,
            pipeline,
            encoders: HashMap::new(),
        })
    }

    /// The size `camera`'s encoder takes, if it has one.
    pub fn encoder_size(&self, camera: u32) -> Option<(u32, u32)> {
        self.encoders.get(&camera).map(|encoder| encoder.size)
    }

    /// Starts an encoder for `camera`'s `size` frames, replacing any it had.  Sessions using the
    /// old one must be restarted.
    pub fn start_encoder(&mut self, camera: u32, size: (u32, u32)) -> anyhow::Result<()> {
        self.stop_encoder(camera);
        let info = gst_video::VideoInfo::builder(gst_video::VideoFormat::I420, size.0, size.1)
            // The frames come when the camera's do; nothing downstream needs a rate.
            .fps(gst::Fraction::new(0, 1))
            .build()?;
        let appsrc = gst_app::AppSrc::builder()
            .name(format!("camera{camera}"))
            .caps(&info.to_caps()?)
            .is_live(true)
            .format(gst::Format::Time)
            .do_timestamp(false)
            // Never hold up the AOS thread: drop the oldest frame if the encoder falls behind.
            .max_buffers(2)
            .leaky_type(gst_app::AppLeakyType::Downstream)
            .build();
        let encoder = element("x264enc")?;
        encoder.set_property_from_str("tune", "zerolatency");
        // superfast and Main (CABAC) match ultrafast and constrained baseline's picture in a
        // third of the bits, for about half again the CPU (7.4 ms rather than 4.4 ms a frame on
        // an Orin Nano's core at 600 kbit/s).  Still no B-frames, so no more latency.  Not High:
        // Chrome's WebRTC doesn't offer to decode it, and its 8x8 transforms gain almost
        // nothing here (SSIM 0.844 rather than 0.842 at 600 kbit/s).
        encoder.set_property_from_str("speed-preset", "superfast");
        encoder.set_property("bitrate", self.settings.bitrate_kbps);
        encoder.set_property("key-int-max", self.settings.keyframe_interval);
        encoder.set_property("bframes", 0u32);
        let h264 = element("capsfilter")?;
        h264.set_property(
            "caps",
            gst::Caps::builder("video/x-h264")
                .field("profile", "main")
                .field("stream-format", "byte-stream")
                .build(),
        );
        let tee = element("tee")?;
        tee.set_property("allow-not-linked", true);
        let elements = vec![appsrc.clone().upcast(), encoder.clone(), h264, tee.clone()];
        self.pipeline.add_many(&elements)?;
        gst::Element::link_many(&elements)?;
        for element in &elements {
            element.sync_state_with_parent()?;
        }
        self.encoders.insert(
            camera,
            Encoder {
                appsrc,
                encoder,
                tee,
                elements,
                size,
            },
        );
        Ok(())
    }

    /// Stops `camera`'s encoder, if it has one.  End its sessions first.
    pub fn stop_encoder(&mut self, camera: u32) {
        if let Some(encoder) = self.encoders.remove(&camera) {
            let pipeline = self.pipeline.clone();
            // Off the AOS thread: stopping waits for the encoder's streaming thread.
            std::thread::spawn(move || {
                for element in encoder.elements.iter().rev() {
                    let _ = element.set_state(gst::State::Null);
                }
                let _ = pipeline.remove_many(&encoder.elements);
            });
        }
    }

    /// Encodes one I420 frame for `camera`, captured at `capture_ns` on the monotonic clock.
    /// Its RTP timestamp comes from the capture time, not when it's encoded, so processing
    /// jitter doesn't change playback timing.
    pub fn push(&self, camera: u32, frame: Vec<u8>, capture_ns: i64) {
        let Some(encoder) = self.encoders.get(&camera) else {
            return;
        };
        // GStreamer's system clock is the monotonic clock too, so running time = capture time -
        // the pipeline's base time.
        let Some(base) = self.pipeline.base_time() else {
            return;
        };
        let Some(pts) = u64::try_from(capture_ns)
            .ok()
            .and_then(|capture| capture.checked_sub(base.nseconds()))
        else {
            return;
        };
        let mut buffer = gst::Buffer::from_mut_slice(frame);
        buffer
            .get_mut()
            .expect("A new buffer is writable")
            .set_pts(gst::ClockTime::from_nseconds(pts));
        // A full appsrc drops the frame by itself; anything else is the pipeline shutting down.
        let _ = encoder.appsrc.push_buffer(buffer);
    }

    /// Starts a viewer's session for `cameras`, every one of which must have an encoder, and
    /// offers it.  The offer and ICE candidates go to `to_browser`.
    pub fn start_session(
        &mut self,
        cameras: Vec<u32>,
        to_browser: UnboundedSender<ToBrowser>,
    ) -> anyhow::Result<Session> {
        let webrtcbin = element("webrtcbin")?;
        webrtcbin.set_property_from_str("bundle-policy", "max-bundle");
        let ice: gst::glib::Object = webrtcbin.property("ice-agent");
        ice.set_property("min-rtp-port", self.settings.min_port);
        ice.set_property("max-rtp-port", self.settings.max_port);
        let signalling = Arc::new(Mutex::new(Signalling::default()));
        {
            let to_browser = to_browser.clone();
            let signalling = signalling.clone();
            webrtcbin.connect("on-ice-candidate", false, move |values| {
                let mline_index = values[1].get::<u32>().expect("on-ice-candidate's mline");
                let candidate = values[2]
                    .get::<String>()
                    .expect("on-ice-candidate's candidate");
                signalling.lock().expect("Not poisoned").send(
                    &to_browser,
                    ToBrowser::Ice {
                        candidate,
                        mline_index,
                    },
                );
                None
            });
        }
        self.pipeline.add(&webrtcbin)?;

        // Once a session is half built, take it apart again if anything fails.
        let mut session = Session {
            cameras: Vec::new(),
            webrtcbin: webrtcbin.clone(),
            branches: Vec::new(),
        };
        let built = (|| -> anyhow::Result<()> {
            for (index, &camera) in cameras.iter().enumerate() {
                let encoder = self
                    .encoders
                    .get(&camera)
                    .ok_or_else(|| anyhow!("Camera {camera} has no encoder yet"))?;
                let branch = self.add_branch(encoder, &webrtcbin, index)?;
                session.branches.push(branch);
                session.cameras.push(camera);
            }
            // The robot only sends.
            for index in 0..cameras.len() {
                let transceiver = webrtcbin
                    .emit_by_name::<Option<gst_webrtc::WebRTCRTPTransceiver>>(
                        "get-transceiver",
                        &[&(index as i32)],
                    )
                    .ok_or_else(|| anyhow!("webrtcbin made no transceiver {index}"))?;
                transceiver.set_direction(gst_webrtc::WebRTCRTPTransceiverDirection::Sendonly);
            }
            // The offer, once every track has its caps, which is when webrtcbin asks for
            // negotiation.  Offered before, a track whose caps came after the answer sometimes
            // never connected (webrtcbin 1.22: "has not received any caps yet. Skipping."), and
            // its video never came.
            let offered = AtomicBool::new(false);
            let to_browser = to_browser.clone();
            let offer_cameras = session.cameras.clone();
            webrtcbin.connect("on-negotiation-needed", false, move |values| {
                if !offered.swap(true, Ordering::Relaxed) {
                    let webrtcbin = values[0]
                        .get::<gst::Element>()
                        .expect("on-negotiation-needed's webrtcbin");
                    create_offer(
                        &webrtcbin,
                        offer_cameras.clone(),
                        to_browser.clone(),
                        signalling.clone(),
                    );
                }
                None
            });
            webrtcbin.sync_state_with_parent()?;
            // Only now, with webrtcbin's pads active: a buffer into an inactive one is refused,
            // and stops its branch's queue for good.
            for branch in &session.branches {
                branch.tee_pad.link(
                    &branch.elements[0]
                        .static_pad("sink")
                        .expect("A queue has a sink pad"),
                )?;
            }
            Ok(())
        })();
        if let Err(error) = built {
            self.end_session(session);
            return Err(error);
        }

        // So the browser has a picture on every track straight away.
        for camera in &session.cameras {
            if let Some(encoder) = self.encoders.get(camera) {
                encoder.encoder.send_event(
                    gst_video::UpstreamForceKeyUnitEvent::builder()
                        .all_headers(true)
                        .build(),
                );
            }
        }
        Ok(session)
    }

    /// One camera's branch, from its encoder's tee into `webrtcbin` as its `index`th track.
    fn add_branch(
        &self,
        encoder: &Encoder,
        webrtcbin: &gst::Element,
        index: usize,
    ) -> anyhow::Result<Branch> {
        let queue = element("queue")?;
        // A slow viewer drops whole buffers here instead of holding up the others.
        queue.set_property_from_str("leaky", "downstream");
        queue.set_property("max-size-buffers", 0u32);
        queue.set_property("max-size-bytes", 0u32);
        queue.set_property("max-size-time", 200_000_000u64);
        // Each track its own payload type, so the browser never confuses them.
        let payload_type = 96 + index as u32;
        let payloader = element("rtph264pay")?;
        payloader.set_property("config-interval", -1i32);
        payloader.set_property("pt", payload_type);
        payloader.set_property_from_str("aggregate-mode", "zero-latency");
        let rtp = element("capsfilter")?;
        rtp.set_property(
            "caps",
            gst::Caps::builder("application/x-rtp")
                .field("media", "video")
                .field("encoding-name", "H264")
                .field("payload", payload_type as i32)
                .field("clock-rate", 90000i32)
                // Say what the encoder makes, Main at level 3.1, in the offer.  Without it a
                // browser assumes constrained baseline (RFC 6184), and may pick a decoder that
                // can't do CABAC.
                .field("profile-level-id", "4d001f")
                .field("packetization-mode", "1")
                .build(),
        );
        let elements = vec![queue.clone(), payloader, rtp.clone()];
        self.pipeline.add_many(&elements)?;
        gst::Element::link_many(&elements)?;
        let sink = webrtcbin
            .request_pad_simple(&format!("sink_{index}"))
            .ok_or_else(|| anyhow!("webrtcbin gave no sink_{index}"))?;
        rtp.static_pad("src")
            .expect("A capsfilter has a src pad")
            .link(&sink)?;
        let tee_pad = encoder
            .tee
            .request_pad_simple("src_%u")
            .ok_or_else(|| anyhow!("The tee gave no pad"))?;
        // Linked to the tee once webrtcbin is running: see start_session().
        for element in &elements {
            element.sync_state_with_parent()?;
        }
        Ok(Branch {
            tee: encoder.tee.clone(),
            tee_pad,
            sink,
            elements,
        })
    }

    /// The browser's answer to a session's offer.
    pub fn set_answer(&self, session: &Session, sdp: &str) -> anyhow::Result<()> {
        let message = gst_sdp::SDPMessage::parse_buffer(sdp.as_bytes())
            .map_err(|_| anyhow!("Couldn't parse the answer's SDP"))?;
        let answer =
            gst_webrtc::WebRTCSessionDescription::new(gst_webrtc::WebRTCSDPType::Answer, message);
        let name = session.webrtcbin.name();
        let set = gst::Promise::with_change_func(move |reply| {
            if let Some(error) = description_error(reply) {
                log::error!("{name}: couldn't set the answer: {error}");
            }
        });
        session
            .webrtcbin
            .emit_by_name::<()>("set-remote-description", &[&answer, &set]);
        Ok(())
    }

    /// One of the browser's ICE candidates for a session.
    pub fn add_ice_candidate(&self, session: &Session, mline_index: u32, candidate: &str) {
        session
            .webrtcbin
            .emit_by_name::<()>("add-ice-candidate", &[&mline_index, &candidate]);
    }

    /// Ends a session and takes its branches and webrtcbin out of the pipeline.
    ///
    /// The teardown runs on a thread of its own: it waits for GStreamer's streaming threads,
    /// which must never hold up the AOS event loop's thread this is called on.
    pub fn end_session(&mut self, session: Session) {
        let pipeline = self.pipeline.clone();
        std::thread::spawn(move || {
            for branch in &session.branches {
                // Unlink each branch while no buffer is passing through its tee's pad.
                let (unlinked, wait) = mpsc::channel();
                branch
                    .tee_pad
                    .add_probe(gst::PadProbeType::IDLE, move |pad, _| {
                        if let Some(peer) = pad.peer() {
                            let _ = pad.unlink(&peer);
                        }
                        let _ = unlinked.send(());
                        gst::PadProbeReturn::Remove
                    });
                if wait.recv_timeout(Duration::from_secs(1)).is_err() {
                    log::warn!("A tee didn't go idle to unlink a viewer");
                }
                branch.tee.release_request_pad(&branch.tee_pad);
            }
            // Downstream first: a queue's streaming thread can't stop while it is pushing into
            // an element that is still running.
            let _ = session.webrtcbin.set_state(gst::State::Null);
            for branch in &session.branches {
                session.webrtcbin.release_request_pad(&branch.sink);
            }
            let _ = pipeline.remove(&session.webrtcbin);
            for branch in &session.branches {
                for element in branch.elements.iter().rev() {
                    let _ = element.set_state(gst::State::Null);
                }
                let _ = pipeline.remove_many(&branch.elements);
            }
        });
    }
}

impl Drop for Pipeline {
    fn drop(&mut self) {
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}

/// What's been sent to a viewer's browser of its session.  An ICE candidate the browser gets
/// ahead of its offer is dropped, and the robot has few: one lost, and the session may never
/// connect (Chrome's are mDNS names the robot can't resolve).
#[derive(Default)]
struct Signalling {
    offered: bool,
    /// The candidates found before the offer went, to follow it.
    candidates: Vec<ToBrowser>,
}

impl Signalling {
    /// Sends `candidate` to the browser once its offer has gone.
    fn send(&mut self, to_browser: &UnboundedSender<ToBrowser>, candidate: ToBrowser) {
        if self.offered {
            let _ = to_browser.send(candidate);
        } else {
            self.candidates.push(candidate);
        }
    }

    /// Sends the offer, then the candidates found before it.
    fn offer(&mut self, to_browser: &UnboundedSender<ToBrowser>, offer: ToBrowser) {
        let _ = to_browser.send(offer);
        for candidate in self.candidates.drain(..) {
            let _ = to_browser.send(candidate);
        }
        self.offered = true;
    }
}

/// Has `webrtcbin` make its offer for `cameras`, sets it as its local description, and only
/// then sends it to the browser: an answer that came back before the offer was set would be
/// refused, and the session would never connect.
fn create_offer(
    webrtcbin: &gst::Element,
    cameras: Vec<u32>,
    to_browser: UnboundedSender<ToBrowser>,
    signalling: Arc<Mutex<Signalling>>,
) {
    let offered = webrtcbin.clone();
    let promise = gst::Promise::with_change_func(move |reply| {
        let offer = match reply {
            Ok(Some(reply)) => reply
                .value("offer")
                .ok()
                .and_then(|offer| offer.get::<gst_webrtc::WebRTCSessionDescription>().ok()),
            _ => None,
        };
        let Some(offer) = offer else {
            log::error!("{}: couldn't make an offer: {reply:?}", offered.name());
            let _ = to_browser.send(ToBrowser::Error {
                message: "Couldn't make an offer".into(),
            });
            return;
        };
        let sdp = match offer.sdp().as_text() {
            Ok(sdp) => sdp,
            Err(error) => {
                log::error!("{}: the offer has no SDP text: {error}", offered.name());
                return;
            }
        };
        let name = offered.name();
        let set = gst::Promise::with_change_func(move |reply| {
            if let Some(error) = description_error(reply) {
                log::error!("{name}: couldn't set the offer: {error}");
                let _ = to_browser.send(ToBrowser::Error {
                    message: "Couldn't make an offer".into(),
                });
                return;
            }
            signalling
                .lock()
                .expect("Not poisoned")
                .offer(&to_browser, ToBrowser::Offer { sdp, cameras });
        });
        offered.emit_by_name::<()>("set-local-description", &[&offer, &set]);
    });
    webrtcbin.emit_by_name::<()>("create-offer", &[&None::<gst::Structure>, &promise]);
}

/// Why setting a description failed, from webrtcbin's reply, if it did.
fn description_error(
    reply: Result<Option<&gst::StructureRef>, gst::PromiseError>,
) -> Option<String> {
    match reply {
        Ok(Some(reply)) => reply
            .value("error")
            .ok()
            .and_then(|error| error.get::<gst::glib::Error>().ok())
            .map(|error| error.to_string()),
        Ok(None) => None,
        Err(error) => Some(format!("{error:?}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ice(candidate: &str) -> ToBrowser {
        ToBrowser::Ice {
            candidate: candidate.into(),
            mline_index: 0,
        }
    }

    fn offer() -> ToBrowser {
        ToBrowser::Offer {
            sdp: "v=0".into(),
            cameras: vec![0, 2],
        }
    }

    fn sent(outgoing: &mut tokio::sync::mpsc::UnboundedReceiver<ToBrowser>) -> Vec<ToBrowser> {
        let mut sent = Vec::new();
        while let Ok(message) = outgoing.try_recv() {
            sent.push(message);
        }
        sent
    }

    #[test]
    fn candidates_found_before_the_offer_follow_it() {
        let (to_browser, mut outgoing) = tokio::sync::mpsc::unbounded_channel();
        let mut signalling = Signalling::default();
        signalling.send(&to_browser, ice("a"));
        signalling.send(&to_browser, ice("b"));
        assert_eq!(sent(&mut outgoing), vec![]);

        signalling.offer(&to_browser, offer());
        assert_eq!(sent(&mut outgoing), vec![offer(), ice("a"), ice("b")]);
    }

    #[test]
    fn candidates_found_after_the_offer_go_straight_away() {
        let (to_browser, mut outgoing) = tokio::sync::mpsc::unbounded_channel();
        let mut signalling = Signalling::default();
        signalling.offer(&to_browser, offer());
        signalling.send(&to_browser, ice("a"));
        assert_eq!(sent(&mut outgoing), vec![offer(), ice("a")]);
    }
}
