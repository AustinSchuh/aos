//! camera_streamer: the robot's cameras, as live video in the browser.
//!
//! Watches each camera's decoded `/cameraN/gray` images, and streams those that someone is
//! watching, with a box drawn on each AprilTag the detector found (`/cameraN/gray`
//! `frc.vision.TargetMap`): H.264 from x264 (or VP8, with `--codec=vp8`), one encoder per camera
//! shared by every viewer.  Each viewer gets one WebRTC peer connection carrying every camera it
//! watches.  Browsers set up their sessions over a websocket on localhost, which robot_server
//! serves to them as `wss://<robot>/cameras/ws`.
//!
//! Runs tokio on its AOS event loop's thread (ADR 0008).

use std::net::TcpListener as StdTcpListener;
use std::path::Path;

use aos_configuration::read_config_from;
use aos_events_tokio::TokioShmEventLoop;
use aos_flatbuffers::Flatbuffer;
use aos_init::Init;
use camera_streamer::server::{serve, Flags};

fn main() {
    let flags = Flags::init();
    gstreamer::init().expect("Failed to start GStreamer");
    // Kept for as long as camera_streamer runs, as its tasks are.
    let config: &'static _ = Box::leak(Box::new(
        read_config_from(Path::new(&flags.config)).expect("Failed to read the config"),
    ));
    let listener = StdTcpListener::bind(("127.0.0.1", flags.port))
        .unwrap_or_else(|error| panic!("Failed to listen on {}: {error}", flags.port));
    TokioShmEventLoop::new(config).run(move |aos| {
        serve(aos, config.message(), flags, listener)
            .expect("Failed to start the GStreamer pipeline")
    });
}
