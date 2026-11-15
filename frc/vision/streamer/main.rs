//! camera_streamer: the robot's cameras, as live video in the browser.
//!
//! Watches each camera's decoded `/cameraN/gray` images, and streams those that someone is
//! watching, with a box drawn on each AprilTag the detector found (`/cameraN/gray`
//! `frc.vision.TargetMap`): H.264 from x264, one encoder per camera shared by every viewer.  Each
//! viewer gets one WebRTC peer connection carrying every camera it watches.  Browsers set up their
//! sessions over a websocket on localhost, which robot_server serves to them as
//! `wss://<robot>/cameras/ws`.
//!
//! Runs tokio on its AOS event loop's thread (ADR 0008).

use std::cell::RefCell;
use std::net::TcpListener as StdTcpListener;
use std::path::{Path, PathBuf};
use std::rc::Rc;

use aos_configuration::read_config_from;
use aos_events_tokio::TokioShmEventLoop;
use aos_init::Init;
use camera_streamer::join::JoinOptions;
use camera_streamer::pipeline::Settings;
use camera_streamer::signalling::{FromBrowser, ToBrowser};
use camera_streamer::streamer::{Options, Streamer};
use clap::Parser;
use futures::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio_tungstenite::tungstenite::Message;
use vision_rust_fbs::frc::vision;

/// The robot's cameras, as live video in the browser.
#[derive(Parser, Debug)]
struct App {
    /// The AOS configuration.
    #[arg(long, default_value = "aos_config.json")]
    config: PathBuf,
    /// The cameras to stream: each N streams /cameraN/gray.
    #[arg(long, value_delimiter = ',', default_value = "0,1,2,3")]
    cameras: Vec<u32>,
    /// Port, on localhost, for the signalling websocket; robot_server proxies it.
    #[arg(long, default_value_t = 1190)]
    port: u16,
    /// x264's target, in kbit/s, for each camera: four cameras are 2.4 Mbit/s a viewer.
    #[arg(long, default_value_t = 600)]
    bitrate_kbps: u32,
    /// Stream at most this many frames a second of each camera.
    #[arg(long, default_value_t = 30.0)]
    max_fps: f64,
    /// Halve each side of the image before encoding (1600x1304 streams as 800x652).
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    half_size: bool,
    /// The lowest and highest UDP ports WebRTC may use.
    #[arg(long, default_value_t = 5811)]
    min_port: u32,
    #[arg(long, default_value_t = 5830)]
    max_port: u32,
    /// The longest a frame waits for its overlays (the boxes on the tags in /cameraN/gray
    /// frc.vision.TargetMap) before it is sent without them.
    #[arg(long, default_value_t = 50)]
    overlay_wait_ms: i64,
    /// The node whose cameras' calibrations, in /constants frc.vision.CameraConstants, to draw
    /// the tags' boxes with.
    #[arg(long, default_value = "orin")]
    node: String,
    /// The AprilTags' size, in meters: 6.5 inches, as the detector assumes.
    #[arg(long, default_value_t = 6.5 * 0.0254)]
    tag_size: f64,
}

type Shared = Rc<RefCell<Streamer>>;

/// Serves one browser's signalling websocket until it closes.
async fn session(stream: TcpStream, id: u64, streamer: Shared) {
    let socket = match tokio_tungstenite::accept_async(stream).await {
        Ok(socket) => socket,
        Err(error) => {
            log::debug!("Signalling handshake failed: {error}");
            return;
        }
    };
    let (mut to_socket, mut from_socket) = socket.split();
    let (to_browser, mut outgoing) = tokio::sync::mpsc::unbounded_channel::<ToBrowser>();
    let cameras = streamer.borrow().cameras();
    let _ = to_browser.send(ToBrowser::Cameras { cameras });

    // Offers and ICE candidates come from GStreamer's threads through `to_browser`.
    let writer = tokio::task::spawn_local(async move {
        while let Some(message) = outgoing.recv().await {
            if to_socket
                .send(Message::text(message.to_text()))
                .await
                .is_err()
            {
                break;
            }
        }
    });

    while let Some(Ok(message)) = from_socket.next().await {
        let text = match message {
            Message::Text(text) => text,
            Message::Close(_) => break,
            _ => continue,
        };
        let result = match FromBrowser::parse(text.as_str()) {
            Ok(FromBrowser::Watch { cameras }) => {
                streamer.borrow_mut().watch(id, cameras, to_browser.clone())
            }
            Ok(FromBrowser::Answer { sdp }) => streamer.borrow().answer(id, &sdp),
            Ok(FromBrowser::Ice {
                candidate,
                mline_index,
            }) => streamer.borrow().ice(id, mline_index, &candidate),
            Err(error) => Err(anyhow::anyhow!(error)),
        };
        if let Err(error) = result {
            let _ = to_browser.send(ToBrowser::Error {
                message: format!("{error:#}"),
            });
        }
    }
    streamer.borrow_mut().leave(id);
    writer.abort();
}

fn main() {
    let app = App::init();
    gstreamer::init().expect("Failed to start GStreamer");
    let config = read_config_from(Path::new(&app.config)).expect("Failed to read the config");
    let listener = StdTcpListener::bind(("127.0.0.1", app.port))
        .unwrap_or_else(|error| panic!("Failed to listen on {}: {error}", app.port));
    listener
        .set_nonblocking(true)
        .expect("Failed to make the listener non-blocking");

    let streamer: Shared = Rc::new(RefCell::new(
        Streamer::new(
            Options {
                settings: Settings {
                    bitrate_kbps: app.bitrate_kbps,
                    keyframe_interval: (2.0 * app.max_fps).round() as u32,
                    min_port: app.min_port,
                    max_port: app.max_port,
                },
                half_size: app.half_size,
                max_fps: app.max_fps,
                join: JoinOptions {
                    max_wait_ns: app.overlay_wait_ms * 1_000_000,
                    // A detector quiet for a second has stopped; don't hold the video for it.
                    absent_after_ns: 1_000_000_000,
                },
                // A camera that hasn't sent a frame in 2 s isn't worth holding the others for.
                start_wait_ns: 2_000_000_000,
                node: app.node.clone(),
                tag_size: app.tag_size,
            },
            &app.cameras,
        )
        .expect("Failed to start the GStreamer pipeline"),
    ));

    TokioShmEventLoop::new(&config).run(move |aos| {
        let watches: Vec<_> = app
            .cameras
            .iter()
            .map(|&camera| {
                let channel = format!("/camera{camera}/gray");
                (
                    camera,
                    aos.make_watch(&channel, "frc.vision.CameraImage"),
                    aos.make_watch(&channel, "frc.vision.TargetMap"),
                    aos.make_timer(),
                )
            })
            .collect();
        // The calibrations: whatever was sent before we started, then any changes.
        let mut constants_fetcher = aos.make_fetcher("/constants", "frc.vision.CameraConstants");
        let mut constants = aos.make_watch("/constants", "frc.vision.CameraConstants");
        async move {
            if constants_fetcher.fetch() {
                if let Some(data) = constants_fetcher.message().data() {
                    streamer.borrow_mut().on_constants(data);
                }
            } else {
                log::warn!("No /constants frc.vision.CameraConstants yet, so no boxes on tags");
            }
            {
                let streamer = streamer.clone();
                tokio::task::spawn_local(async move {
                    loop {
                        let message = constants.next().await;
                        if let Some(data) = message.data() {
                            streamer.borrow_mut().on_constants(data);
                        }
                    }
                });
            }

            for (camera, mut images, mut overlays, mut timer) in watches {
                let streamer = streamer.clone();
                tokio::task::spawn_local(async move {
                    loop {
                        let deadline = streamer.borrow().next_deadline_ns(camera);
                        tokio::select! {
                            message = images.latest() => {
                                let now_ns = message.monotonic_event_time_ns();
                                let Some(data) = message.data() else {
                                    continue;
                                };
                                let Ok(image) = vision::root_as_camera_image(data) else {
                                    log::warn!("Camera {camera}: not a CameraImage");
                                    continue;
                                };
                                if image.format() != vision::ImageFormat::MONO8 {
                                    continue;
                                }
                                let Some(gray) = image.data() else {
                                    continue;
                                };
                                streamer.borrow_mut().on_image(
                                    camera,
                                    gray.bytes(),
                                    image.cols() as u32,
                                    image.rows() as u32,
                                    image.monotonic_timestamp_ns(),
                                    now_ns,
                                );
                            }
                            message = overlays.latest() => {
                                let now_ns = message.monotonic_event_time_ns();
                                if let Some(data) = message.data() {
                                    streamer.borrow_mut().on_target_map(camera, data, now_ns);
                                }
                            }
                            // The oldest waiting frame has waited long enough for its overlays.
                            now_ns = async {
                                match deadline {
                                    Some(deadline) => {
                                        timer.sleep_until(deadline).await;
                                        deadline
                                    }
                                    None => std::future::pending().await,
                                }
                            } => {
                                streamer.borrow_mut().expire(camera, now_ns);
                            }
                        }
                    }
                });
            }

            {
                let streamer = streamer.clone();
                tokio::task::spawn_local(async move {
                    let mut every = tokio::time::interval(std::time::Duration::from_secs(10));
                    loop {
                        every.tick().await;
                        streamer.borrow_mut().log_statistics();
                    }
                });
            }

            let listener =
                TcpListener::from_std(listener).expect("Failed to hand the listener to tokio");
            log::info!("Signalling on 127.0.0.1:{}", app.port);
            let mut next_id = 0u64;
            loop {
                match listener.accept().await {
                    Ok((stream, _)) => {
                        next_id += 1;
                        tokio::task::spawn_local(session(stream, next_id, streamer.clone()));
                    }
                    Err(error) => log::warn!("Failed to accept a signalling connection: {error}"),
                }
            }
        }
    });
}
