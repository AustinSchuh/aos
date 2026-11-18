//! camera_streamer's server: the cameras' images in, each viewer's signalling websocket and
//! WebRTC session out.  main.rs runs it on the robot; a simulation can add it with
//! `camera_streamer_add_to_simulation_with_flags`.

use std::cell::RefCell;
use std::net::TcpListener as StdTcpListener;
use std::path::PathBuf;
use std::rc::Rc;

use crate::annotations::annotations_overlay;
use crate::join::JoinOptions;
use crate::pipeline::{Codec, Settings};
use crate::signalling::{FromBrowser, ToBrowser};
use crate::streamer::{Options, Streamer};
use anyhow::Context;
use aos_flatbuffers::Flatbuffer;
use clap::Parser;
use futures::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio_tungstenite::tungstenite::Message;
use vision_rust_fbs::frc::vision;

/// camera_streamer's flags: its command line, and what a simulation starts it with.
#[derive(Parser, Debug)]
#[command(name = "camera_streamer")]
pub struct Flags {
    /// The AOS configuration.
    #[arg(long, default_value = "aos_config.json")]
    pub config: PathBuf,
    /// The cameras to stream: each N streams /cameraN/gray.
    #[arg(long, value_delimiter = ',', default_value = "0,1,2,3")]
    pub cameras: Vec<u32>,
    /// Port, on localhost, for the signalling websocket; robot_server proxies it.
    #[arg(long, default_value_t = 1190)]
    pub port: u16,
    /// What to encode the cameras with.
    #[arg(long, value_enum, default_value_t = Codec::H264)]
    pub codec: Codec,
    /// The encoder's target, in kbit/s, for each camera: four cameras are 2.4 Mbit/s a viewer.
    #[arg(long, default_value_t = 600)]
    pub bitrate_kbps: u32,
    /// Stream at most this many frames a second of each camera.
    #[arg(long, default_value_t = 30.0)]
    pub max_fps: f64,
    /// Halve each side of the image before encoding (1600x1304 streams as 800x652).
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub half_size: bool,
    /// The lowest and highest UDP ports WebRTC may use.
    #[arg(long, default_value_t = 5811)]
    pub min_port: u32,
    #[arg(long, default_value_t = 5830)]
    pub max_port: u32,
    /// Gather ICE candidates on loopback too, as well as every other interface: for a browser on
    /// this machine when there's nothing else, as in a test's sandbox.
    #[arg(long)]
    pub ice_loopback: bool,
    /// The longest a frame waits for its overlays (the boxes on the tags in /cameraN/gray
    /// frc.vision.TargetMap) before it is sent without them.
    #[arg(long, default_value_t = 50)]
    pub overlay_wait_ms: i64,
    /// The node whose cameras' calibrations, in /constants frc.vision.CameraConstants, to draw
    /// the tags' boxes with.
    #[arg(long, default_value = "orin")]
    pub node: String,
    /// The AprilTags' size, in meters: 6.5 inches, as the detector assumes.
    #[arg(long, default_value_t = 6.5 * 0.0254)]
    pub tag_size: f64,
}

type Shared = Rc<RefCell<Streamer>>;

/// Every interface's addresses, loopback's too: IPv4, and IPv6 but link-local (which would need
/// its interface named).
fn interface_addresses() -> anyhow::Result<Vec<String>> {
    let mut addresses = Vec::new();
    let mut list: *mut libc::ifaddrs = std::ptr::null_mut();
    // SAFETY: getifaddrs fills in `list`, which is freed below.
    if unsafe { libc::getifaddrs(&mut list) } != 0 {
        return Err(std::io::Error::last_os_error()).context("Failed to list the interfaces");
    }
    let mut entry = list;
    while !entry.is_null() {
        // SAFETY: A node of the list getifaddrs made, whose address, if any, is a sockaddr of its
        // family.
        unsafe {
            let address = (*entry).ifa_addr;
            if !address.is_null() {
                match (*address).sa_family as i32 {
                    libc::AF_INET => {
                        let address = &*(address as *const libc::sockaddr_in);
                        addresses.push(
                            std::net::Ipv4Addr::from(u32::from_be(address.sin_addr.s_addr))
                                .to_string(),
                        );
                    }
                    libc::AF_INET6 => {
                        let address = &*(address as *const libc::sockaddr_in6);
                        let address = std::net::Ipv6Addr::from(address.sin6_addr.s6_addr);
                        if !address.is_unicast_link_local() {
                            addresses.push(address.to_string());
                        }
                    }
                    _ => {}
                }
            }
            entry = (*entry).ifa_next;
        }
    }
    // SAFETY: From getifaddrs, freed once.
    unsafe { libc::freeifaddrs(list) };
    Ok(addresses)
}

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

/// Sets camera_streamer up on `aos`'s event loop, which mustn't be running yet: its watches are
/// made now.  The future serves signalling on `listener` until the process ends; run it on the
/// event loop's thread.  GStreamer must be initialized.
pub fn serve(
    aos: &aos_events_tokio::AosHandle,
    config: aos_configuration_fbs::aos::Configuration<'static>,
    flags: Flags,
    listener: StdTcpListener,
) -> anyhow::Result<impl std::future::Future<Output = ()> + 'static> {
    let streamer: Shared = Rc::new(RefCell::new(Streamer::new(
        Options {
            settings: Settings {
                codec: flags.codec,
                bitrate_kbps: flags.bitrate_kbps,
                keyframe_interval: (2.0 * flags.max_fps).round() as u32,
                min_port: flags.min_port,
                max_port: flags.max_port,
                ice_addresses: if flags.ice_loopback {
                    let addresses = interface_addresses()?;
                    log::info!("ICE gathers on {addresses:?}");
                    addresses
                } else {
                    Vec::new()
                },
            },
            half_size: flags.half_size,
            max_fps: flags.max_fps,
            join: JoinOptions {
                max_wait_ns: flags.overlay_wait_ms * 1_000_000,
                // A detector quiet for a second has stopped; don't hold the video for it.
                absent_after_ns: 1_000_000_000,
            },
            // A camera that hasn't sent a frame in 2 s isn't worth holding the others for.
            start_wait_ns: 2_000_000_000,
            node: flags.node.clone(),
            tag_size: flags.tag_size,
        },
        &flags.cameras,
    )?));

    // calibration_server's corners, on robots whose config has them.
    let corner_cameras: Vec<u32> = flags
        .cameras
        .iter()
        .copied()
        .filter(|camera| {
            config.channels().iter().flatten().any(|channel| {
                channel.name() == Some(&format!("/camera{camera}/calibration"))
                    && channel.type_() == Some("foxglove.ImageAnnotations")
            })
        })
        .collect();
    let watches: Vec<_> = flags
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
    let corner_watches: Vec<_> = corner_cameras
        .iter()
        .map(|&camera| {
            (
                camera,
                aos.make_watch(
                    &format!("/camera{camera}/calibration"),
                    "foxglove.ImageAnnotations",
                ),
            )
        })
        .collect();
    let statistics_timer = aos.make_timer();
    let aos = aos.clone();
    listener.set_nonblocking(true)?;
    let serve = async move {
        for (camera, mut watch) in corner_watches {
            let streamer = streamer.clone();
            tokio::task::spawn_local(async move {
                loop {
                    let message = watch.next().await;
                    if let Some(corners) = message.data().and_then(annotations_overlay) {
                        streamer.borrow_mut().on_corners(camera, corners);
                    }
                }
            });
        }
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
            let aos = aos.clone();
            tokio::task::spawn_local(async move {
                loop {
                    let deadline = streamer.borrow().next_deadline_ns(camera);
                    tokio::select! {
                        message = images.latest() => {
                            let now_ns = aos.monotonic_now_ns();
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
                            let now_ns = aos.monotonic_now_ns();
                            if let Some(data) = message.data() {
                                streamer.borrow_mut().on_target_map(camera, data, now_ns);
                            }
                        }
                        // The oldest waiting frame has waited long enough for its overlays.
                        () = async {
                            match deadline {
                                Some(deadline) => timer.sleep_until(deadline).await,
                                None => std::future::pending().await,
                            }
                        } => {
                            streamer.borrow_mut().expire(camera, aos.monotonic_now_ns());
                        }
                    }
                }
            });
        }

        {
            let streamer = streamer.clone();
            let mut timer = statistics_timer;
            let mut next_ns = aos.monotonic_now_ns();
            tokio::task::spawn_local(async move {
                // Every 10 s, on the event loop's clock.
                loop {
                    next_ns += 10_000_000_000;
                    timer.sleep_until(next_ns).await;
                    streamer.borrow_mut().log_statistics();
                }
            });
        }

        let listener =
            TcpListener::from_std(listener).expect("Failed to hand the listener to tokio");
        log::info!("Signalling on 127.0.0.1:{}", flags.port);
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
    };
    Ok(serve)
}

/// camera_streamer in a simulation a C++ test assembled (`aos/events/tokio_simulation.h`),
/// started with `argv`, its own command line (`argc` arguments, the first the program's name):
/// signalling on 127.0.0.1 on its `--port`.  `--config` is ignored: the simulation's
/// configuration is `config`, a flatbuffer of `config_size` bytes.  Returns false, having logged
/// why, if it can't start.
///
/// # Safety
///
/// `simulation` is a live, unrun `aos_tokio_simulation_t`, `event_loop` one of its event loops,
/// `config` points to `config_size` bytes, and `argv` to `argc` C strings.
#[no_mangle]
pub unsafe extern "C" fn camera_streamer_add_to_simulation_with_flags(
    simulation: *mut aos_events_tokio::TokioSimulation,
    event_loop: *mut aos_events_event_loop_c::aos_event_loop_t,
    config: *const u8,
    config_size: usize,
    argc: std::os::raw::c_int,
    argv: *const *const std::os::raw::c_char,
) -> bool {
    // Logging as it would on its own (aos/init.rs), unless the simulation already set a logger.
    let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .try_init();
    // SAFETY: As the caller promises.
    let bytes = unsafe { std::slice::from_raw_parts(config, config_size) }.to_vec();
    // Kept for as long as the process runs, as the server's tasks are.
    let config: &'static _ = match aos_flatbuffers::NonSizePrefixedFlatbuffer::<
        aos_configuration_fbs::aos::Configuration<'static>,
        Vec<u8>,
    >::new(bytes)
    {
        Ok(config) => Box::leak(Box::new(config)),
        Err(error) => {
            log::error!("camera_streamer: the configuration isn't one: {error}");
            return false;
        }
    };
    let mut args = Vec::new();
    for i in 0..argc.max(0) as usize {
        // SAFETY: As the caller promises.
        match unsafe { std::ffi::CStr::from_ptr(*argv.add(i)) }.to_str() {
            Ok(arg) => args.push(arg.to_string()),
            Err(_) => {
                log::error!("camera_streamer: argument {i} isn't UTF-8");
                return false;
            }
        }
    }
    let flags = match Flags::try_parse_from(&args) {
        Ok(flags) => flags,
        Err(error) => {
            log::error!("camera_streamer: {error}");
            return false;
        }
    };
    if let Err(error) = gstreamer::init() {
        log::error!("camera_streamer: GStreamer won't start: {error}");
        return false;
    }
    let listener = match StdTcpListener::bind(("127.0.0.1", flags.port)) {
        Ok(listener) => listener,
        Err(error) => {
            log::error!("camera_streamer: can't listen on {}: {error}", flags.port);
            return false;
        }
    };
    // SAFETY: As the caller promises.
    let simulation = unsafe { &mut *simulation };
    // SAFETY: As the caller promises.
    let aos = unsafe { simulation.setup().handle(event_loop) };
    match serve(&aos, config.message(), flags, listener) {
        Ok(server) => {
            simulation.spawn(server);
            true
        }
        Err(error) => {
            log::error!("camera_streamer: {error:#}");
            false
        }
    }
}
