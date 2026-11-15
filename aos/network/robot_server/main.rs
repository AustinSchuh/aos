//! The robot's HTTPS front door.  See `documentation/adr/0007-robot-web-server-tls-mcp-logs.md`.
//!
//! Serves the robot's pages (what `web_proxy` serves from its `--data_dir`) over HTTPS with the
//! robot's certificate, and proxies `/ws` to `web_proxy`, `/foxglove` to the Foxglove server and
//! `/cameras/ws` to camera_streamer, so browsers and the Foxglove web app connect over `wss://`.
//! `/api` answers GET requests for the channels' latest messages, as JSON.  Plain HTTP serves a
//! page that checks whether this computer trusts the certificate.
//!
//! Starter runs it as root so it can listen on 443 and 80; it drops to `--user` as soon as it
//! has.  Runs tokio on its AOS event loop's thread (ADR 0008).

use std::{
    net::TcpListener as StdTcpListener,
    path::{Path, PathBuf},
};

use aos_configuration::read_config_from;
use aos_events_tokio::TokioShmEventLoop;
use aos_flatbuffers::Flatbuffer;
use aos_init::Init;
use clap::Parser;

/// The robot's HTTPS front door.
#[derive(Parser, Debug)]
struct App {
    /// The AOS configuration.
    #[arg(long, default_value = "aos_config.json")]
    config: PathBuf,
    /// Port for HTTPS: the pages and the websocket proxies.
    #[arg(long, default_value_t = 443)]
    https_port: u16,
    /// Port for plain HTTP: the page that checks whether this computer trusts the robot.
    #[arg(long, default_value_t = 80)]
    http_port: u16,
    /// The user to run as once listening.  Starter starts robot_server as root so it can listen
    /// on ports below 1024; it refuses to carry on as root without this.
    #[arg(long, default_value = "")]
    user: String,
    /// Where the pages are, as web_proxy's --data_dir.
    #[arg(long, default_value = "www")]
    data_dir: PathBuf,
    /// Where aos_make_tls_cert keeps the certificate.
    #[arg(long, default_value = "/etc/aos/tls")]
    tls_dir: PathBuf,
    /// web_proxy's port; /ws is proxied to its websocket.
    #[arg(long, default_value_t = 1180)]
    web_proxy_port: u16,
    /// The Foxglove server's port; /foxglove is proxied to it.
    #[arg(long, default_value_t = 8765)]
    foxglove_port: u16,
    /// camera_streamer's signalling port; /cameras/ws is proxied to it.
    #[arg(long, default_value_t = 1190)]
    camera_streamer_port: u16,
}

/// Listens on `port` on every address, IPv4 included, ready for tokio.
fn listen(port: u16, what: &str) -> StdTcpListener {
    let listener = StdTcpListener::bind(("::", port))
        .unwrap_or_else(|error| panic!("Failed to listen for {what} on {port}: {error}"));
    listener
        .set_nonblocking(true)
        .expect("Failed to make a listener non-blocking");
    listener
}

fn main() {
    let app = App::init();

    // Listen while still root, then give root up before reading anything or starting the event
    // loop, so its shared memory belongs to the user.
    let https = listen(app.https_port, "HTTPS");
    let http = listen(app.http_port, "HTTP");
    if !app.user.is_empty() {
        robot_server::drop_privileges(&app.user)
            .unwrap_or_else(|error| panic!("Failed to become {}: {error}", app.user));
    } else if unsafe { libc::geteuid() } == 0 {
        panic!("robot_server won't run as root; pass --user to say who to run as");
    }

    // Kept for as long as robot_server runs, so its tasks, which outlive main's borrows, can
    // read it.
    let config: &'static _ = Box::leak(Box::new(
        read_config_from(Path::new(&app.config)).expect("Failed to read the config"),
    ));
    let tls = robot_server::tls_config(&app.tls_dir).unwrap_or_else(|error| {
        panic!(
            "Failed to load the TLS certificate from {}: {error}.  aos-tls.service makes it at \
             boot (aos_make_tls_cert).",
            app.tls_dir.display()
        )
    });
    let options = robot_server::Options {
        data_dir: app.data_dir.clone(),
        web_proxy_ws: format!("ws://127.0.0.1:{}/ws", app.web_proxy_port),
        foxglove_ws: format!("ws://127.0.0.1:{}", app.foxglove_port),
        camera_streamer_ws: format!("ws://127.0.0.1:{}", app.camera_streamer_port),
    };
    log::info!(
        "Serving HTTPS on {} and HTTP on {} as {}",
        app.https_port,
        app.http_port,
        if app.user.is_empty() {
            "this user"
        } else {
            &app.user
        }
    );
    let listen = robot_server::Listen::Tls {
        https,
        https_port: app.https_port,
        tls,
        http,
    };
    TokioShmEventLoop::new(config)
        .run(|aos| robot_server::start(aos, config.message(), options, listen));
}
