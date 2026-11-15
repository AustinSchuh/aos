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
    collections::HashMap,
    net::TcpListener as StdTcpListener,
    path::{Path, PathBuf},
};

use aos_configuration::{read_config_from, AsChannel, ChannelExt, ConfigurationExt, Node};
use aos_configuration_fbs::aos::{Channel, Configuration, ReadMethod};
use aos_events_tokio::{AosHandle, RawFetcher, TokioShmEventLoop};
use aos_flatbuffers::Flatbuffer;
use aos_init::Init;
use clap::Parser;
use robot_server::api::{ApiRequest, ChannelInfo, Latest, LatestError, MAX_FETCHABLE_SIZE};
use tokio::{net::TcpListener, sync::mpsc};

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

/// Why `/api/latest` won't fetch `channel`, if it won't.
fn not_fetchable(channel: &Channel, node: Option<&Node>) -> Option<String> {
    if !channel.as_channel().is_readable_on_node(node) {
        Some("This node can't read it".into())
    } else if channel.read_method() == ReadMethod::PIN {
        Some("It's read by pinning, which counts its readers, so it isn't fetched".into())
    } else if channel.max_size() as i64 > MAX_FETCHABLE_SIZE as i64 {
        Some(format!(
            "Its messages can be {} bytes, more than the {MAX_FETCHABLE_SIZE} fetched for the web",
            channel.max_size()
        ))
    } else {
        None
    }
}

/// A channel's name and type, as `/api` names it.
type ChannelKey = (String, String);

fn key(channel: &Channel) -> ChannelKey {
    (
        channel.name().unwrap_or_default().to_string(),
        channel.type_().unwrap_or_default().to_string(),
    )
}

/// What `/api` serves from: the configuration, and a fetcher on every channel it may fetch.
/// AOS makes fetchers only before the event loop runs, so they're all made up front.
struct Api {
    config: Configuration<'static>,
    node: Option<&'static Node>,
    fetchers: HashMap<ChannelKey, RawFetcher>,
}

impl Api {
    fn new(aos: &AosHandle, config: Configuration<'static>, node: Option<&'static Node>) -> Self {
        let fetchers = config
            .channels()
            .iter()
            .flatten()
            .filter(|channel| not_fetchable(channel, node).is_none())
            .map(|channel| {
                let (name, type_) = key(&channel);
                let fetcher = aos.make_fetcher(&name, &type_);
                ((name, type_), fetcher)
            })
            .collect();
        Self {
            config,
            node,
            fetchers,
        }
    }

    fn channels(&self) -> Vec<ChannelInfo> {
        self.config
            .channels()
            .iter()
            .flatten()
            .map(|channel| {
                let (name, type_) = key(&channel);
                ChannelInfo {
                    name,
                    type_,
                    max_size: channel.max_size().max(0) as u32,
                    frequency: channel.frequency(),
                    fetchable: not_fetchable(&channel, self.node).is_none(),
                }
            })
            .collect()
    }

    /// The latest message on `name` `type_`, after the configuration's maps.
    fn latest(
        &mut self,
        name: &str,
        type_: &str,
        max_vector_size: usize,
    ) -> Result<Latest, LatestError> {
        let channel = self
            .config
            .get_channel(name, type_, "robot_server", self.node)
            .map_err(|_| LatestError::NoChannel)?;
        let channel_key = (
            channel.name().unwrap_or_default().to_string(),
            channel.type_().unwrap_or_default().to_string(),
        );
        let Some(fetcher) = self.fetchers.get_mut(&channel_key) else {
            let entry = self
                .config
                .channels()
                .iter()
                .flatten()
                .find(|entry| key(entry) == channel_key)
                .ok_or(LatestError::NoChannel)?;
            return Err(LatestError::NotFetchable(
                not_fetchable(&entry, self.node).unwrap_or_default(),
            ));
        };
        fetcher.fetch();
        let message = fetcher.message();
        let data = message.data().ok_or(LatestError::NoMessage)?;
        Ok(Latest {
            monotonic_event_time_ns: message.monotonic_event_time_ns(),
            realtime_event_time_ns: message.realtime_event_time_ns(),
            json: channel
                .message_to_json(data, max_vector_size)
                .ok_or(LatestError::NotJson)?,
        })
    }
}

/// Answers `/api`'s requests, on the event loop's thread where the fetchers live.
async fn serve_api(mut api: Api, mut requests: mpsc::UnboundedReceiver<ApiRequest>) {
    while let Some(request) = requests.recv().await {
        match request {
            ApiRequest::Channels { reply } => {
                let _ = reply.send(api.channels());
            }
            ApiRequest::Latest {
                name,
                type_,
                max_vector_size,
                reply,
            } => {
                let _ = reply.send(api.latest(&name, &type_, max_vector_size));
            }
        }
    }
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

    // Kept for as long as robot_server runs, so the API's task, which outlives main's borrows,
    // can read it.
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
    let (api_requests, api_incoming) = mpsc::unbounded_channel();
    let router = robot_server::router(robot_server::Options {
        data_dir: app.data_dir.clone(),
        web_proxy_ws: format!("ws://127.0.0.1:{}/ws", app.web_proxy_port),
        foxglove_ws: format!("ws://127.0.0.1:{}", app.foxglove_port),
        camera_streamer_ws: format!("ws://127.0.0.1:{}", app.camera_streamer_port),
        api: api_requests,
    });

    TokioShmEventLoop::new(config).run(|aos| {
        // SAFETY of 'static: the node is part of the configuration, which is leaked.
        let node = aos.node().map(|node| unsafe { &*(node as *const Node) });
        let api = Api::new(aos, config.message(), node);
        async move {
            tokio::task::spawn_local(serve_api(api, api_incoming));
            let https =
                TcpListener::from_std(https).expect("Failed to hand the HTTPS listener to tokio");
            let http =
                TcpListener::from_std(http).expect("Failed to hand the HTTP listener to tokio");
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
            tokio::join!(
                robot_server::serve_https(https, tls, router),
                robot_server::serve_http(http, robot_server::trust_router(app.https_port)),
            );
        }
    });
}
