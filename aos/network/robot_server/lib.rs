//! The robot's HTTPS front door: static pages, and the websocket proxies to the servers behind
//! it, over TLS from the robot's certificate.  See
//! `documentation/adr/0007-robot-web-server-tls-mcp-logs.md`, and ADR 0008 for how it runs on
//! the AOS event loop's thread.

pub mod api;
pub mod condition;
pub mod session;

use std::{
    ffi::CString,
    fs, io,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    time::SystemTime,
};

use axum::{
    extract::ws::{self, WebSocket, WebSocketUpgrade},
    response::{Html, IntoResponse, Response},
    routing::get,
    Router,
};
use futures::{SinkExt, StreamExt};
use hyper_util::{rt::TokioIo, service::TowerToHyperService};
use rustls::{
    server::{ClientHello, ResolvesServerCert},
    sign::CertifiedKey,
    ServerConfig,
};
use rustls_pki_types::{pem::PemObject, CertificateDer, PrivateKeyDer};
use tokio::{net::TcpListener, sync::mpsc};
use tokio_rustls::TlsAcceptor;
use tokio_tungstenite::tungstenite::{self, client::IntoClientRequest};
use tower_http::services::ServeDir;

/// The certificate chain file `aos_make_tls_cert` writes: the server certificate, then its CA.
pub const CHAIN_FILE: &str = "server-chain.crt";
/// The private key file `aos_make_tls_cert` writes.
pub const KEY_FILE: &str = "server.key";

/// The websocket subprotocol Foxglove speaks.
const FOXGLOVE_SUBPROTOCOL: &str = "foxglove.websocket.v1";

/// The robot's certificate, read again whenever its files change.
///
/// `aos_make_tls_cert` reissues the certificate at boot and after a rename, and a server that
/// kept the old one would serve a certificate for the wrong name until restarted.  Checking the
/// files' modification times on each handshake is two `stat()`s, and a reissued certificate
/// takes effect on the next connection.
#[derive(Debug)]
pub struct ReloadingCert {
    chain_path: PathBuf,
    key_path: PathBuf,
    loaded: Mutex<Loaded>,
}

#[derive(Debug)]
struct Loaded {
    stamp: (SystemTime, SystemTime),
    key: Arc<CertifiedKey>,
}

fn modified(path: &Path) -> io::Result<SystemTime> {
    fs::metadata(path)?.modified()
}

fn invalid(error: impl std::fmt::Display) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, error.to_string())
}

impl ReloadingCert {
    /// Loads [`CHAIN_FILE`] and [`KEY_FILE`] from `tls_dir`.
    pub fn new(tls_dir: &Path) -> io::Result<Arc<Self>> {
        let chain_path = tls_dir.join(CHAIN_FILE);
        let key_path = tls_dir.join(KEY_FILE);
        let loaded = Self::load(&chain_path, &key_path)?;
        Ok(Arc::new(Self {
            chain_path,
            key_path,
            loaded: Mutex::new(loaded),
        }))
    }

    fn load(chain_path: &Path, key_path: &Path) -> io::Result<Loaded> {
        // Before reading, so a file replaced while reading is read again next time.
        let stamp = (modified(chain_path)?, modified(key_path)?);
        let chain = CertificateDer::pem_file_iter(chain_path)
            .map_err(invalid)?
            .collect::<Result<Vec<_>, _>>()
            .map_err(invalid)?;
        if chain.is_empty() {
            return Err(invalid(format!(
                "{} has no certificates",
                chain_path.display()
            )));
        }
        let key = PrivateKeyDer::from_pem_file(key_path).map_err(invalid)?;
        let signing_key = rustls::crypto::ring::sign::any_supported_type(&key).map_err(invalid)?;
        Ok(Loaded {
            stamp,
            key: Arc::new(CertifiedKey::new(chain, signing_key)),
        })
    }

    /// The current certificate, reloaded first if its files changed.  A reload that fails (a
    /// file half-written) keeps serving the previous one.
    fn current(&self) -> Arc<CertifiedKey> {
        let mut loaded = self.loaded.lock().unwrap();
        let stamp = (modified(&self.chain_path), modified(&self.key_path));
        if let (Ok(chain), Ok(key)) = stamp {
            if (chain, key) != loaded.stamp {
                match Self::load(&self.chain_path, &self.key_path) {
                    Ok(reloaded) => {
                        log::info!(
                            "Reloaded the TLS certificate from {}",
                            self.chain_path.display()
                        );
                        *loaded = reloaded;
                    }
                    Err(error) => log::warn!(
                        "Failed to reload the TLS certificate, keeping the old one: {error}"
                    ),
                }
            }
        }
        loaded.key.clone()
    }
}

impl ResolvesServerCert for ReloadingCert {
    fn resolve(&self, _client_hello: ClientHello<'_>) -> Option<Arc<CertifiedKey>> {
        Some(self.current())
    }
}

/// A rustls server configuration serving the robot's certificate from `tls_dir`.
pub fn tls_config(tls_dir: &Path) -> io::Result<Arc<ServerConfig>> {
    let mut config =
        ServerConfig::builder_with_provider(Arc::new(rustls::crypto::ring::default_provider()))
            .with_safe_default_protocol_versions()
            .map_err(invalid)?
            .with_no_client_auth()
            .with_cert_resolver(ReloadingCert::new(tls_dir)?);
    config.alpn_protocols = vec![b"http/1.1".to_vec()];
    Ok(Arc::new(config))
}

/// robot_server's flags: its command line, and what a simulation starts it with.
#[derive(clap::Parser, Debug)]
#[command(name = "robot_server")]
pub struct Flags {
    /// The AOS configuration.
    #[arg(long, default_value = "aos_config.json")]
    pub config: PathBuf,
    /// Port for HTTPS: the pages and the websocket proxies.
    #[arg(long, default_value_t = 443)]
    pub https_port: u16,
    /// Port for plain HTTP: the page that checks whether this computer trusts the robot.
    #[arg(long, default_value_t = 80)]
    pub http_port: u16,
    /// The user to run as once listening.  Starter starts robot_server as root so it can listen
    /// on ports below 1024; it refuses to carry on as root without this.
    #[arg(long, default_value = "")]
    pub user: String,
    /// Where the pages are, as web_proxy's --data_dir.
    #[arg(long, default_value = "www")]
    pub data_dir: PathBuf,
    /// Where aos_make_tls_cert keeps the certificate.
    #[arg(long, default_value = "/etc/aos/tls")]
    pub tls_dir: PathBuf,
    /// web_proxy's port; /ws is proxied to its websocket.
    #[arg(long, default_value_t = 1180)]
    pub web_proxy_port: u16,
    /// The Foxglove server's port; /foxglove is proxied to it.
    #[arg(long, default_value_t = 8765)]
    pub foxglove_port: u16,
    /// camera_streamer's signalling port; /cameras/ws is proxied to it.
    #[arg(long, default_value_t = 1190)]
    pub camera_streamer_port: u16,
    /// Channels POST /api/send may send on, as name:type.
    #[arg(long, value_delimiter = ',')]
    pub sendable_channels: Vec<String>,
    /// An app the page starts and stops with commands, as JSON with an "action", on this channel
    /// (name:type).  Empty for none.
    #[arg(long, default_value = "")]
    pub session_commands: String,
    /// The app, as starter names it.
    #[arg(long, default_value = "")]
    pub session_app: String,
    /// The apps to stop while it runs.
    #[arg(long, value_delimiter = ',')]
    pub session_stops: Vec<String>,
    /// A channel (name:type) the app sends on regularly, to tell it's ready.
    #[arg(long, default_value = "")]
    pub session_ready: String,
    /// The actions that start and end a session.
    #[arg(long, value_delimiter = ',', default_value = "START")]
    pub session_start_actions: Vec<String>,
    #[arg(long, value_delimiter = ',', default_value = "FINISH,CANCEL")]
    pub session_end_actions: Vec<String>,
}

impl Flags {
    /// What `/api` may send on, and the app it starts and stops.
    pub fn api_options(&self) -> api::ApiOptions {
        use crate::session::{parse_channel, SessionApp};
        let channel = |text: &str, flag: &str| {
            parse_channel(text).unwrap_or_else(|| panic!("--{flag} {text} isn't name:type"))
        };
        api::ApiOptions {
            sendable: self
                .sendable_channels
                .iter()
                .map(|text| channel(text, "sendable_channels"))
                .collect(),
            session: (!self.session_commands.is_empty()).then(|| SessionApp {
                commands: channel(&self.session_commands, "session_commands"),
                app: self.session_app.clone(),
                stops: self.session_stops.clone(),
                ready: channel(&self.session_ready, "session_ready"),
                start_actions: self.session_start_actions.clone(),
                end_actions: self.session_end_actions.clone(),
            }),
        }
    }
}

/// Where the HTTPS router finds what it serves.
#[derive(Clone, Debug)]
pub struct Options {
    /// The pages, as `web_proxy` serves them from its `--data_dir`.
    pub data_dir: PathBuf,
    /// `web_proxy`'s websocket, which only carries WebRTC signaling (`ws://127.0.0.1:1180/ws`).
    pub web_proxy_ws: String,
    /// The Foxglove server's websocket (`ws://127.0.0.1:8765`).
    pub foxglove_ws: String,
    /// camera_streamer's websocket, which carries its WebRTC signalling (`ws://127.0.0.1:1190`).
    pub camera_streamer_ws: String,
}

/// The HTTPS side: the pages, the channels' latest messages, which it asks `api` for, and the
/// websocket proxies.
pub fn router(options: Options, api: mpsc::UnboundedSender<api::ApiRequest>) -> Router {
    let web_proxy_ws = options.web_proxy_ws.clone();
    let foxglove_ws = options.foxglove_ws.clone();
    let camera_streamer_ws = options.camera_streamer_ws.clone();
    Router::new()
        .route("/healthz", get(|| async { "ok" }))
        .merge(api::router(api))
        .route(
            "/ws",
            get(move |upgrade: WebSocketUpgrade| {
                let upstream = web_proxy_ws.clone();
                async move { proxy_websocket(upgrade, upstream, None) }
            }),
        )
        .route(
            "/foxglove",
            get(move |upgrade: WebSocketUpgrade| {
                let upstream = foxglove_ws.clone();
                async move { proxy_websocket(upgrade, upstream, Some(FOXGLOVE_SUBPROTOCOL)) }
            }),
        )
        .route(
            "/cameras/ws",
            get(move |upgrade: WebSocketUpgrade| {
                let upstream = camera_streamer_ws.clone();
                async move { proxy_websocket(upgrade, upstream, None) }
            }),
        )
        .fallback_service(ServeDir::new(options.data_dir))
        // The pages change with every deploy.  Browsers may keep them, but must ask the robot
        // (a 304 if unchanged) before showing them, rather than guess from their age.
        .layer(axum::middleware::map_response(no_cache))
}

async fn no_cache(mut response: Response) -> Response {
    response.headers_mut().insert(
        axum::http::header::CACHE_CONTROL,
        axum::http::HeaderValue::from_static("no-cache"),
    );
    response
}

/// Accepts the browser's websocket and joins it to `upstream`, passing `subprotocol` through.
fn proxy_websocket(
    upgrade: WebSocketUpgrade,
    upstream: String,
    subprotocol: Option<&'static str>,
) -> Response {
    let upgrade = match subprotocol {
        Some(subprotocol) => upgrade.protocols([subprotocol]),
        None => upgrade,
    };
    upgrade.on_upgrade(move |browser| async move {
        if let Err(error) = pump(browser, &upstream, subprotocol).await {
            log::warn!("Websocket proxy to {upstream} ended: {error}");
        }
    })
}

/// Copies messages both ways between `browser` and a new connection to `upstream`, until either
/// side closes.
async fn pump(
    browser: WebSocket,
    upstream: &str,
    subprotocol: Option<&'static str>,
) -> anyhow::Result<()> {
    let mut request = upstream.into_client_request()?;
    if let Some(subprotocol) = subprotocol {
        request
            .headers_mut()
            .insert("Sec-WebSocket-Protocol", subprotocol.parse()?);
    }
    let (upstream, _) = tokio_tungstenite::connect_async(request).await?;
    let (mut upstream_tx, mut upstream_rx) = upstream.split();
    let (mut browser_tx, mut browser_rx) = browser.split();

    let to_upstream = async {
        while let Some(message) = browser_rx.next().await {
            let message = match message? {
                ws::Message::Text(text) => tungstenite::Message::text(text.as_str()),
                ws::Message::Binary(data) => tungstenite::Message::binary(data),
                ws::Message::Close(_) => break,
                // Each side answers its own pings.
                ws::Message::Ping(_) | ws::Message::Pong(_) => continue,
            };
            upstream_tx.send(message).await?;
        }
        let _ = upstream_tx.close().await;
        anyhow::Ok(())
    };
    let to_browser = async {
        while let Some(message) = upstream_rx.next().await {
            let message = match message? {
                tungstenite::Message::Text(text) => ws::Message::text(text.as_str()),
                tungstenite::Message::Binary(data) => ws::Message::binary(data),
                tungstenite::Message::Close(_) => break,
                tungstenite::Message::Ping(_)
                | tungstenite::Message::Pong(_)
                | tungstenite::Message::Frame(_) => continue,
            };
            browser_tx.send(message).await?;
        }
        let _ = browser_tx.close().await;
        anyhow::Ok(())
    };
    // Whichever side closes first ends both.
    tokio::select! {
        result = to_upstream => result,
        result = to_browser => result,
    }
}

/// The plain-HTTP side: whatever path is asked for, a page that checks whether this computer
/// trusts the robot's certificate.  If it does, the page goes straight to the same path over
/// HTTPS; if not, it says how to trust it, and keeps checking.  A browser can't tell an
/// untrusted certificate from an unreachable server, but a page that loaded over HTTP knows the
/// robot is reachable, so a failed HTTPS probe means the certificate.
pub fn trust_router(https_port: u16) -> Router {
    let page = trust_page(https_port);
    Router::new().fallback(get(move || {
        let page = page.clone();
        async move { Html(page).into_response() }
    }))
}

fn trust_page(https_port: u16) -> String {
    // What follows the host in the HTTPS links: nothing on the default port.
    let port = if https_port == 443 {
        String::new()
    } else {
        format!(":{https_port}")
    };
    format!(
        r##"<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot pages</title>
<style>
  body {{ font-family: system-ui, sans-serif; max-width: 40rem; margin: 2rem auto; padding: 0 1rem; line-height: 1.5; }}
  code {{ background: #eee; padding: 0.1rem 0.3rem; border-radius: 3px; }}
  .hidden {{ display: none; }}
</style>
</head>
<body>
<h1>Robot pages</h1>
<p id="checking">Checking whether this computer trusts the robot's certificate&hellip;</p>
<p id="trusted" class="hidden">This computer trusts the robot. Going to
  <a id="secure" href="#">the secure page</a>&hellip;</p>
<div id="untrusted" class="hidden">
  <p>This computer doesn't trust the robot's certificate yet, so browsers and Foxglove can't
  connect to it securely. To trust it, once per computer:</p>
  <ol>
    <li>Get <code>trust_orin.py</code> from meta-frc4646 and run it:
      <p><code id="command">python3 trust_orin.py</code></p>
      It shows the robot's certificate authority and what it can vouch for: only your team's
      robots, nothing else. Check the fingerprint it prints, then say yes.</li>
    <li>Restart the browser completely (every window). Chrome remembers a certificate it has
      already rejected until it restarts.</li>
    <li>Come back to <a id="again" href="#">this page</a>. It goes on to the secure page by
      itself once the robot is trusted.</li>
  </ol>
  <p>On a Mac, it asks for your password. On Windows, it asks you to confirm. On Linux, it
  needs <code>certutil</code> (<code>sudo apt install libnss3-tools</code>). Firefox on a Mac
  or Windows also needs <code>security.enterprise_roots.enabled</code> set in
  <code>about:config</code>. Foxglove's desktop app trusts it after a restart too.</p>
</div>
<script>
  // The certificate names orin-TEAM-N.local, not the bare name, so always go there.
  const host = /^orin-[0-9]+-[0-9]$/.test(location.hostname)
      ? `${{location.hostname}}.local` : location.hostname;
  const https = `https://${{host}}{port}`;
  const target = https + location.pathname + location.search + location.hash;
  document.getElementById('secure').href = target;
  document.getElementById('again').href = `http://${{host}}${{location.pathname}}`;
  document.getElementById('command').textContent =
      `python3 trust_orin.py ${{host.replace(/\.local$/, '')}}`;
  const show = (id) => {{
    for (const other of ['checking', 'trusted', 'untrusted']) {{
      document.getElementById(other).classList.toggle('hidden', other !== id);
    }}
  }};
  // An opaque response is enough: it only resolves if the TLS handshake succeeded.  A probe
  // that hangs counts as untrusted.
  const probe = () => fetch(`${{https}}/healthz`, {{
    mode: 'no-cors', cache: 'no-store', signal: AbortSignal.timeout(5000)}});
  const check = () => probe().then(() => {{
    show('trusted');
    // replace, so Back doesn't return here and bounce straight forward again.
    location.replace(target);
  }}, () => {{
    show('untrusted');
    setTimeout(check, 3000);
  }});
  check();
</script>
</body>
</html>
"##
    )
}

/// Becomes `user` for good: its groups, then its group and user IDs, real, effective and saved.
///
/// Starter runs robot_server as root so it can listen on 443 and 80, as `irq_affinity` is run
/// as root.  Unlike `irq_affinity`, nothing after the listeners needs root, so this keeps no
/// way back to it.  Call it before anything else starts threads, and before the event loop
/// creates its shared memory, so that belongs to `user`.
pub fn drop_privileges(user: &str) -> io::Result<()> {
    let name = CString::new(user).map_err(|_| invalid(format!("Bad user name {user:?}")))?;
    let last_error = |what: String| {
        let error = io::Error::last_os_error();
        io::Error::new(error.kind(), format!("{what}: {error}"))
    };

    // SAFETY: getpwnam_r writes only into `entry` and `buffer`, which outlive the call, and
    // `entry`'s pointers into `buffer` aren't used after it.
    let (uid, gid) = unsafe {
        let mut entry: libc::passwd = std::mem::zeroed();
        let mut buffer = vec![0 as libc::c_char; 16 * 1024];
        let mut found: *mut libc::passwd = std::ptr::null_mut();
        let result = libc::getpwnam_r(
            name.as_ptr(),
            &mut entry,
            buffer.as_mut_ptr(),
            buffer.len(),
            &mut found,
        );
        if result != 0 {
            let error = io::Error::from_raw_os_error(result);
            return Err(io::Error::new(
                error.kind(),
                format!("Failed to look up user {user}: {error}"),
            ));
        }
        if found.is_null() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("There is no user {user}"),
            ));
        }
        (entry.pw_uid, entry.pw_gid)
    };

    // Groups first: only root may set them.
    // SAFETY: plain syscalls on values, and `name` is a valid C string.
    unsafe {
        if libc::initgroups(name.as_ptr(), gid) != 0 {
            return Err(last_error(format!("Failed to set {user}'s groups")));
        }
        if libc::setresgid(gid, gid, gid) != 0 {
            return Err(last_error(format!(
                "Failed to change the group ID to {gid}"
            )));
        }
        if libc::setresuid(uid, uid, uid) != 0 {
            return Err(last_error(format!("Failed to change the user ID to {uid}")));
        }
        if uid != 0 && libc::setuid(0) == 0 {
            return Err(io::Error::other(format!(
                "Still able to become root after changing to {user}"
            )));
        }
    }
    Ok(())
}

/// Serves `router` over TLS on `listener`, until the future is dropped.
pub async fn serve_https(listener: TcpListener, tls: Arc<ServerConfig>, router: Router) {
    let acceptor = TlsAcceptor::from(tls);
    loop {
        let (tcp, peer) = match listener.accept().await {
            Ok(accepted) => accepted,
            Err(error) => {
                log::warn!("Failed to accept an HTTPS connection: {error}");
                continue;
            }
        };
        let acceptor = acceptor.clone();
        let router = router.clone();
        tokio::spawn(async move {
            let stream = match acceptor.accept(tcp).await {
                Ok(stream) => stream,
                Err(error) => {
                    // Untrusted-certificate rejections from browsers land here, routinely.
                    log::debug!("TLS handshake with {peer} failed: {error}");
                    return;
                }
            };
            serve_connection(TokioIo::new(stream), router, peer).await;
        });
    }
}

/// Serves `router` over plain HTTP on `listener`, until the future is dropped.
pub async fn serve_http(listener: TcpListener, router: Router) {
    loop {
        let (tcp, peer) = match listener.accept().await {
            Ok(accepted) => accepted,
            Err(error) => {
                log::warn!("Failed to accept an HTTP connection: {error}");
                continue;
            }
        };
        let router = router.clone();
        tokio::spawn(async move { serve_connection(TokioIo::new(tcp), router, peer).await });
    }
}

async fn serve_connection<I>(io: I, router: Router, peer: SocketAddr)
where
    I: hyper::rt::Read + hyper::rt::Write + Unpin + Send + 'static,
{
    if let Err(error) = hyper::server::conn::http1::Builder::new()
        .serve_connection(io, TowerToHyperService::new(router))
        // Websockets upgrade the connection.
        .with_upgrades()
        .await
    {
        log::debug!("HTTP connection from {peer} ended: {error}");
    }
}

/// How robot_server listens.
pub enum Listen {
    /// HTTPS, and the page that checks whether a computer trusts the robot on plain HTTP: the
    /// robot.
    Tls {
        https: std::net::TcpListener,
        https_port: u16,
        tls: Arc<ServerConfig>,
        http: std::net::TcpListener,
    },
    /// Plain HTTP only: a replay on a laptop, which has no robot certificate.
    Plain { http: std::net::TcpListener },
}

/// Sets robot_server up on `aos`'s event loop, which mustn't be running yet: `/api`'s fetchers
/// are made now.  The future serves until the process ends; run it on the event loop's thread.
pub fn start(
    aos: &aos_events_tokio::AosHandle,
    config: aos_configuration_fbs::aos::Configuration<'static>,
    options: Options,
    api_options: api::ApiOptions,
    listen: Listen,
) -> impl std::future::Future<Output = ()> + 'static {
    let fetchers = api::Fetchers::new(aos, config, api_options);
    let (api_requests, api_incoming) = mpsc::unbounded_channel();
    let router = router(options, api_requests);
    async move {
        tokio::task::spawn_local(api::serve(fetchers, api_incoming));
        let tokio_listener = |listener: std::net::TcpListener| {
            listener
                .set_nonblocking(true)
                .expect("Failed to make a listener non-blocking");
            TcpListener::from_std(listener).expect("Failed to hand a listener to tokio")
        };
        match listen {
            Listen::Tls {
                https,
                https_port,
                tls,
                http,
            } => {
                tokio::join!(
                    serve_https(tokio_listener(https), tls, router),
                    serve_http(tokio_listener(http), trust_router(https_port)),
                );
            }
            Listen::Plain { http } => serve_http(tokio_listener(http), router).await,
        }
    }
}

/// robot_server in a simulation a C++ replay assembled (`aos/events/tokio_simulation.h`):
/// serves the pages from `data_dir` and `/api` on plain HTTP on `port`, and proxies `/ws` to a
/// web_proxy on `web_proxy_port`.  `config` is a copy of the configuration, as a flatbuffer of
/// `config_size` bytes.  Returns false, having logged why, if it can't.
///
/// # Safety
///
/// `simulation` is a live, unrun `aos_tokio_simulation_t`, `event_loop` one of its event loops,
/// `config` points to `config_size` bytes, and `data_dir` is a C string.
#[no_mangle]
pub unsafe extern "C" fn robot_server_add_to_simulation(
    simulation: *mut aos_events_tokio::TokioSimulation,
    event_loop: *mut aos_events_event_loop_c::aos_event_loop_t,
    config: *const u8,
    config_size: usize,
    data_dir: *const std::os::raw::c_char,
    port: u16,
    web_proxy_port: u16,
) -> bool {
    use aos_flatbuffers::Flatbuffer;
    // SAFETY: As the caller promises.
    let Some(config) = (unsafe { leak_config(config, config_size) }) else {
        return false;
    };
    // SAFETY: As the caller promises.
    let data_dir = match unsafe { std::ffi::CStr::from_ptr(data_dir) }.to_str() {
        Ok(data_dir) => PathBuf::from(data_dir),
        Err(_) => {
            log::error!("robot_server: data_dir isn't UTF-8");
            return false;
        }
    };
    let http = match std::net::TcpListener::bind(("::", port)) {
        Ok(http) => http,
        Err(error) => {
            log::error!("robot_server: can't listen on {port}: {error}");
            return false;
        }
    };
    // SAFETY: As the caller promises.
    let simulation = unsafe { &mut *simulation };
    // SAFETY: As the caller promises.
    let aos = unsafe { simulation.setup().handle(event_loop) };
    let server = start(
        &aos,
        config.message(),
        Options {
            data_dir,
            web_proxy_ws: format!("ws://127.0.0.1:{web_proxy_port}/ws"),
            foxglove_ws: "ws://127.0.0.1:8765".to_string(),
            camera_streamer_ws: "ws://127.0.0.1:1190".to_string(),
        },
        api::ApiOptions::default(),
        Listen::Plain { http },
    );
    simulation.spawn(server);
    true
}

/// Why robot_server couldn't start in a simulation: logged, and on stderr, since a C++
/// simulation may not have set Rust's logging up.
macro_rules! startup_error {
    ($($arg:tt)*) => {{
        log::error!($($arg)*);
        eprintln!($($arg)*);
    }};
}

/// robot_server in a simulation a C++ test or replay assembled, as on the robot: started with
/// `argv`, robot_server's own command line (`argc` arguments, the first the program's name),
/// serving HTTPS and HTTP on its ports with the certificate in its `--tls-dir`, with its
/// `--sendable-channels` and `--session-*`.  `--config` and `--user` are ignored: the
/// simulation's configuration is `config`, a flatbuffer of `config_size` bytes, and nothing
/// runs as root.  Returns false, having logged why, if it can't start.
///
/// # Safety
///
/// `simulation` is a live, unrun `aos_tokio_simulation_t`, `event_loop` one of its event loops,
/// `config` points to `config_size` bytes, and `argv` to `argc` C strings.
#[no_mangle]
pub unsafe extern "C" fn robot_server_add_to_simulation_with_flags(
    simulation: *mut aos_events_tokio::TokioSimulation,
    event_loop: *mut aos_events_event_loop_c::aos_event_loop_t,
    config: *const u8,
    config_size: usize,
    argc: std::os::raw::c_int,
    argv: *const *const std::os::raw::c_char,
) -> bool {
    use aos_flatbuffers::Flatbuffer;
    use clap::Parser;
    // SAFETY: As the caller promises.
    let Some(config) = (unsafe { leak_config(config, config_size) }) else {
        return false;
    };
    let mut args = Vec::new();
    for i in 0..argc.max(0) as usize {
        // SAFETY: As the caller promises.
        match unsafe { std::ffi::CStr::from_ptr(*argv.add(i)) }.to_str() {
            Ok(arg) => args.push(arg.to_string()),
            Err(_) => {
                startup_error!("robot_server: argument {i} isn't UTF-8");
                return false;
            }
        }
    }
    let flags = match Flags::try_parse_from(&args) {
        Ok(flags) => flags,
        Err(error) => {
            startup_error!("robot_server: {error}");
            return false;
        }
    };
    let bind = |port: u16, what: &str| match std::net::TcpListener::bind(("::", port)) {
        Ok(listener) => Some(listener),
        Err(error) => {
            startup_error!("robot_server: can't listen for {what} on {port}: {error}");
            None
        }
    };
    let (Some(https), Some(http)) = (
        bind(flags.https_port, "HTTPS"),
        bind(flags.http_port, "HTTP"),
    ) else {
        return false;
    };
    let tls = match tls_config(&flags.tls_dir) {
        Ok(tls) => tls,
        Err(error) => {
            startup_error!(
                "robot_server: no certificate in {}: {error}",
                flags.tls_dir.display()
            );
            return false;
        }
    };
    // SAFETY: As the caller promises.
    let simulation = unsafe { &mut *simulation };
    // SAFETY: As the caller promises.
    let aos = unsafe { simulation.setup().handle(event_loop) };
    let server = start(
        &aos,
        config.message(),
        Options {
            data_dir: flags.data_dir.clone(),
            web_proxy_ws: format!("ws://127.0.0.1:{}/ws", flags.web_proxy_port),
            foxglove_ws: format!("ws://127.0.0.1:{}", flags.foxglove_port),
            camera_streamer_ws: format!("ws://127.0.0.1:{}", flags.camera_streamer_port),
        },
        flags.api_options(),
        Listen::Tls {
            https,
            https_port: flags.https_port,
            tls,
            http,
        },
    );
    simulation.spawn(server);
    true
}

/// The configuration a simulation hands robot_server, kept for as long as the process runs, as
/// the server's tasks are; None, having logged why, if it isn't one.
///
/// # Safety
///
/// `config` points to `config_size` bytes.
unsafe fn leak_config(
    config: *const u8,
    config_size: usize,
) -> Option<
    &'static aos_flatbuffers::NonSizePrefixedFlatbuffer<
        aos_configuration_fbs::aos::Configuration<'static>,
        Vec<u8>,
    >,
> {
    // SAFETY: As the caller promises.
    let bytes = unsafe { std::slice::from_raw_parts(config, config_size) }.to_vec();
    match aos_flatbuffers::NonSizePrefixedFlatbuffer::<
        aos_configuration_fbs::aos::Configuration<'static>,
        Vec<u8>,
    >::new(bytes)
    {
        Ok(config) => Some(Box::leak(Box::new(config))),
        Err(error) => {
            startup_error!("robot_server: the configuration isn't one: {error}");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustls::{pki_types::ServerName, ClientConfig, RootCertStore};
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio_rustls::TlsConnector;

    fn testdata(name: &str) -> PathBuf {
        aos_testing_path::artifact_path(Path::new("aos/network/robot_server/testdata")).join(name)
    }

    /// A directory holding certificate `which` ("a" or "b") under the names the server reads.
    /// Writes certificate `which` into `dir`.  It copies the contents, not the files: runfiles
    /// can be read-only (they are on remote executors), and fs::copy would copy that mode, so
    /// the next install couldn't overwrite them.
    fn install_cert(dir: &Path, which: &str) {
        for (from, to) in [
            (format!("server-{which}-chain.crt"), CHAIN_FILE),
            (format!("server-{which}.key"), KEY_FILE),
        ] {
            fs::write(dir.join(to), fs::read(testdata(&from)).unwrap()).unwrap();
        }
    }

    fn temp_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("robot_server_test_{name}_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn client_tls() -> TlsConnector {
        let mut roots = RootCertStore::empty();
        for cert in CertificateDer::pem_file_iter(testdata("ca.crt")).unwrap() {
            roots.add(cert.unwrap()).unwrap();
        }
        let config =
            ClientConfig::builder_with_provider(Arc::new(rustls::crypto::ring::default_provider()))
                .with_safe_default_protocol_versions()
                .unwrap()
                .with_root_certificates(roots)
                .with_no_client_auth();
        TlsConnector::from(Arc::new(config))
    }

    async fn start(tls_dir: &Path, options: Options) -> SocketAddr {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        tokio::spawn(serve_https(
            listener,
            tls_config(tls_dir).unwrap(),
            router(options, mpsc::unbounded_channel().0),
        ));
        address
    }

    fn options(data_dir: &Path) -> Options {
        Options {
            data_dir: data_dir.to_path_buf(),
            web_proxy_ws: "ws://127.0.0.1:1/ws".to_string(),
            foxglove_ws: "ws://127.0.0.1:1".to_string(),
            camera_streamer_ws: "ws://127.0.0.1:1".to_string(),
        }
    }

    /// GETs `path` over TLS, returning the response and the server's leaf certificate.
    async fn get(address: SocketAddr, path: &str) -> (String, Vec<u8>) {
        let tcp = tokio::net::TcpStream::connect(address).await.unwrap();
        let mut stream = client_tls()
            .connect(ServerName::try_from("127.0.0.1").unwrap(), tcp)
            .await
            .unwrap();
        let leaf = stream.get_ref().1.peer_certificates().unwrap()[0].to_vec();
        stream
            .write_all(
                format!("GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
                    .as_bytes(),
            )
            .await
            .unwrap();
        let mut response = String::new();
        stream.read_to_string(&mut response).await.unwrap();
        (response, leaf)
    }

    fn leaf_of(which: &str) -> Vec<u8> {
        CertificateDer::pem_file_iter(testdata(&format!("server-{which}-chain.crt")))
            .unwrap()
            .next()
            .unwrap()
            .unwrap()
            .to_vec()
    }

    /// The pages are served over TLS, with the robot's certificate.
    #[tokio::test]
    async fn serves_pages_over_tls() {
        let tls_dir = temp_dir("pages_tls");
        install_cert(&tls_dir, "a");
        let data_dir = temp_dir("pages_data");
        fs::write(data_dir.join("field.html"), "<p>field</p>").unwrap();
        let address = start(&tls_dir, options(&data_dir)).await;

        let (response, leaf) = get(address, "/field.html").await;
        assert!(response.starts_with("HTTP/1.1 200"), "{response}");
        assert!(response.ends_with("<p>field</p>"), "{response}");
        // Pages change with each deploy, so browsers must check before reusing them.
        assert!(
            response
                .to_ascii_lowercase()
                .contains("cache-control: no-cache"),
            "{response}"
        );
        assert_eq!(leaf, leaf_of("a"));

        let (response, _) = get(address, "/healthz").await;
        assert!(response.ends_with("ok"), "{response}");
    }

    /// A reissued certificate is served from the next connection on, without a restart.
    #[tokio::test]
    async fn reloads_a_reissued_certificate() {
        let tls_dir = temp_dir("reload_tls");
        install_cert(&tls_dir, "a");
        let data_dir = temp_dir("reload_data");
        let address = start(&tls_dir, options(&data_dir)).await;

        assert_eq!(get(address, "/healthz").await.1, leaf_of("a"));
        // Modification times can be coarse; make sure the new files look newer.
        std::thread::sleep(std::time::Duration::from_millis(20));
        install_cert(&tls_dir, "b");
        let later = SystemTime::now() + std::time::Duration::from_secs(1);
        for file in [CHAIN_FILE, KEY_FILE] {
            fs::File::options()
                .append(true)
                .open(tls_dir.join(file))
                .unwrap()
                .set_modified(later)
                .unwrap();
        }
        assert_eq!(get(address, "/healthz").await.1, leaf_of("b"));
    }

    /// A websocket to /foxglove reaches the server behind it, with Foxglove's subprotocol passed
    /// through both ways, and messages flow in both directions.
    #[tokio::test]
    async fn proxies_foxglove_websocket() {
        // The "Foxglove server": accepts one websocket, checks the subprotocol, echoes.
        let upstream = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let upstream_address = upstream.local_addr().unwrap();
        tokio::spawn(async move {
            let (tcp, _) = upstream.accept().await.unwrap();
            let callback =
                |request: &tungstenite::handshake::server::Request,
                 mut response: tungstenite::handshake::server::Response| {
                    assert_eq!(
                        request.headers().get("Sec-WebSocket-Protocol").unwrap(),
                        FOXGLOVE_SUBPROTOCOL
                    );
                    response.headers_mut().insert(
                        "Sec-WebSocket-Protocol",
                        FOXGLOVE_SUBPROTOCOL.parse().unwrap(),
                    );
                    Ok(response)
                };
            let mut socket = tokio_tungstenite::accept_hdr_async(tcp, callback)
                .await
                .unwrap();
            while let Some(Ok(message)) = socket.next().await {
                if message.is_text() || message.is_binary() {
                    socket.send(message).await.unwrap();
                }
            }
        });

        let tls_dir = temp_dir("ws_tls");
        install_cert(&tls_dir, "a");
        let data_dir = temp_dir("ws_data");
        let address = start(
            &tls_dir,
            Options {
                data_dir: data_dir.clone(),
                web_proxy_ws: "ws://127.0.0.1:1/ws".to_string(),
                foxglove_ws: format!("ws://{upstream_address}"),
                camera_streamer_ws: "ws://127.0.0.1:1".to_string(),
            },
        )
        .await;

        let tcp = tokio::net::TcpStream::connect(address).await.unwrap();
        let tls = client_tls()
            .connect(ServerName::try_from("127.0.0.1").unwrap(), tcp)
            .await
            .unwrap();
        let mut request = format!("wss://127.0.0.1:{}/foxglove", address.port())
            .into_client_request()
            .unwrap();
        request.headers_mut().insert(
            "Sec-WebSocket-Protocol",
            FOXGLOVE_SUBPROTOCOL.parse().unwrap(),
        );
        let (mut socket, response) = tokio_tungstenite::client_async(request, tls).await.unwrap();
        assert_eq!(
            response.headers().get("Sec-WebSocket-Protocol").unwrap(),
            FOXGLOVE_SUBPROTOCOL
        );
        socket
            .send(tungstenite::Message::text("hello"))
            .await
            .unwrap();
        let echoed = socket.next().await.unwrap().unwrap();
        assert_eq!(echoed.to_text().unwrap(), "hello");
        socket
            .send(tungstenite::Message::binary(vec![1u8, 2, 3]))
            .await
            .unwrap();
        let echoed = socket.next().await.unwrap().unwrap();
        assert_eq!(echoed.into_data().to_vec(), vec![1u8, 2, 3]);
    }

    /// Fetches `path` over plain HTTP from a trust router for `https_port`.
    async fn fetch_trust_page(https_port: u16, path: &str) -> String {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        tokio::spawn(serve_http(listener, trust_router(https_port)));
        let mut tcp = tokio::net::TcpStream::connect(address).await.unwrap();
        tcp.write_all(
            format!("GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
                .as_bytes(),
        )
        .await
        .unwrap();
        let mut response = String::new();
        tcp.read_to_string(&mut response).await.unwrap();
        assert!(response.starts_with("HTTP/1.1 200"), "{response}");
        response
    }

    /// The plain-HTTP page probes the HTTPS port it was told about, goes on to the same path
    /// there once that works, and says how to trust the robot until it does.
    #[tokio::test]
    async fn trust_page_redirects_when_trusted() {
        let response = fetch_trust_page(8443, "/").await;
        assert!(response.contains("`https://${host}:8443`"), "{response}");
        assert!(response.contains("fetch(`${https}/healthz`"), "{response}");
        assert!(
            response.contains("https + location.pathname + location.search + location.hash"),
            "{response}"
        );
        assert!(response.contains("location.replace(target)"), "{response}");
        assert!(response.contains("python3 trust_orin.py"), "{response}");
        // Retries until the robot is trusted.
        assert!(response.contains("setTimeout(check"), "{response}");
    }

    /// The bare name has no certificate, so the page probes and redirects to the .local name.
    #[tokio::test]
    async fn trust_page_uses_the_certificates_name() {
        let response = fetch_trust_page(443, "/").await;
        assert!(response.contains("/^orin-[0-9]+-[0-9]$/"), "{response}");
        assert!(
            response.contains("`${location.hostname}.local`"),
            "{response}"
        );
    }

    /// Every path gets the page, so the redirect keeps the page that was asked for.
    #[tokio::test]
    async fn trust_page_serves_every_path() {
        let response = fetch_trust_page(443, "/field.html?x=1").await;
        assert!(response.contains("location.replace(target)"), "{response}");
    }

    /// On 443 the page's URLs leave the port out, as a browser would.
    #[tokio::test]
    async fn trust_page_omits_the_default_port() {
        let response = fetch_trust_page(443, "/").await;
        assert!(response.contains("`https://${host}`"), "{response}");
        assert!(!response.contains(":443"), "{response}");
    }

    /// Dropping to a user that doesn't exist fails, naming it, instead of carrying on as root.
    #[test]
    fn drop_privileges_to_unknown_user_fails() {
        let error = drop_privileges("no-such-user-robot-server-test").unwrap_err();
        assert!(
            error.to_string().contains("no-such-user-robot-server-test"),
            "{error}"
        );
    }
}
