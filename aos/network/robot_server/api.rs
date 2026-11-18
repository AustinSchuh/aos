//! GET requests for what is on the robot's channels, as JSON, which MCP's tools ask for too.
//!
//! ```text
//! GET /api/channels
//!     [{"name": "/constants", "type": "frc.vision.CameraConstants", "max_size": 4096,
//!       "frequency": 1, "fetchable": true}, ...]
//! GET /api/latest?name=/constants&type=frc.vision.CameraConstants[&max_vector_size=100]
//!     {"name": "/constants", "type": "frc.vision.CameraConstants",
//!      "monotonic_event_time_ns": 123, "realtime_event_time_ns": 456, "message": {...}}
//! POST /api/send?name=/calibration&type=frc.vision.calibration.Command, the message as JSON
//!     {"sent": true}
//! ```
//!
//! Only the channels robot_server is told it may send on (`--sendable-channels`) take a POST.
//! A command that starts or ends a [`SessionApp`]'s session also starts or stops apps.
//!
//! The router's handlers run on any of tokio's threads, but AOS's fetchers live on the event
//! loop's, so they ask an [`ApiRequest`] of [`serve()`], which runs there, and wait for its
//! answer.
//! Channels too big to want as JSON (images) aren't fetchable.

use std::{cell::RefCell, collections::HashMap, ffi::CStr, rc::Rc, time::Duration};

use aos_configuration::{AsChannel, ChannelExt, ConfigurationExt, Node};
use aos_configuration_fbs::aos::{Channel, Configuration, ReadMethod};
use aos_events_event_loop_c::aos_configuration_node_name;
use aos_events_tokio::{AosHandle, RawFetcher, RawSender, Timer};

use axum::{
    extract::Query,
    http::{header, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Router,
};

use crate::session::{Effect, SessionApp, StarterCommand};
use serde_json::json;
use tokio::sync::{mpsc, oneshot};

/// The largest message a channel may hold for its latest to be fetchable.
pub const MAX_FETCHABLE_SIZE: u32 = 256 * 1024;

/// How many elements of a vector the JSON keeps, unless the request says otherwise; longer ones
/// are left out.
pub const DEFAULT_MAX_VECTOR_SIZE: usize = 100;

/// A channel, as `/api/channels` lists it.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelInfo {
    pub name: String,
    pub type_: String,
    pub max_size: u32,
    pub frequency: i32,
    /// Whether `/api/latest` will fetch it: false for channels whose messages are too big, or
    /// that are read by pinning (their readers are counted).
    pub fetchable: bool,
}

/// A channel's latest message.
#[derive(Clone, Debug, PartialEq)]
pub struct Latest {
    pub monotonic_event_time_ns: i64,
    pub realtime_event_time_ns: i64,
    /// How long ago it was sent, on the event loop's clock.
    pub age_ns: i64,
    /// The message, as JSON.
    pub json: String,
}

/// Why there's no latest message to give.
#[derive(Clone, Debug, PartialEq)]
pub enum LatestError {
    /// The configuration has no such channel.
    NoChannel,
    /// The channel isn't fetchable (see [`ChannelInfo::fetchable`]).
    NotFetchable(String),
    /// Nothing has been sent on it yet.
    NoMessage,
    /// The message doesn't match the channel's schema, or it has none.
    NotJson,
}

/// Why a message wasn't sent.
#[derive(Clone, Debug, PartialEq)]
pub enum SendError {
    /// The configuration has no such channel.
    NoChannel,
    /// robot_server isn't to send on it.
    NotSendable,
    /// The JSON isn't a message of the channel's type.
    NotAMessage,
    /// The session's app didn't say it was ready.
    NotReady(String),
    /// AOS refused it, with this `aos_const_raw_sender_error_*` code.
    Failed(i32),
}

/// What robot_server may send, and the apps the page starts and stops.
#[derive(Clone, Debug, Default)]
pub struct ApiOptions {
    pub sendable: Vec<(String, String)>,
    pub session: Option<SessionApp>,
}

/// How long a session's app has to say it's ready.
const READY_TIMEOUT: Duration = Duration::from_secs(15);
/// How often a session's start looks for its app being ready.
const READY_POLL: Duration = Duration::from_millis(100);

/// What a handler asks of the event loop's thread.
#[derive(Debug)]
pub enum ApiRequest {
    Channels {
        reply: oneshot::Sender<Vec<ChannelInfo>>,
    },
    Latest {
        name: String,
        type_: String,
        max_vector_size: usize,
        reply: oneshot::Sender<Result<Latest, LatestError>>,
    },
    Send {
        name: String,
        type_: String,
        json: String,
        reply: oneshot::Sender<Result<(), SendError>>,
    },
}

fn json_response(status: StatusCode, body: String) -> Response {
    (status, [(header::CONTENT_TYPE, "application/json")], body).into_response()
}

fn error(status: StatusCode, message: &str) -> Response {
    json_response(status, json!({ "error": message }).to_string())
}

fn gone() -> Response {
    error(
        StatusCode::SERVICE_UNAVAILABLE,
        "The robot's event loop isn't answering",
    )
}

async fn channels(requests: mpsc::UnboundedSender<ApiRequest>) -> Response {
    let (reply, answer) = oneshot::channel();
    if requests.send(ApiRequest::Channels { reply }).is_err() {
        return gone();
    }
    let Ok(channels) = answer.await else {
        return gone();
    };
    let list: Vec<_> = channels
        .iter()
        .map(|channel| {
            json!({
                "name": channel.name,
                "type": channel.type_,
                "max_size": channel.max_size,
                "frequency": channel.frequency,
                "fetchable": channel.fetchable,
            })
        })
        .collect();
    json_response(StatusCode::OK, serde_json::Value::from(list).to_string())
}

async fn latest(
    requests: mpsc::UnboundedSender<ApiRequest>,
    query: HashMap<String, String>,
) -> Response {
    let (Some(name), Some(type_)) = (query.get("name"), query.get("type")) else {
        return error(
            StatusCode::BAD_REQUEST,
            "Say which channel: ?name=...&type=...",
        );
    };
    let max_vector_size = match query.get("max_vector_size") {
        None => DEFAULT_MAX_VECTOR_SIZE,
        Some(size) => match size.parse() {
            Ok(size) => size,
            Err(_) => return error(StatusCode::BAD_REQUEST, "max_vector_size isn't a number"),
        },
    };
    let (reply, answer) = oneshot::channel();
    let request = ApiRequest::Latest {
        name: name.clone(),
        type_: type_.clone(),
        max_vector_size,
        reply,
    };
    if requests.send(request).is_err() {
        return gone();
    }
    let Ok(result) = answer.await else {
        return gone();
    };
    match result {
        // The message is JSON already: put it in as it is, rather than parse it to write it again.
        Ok(latest) => json_response(
            StatusCode::OK,
            format!(
                "{{\"name\":{},\"type\":{},\"monotonic_event_time_ns\":{},\
                 \"realtime_event_time_ns\":{},\"message\":{}}}",
                json!(name),
                json!(type_),
                latest.monotonic_event_time_ns,
                latest.realtime_event_time_ns,
                latest.json
            ),
        ),
        Err(LatestError::NoChannel) => error(
            StatusCode::NOT_FOUND,
            &format!("There's no channel {name} {type_}"),
        ),
        Err(LatestError::NotFetchable(why)) => error(StatusCode::FORBIDDEN, &why),
        Err(LatestError::NoMessage) => error(
            StatusCode::NOT_FOUND,
            &format!("Nothing has been sent on {name} {type_} yet"),
        ),
        Err(LatestError::NotJson) => error(
            StatusCode::INTERNAL_SERVER_ERROR,
            &format!("{name} {type_}'s latest message doesn't match its schema"),
        ),
    }
}

async fn send(
    requests: mpsc::UnboundedSender<ApiRequest>,
    query: HashMap<String, String>,
    json: String,
) -> Response {
    let (Some(name), Some(type_)) = (query.get("name"), query.get("type")) else {
        return error(
            StatusCode::BAD_REQUEST,
            "Say which channel: ?name=...&type=...",
        );
    };
    let (reply, answer) = oneshot::channel();
    let request = ApiRequest::Send {
        name: name.clone(),
        type_: type_.clone(),
        json,
        reply,
    };
    if requests.send(request).is_err() {
        return gone();
    }
    let Ok(result) = answer.await else {
        return gone();
    };
    match result {
        Ok(()) => json_response(StatusCode::OK, json!({ "sent": true }).to_string()),
        Err(SendError::NoChannel) => error(
            StatusCode::NOT_FOUND,
            &format!("There's no channel {name} {type_}"),
        ),
        Err(SendError::NotSendable) => error(
            StatusCode::FORBIDDEN,
            &format!("robot_server doesn't send on {name} {type_}"),
        ),
        Err(SendError::NotAMessage) => error(
            StatusCode::BAD_REQUEST,
            &format!("That isn't a {type_}, as JSON"),
        ),
        Err(SendError::NotReady(app)) => {
            error(StatusCode::GATEWAY_TIMEOUT, &format!("{app} didn't start"))
        }
        Err(SendError::Failed(code)) => error(
            StatusCode::INTERNAL_SERVER_ERROR,
            &format!("Sending failed: AOS error {code}"),
        ),
    }
}

/// The `/api` routes, asking `requests` for what's on the channels.
pub fn router(requests: mpsc::UnboundedSender<ApiRequest>) -> Router {
    let for_latest = requests.clone();
    let for_send = requests.clone();
    Router::new()
        .route("/api/channels", get(move || channels(requests.clone())))
        .route(
            "/api/latest",
            get(move |Query(query): Query<HashMap<String, String>>| {
                latest(for_latest.clone(), query)
            }),
        )
        .route(
            "/api/send",
            post(
                move |Query(query): Query<HashMap<String, String>>, body: String| {
                    send(for_send.clone(), query, body)
                },
            ),
        )
}

/// The event loop's node, in `config`: the configuration this serves from may be a copy of the
/// event loop's own.
fn node_of(aos: &AosHandle, config: Configuration<'static>) -> Option<&'static Node> {
    let node = aos.node()?;
    // SAFETY: Nodes always have names, and the event loop's outlives this call.
    let name = unsafe { CStr::from_ptr(aos_configuration_node_name(node)) };
    config.get_node(name.to_str().ok()?).ok()
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

/// A channel `get_channel` found, as `/api` keys it.
fn mapped_key(channel: &aos_configuration::Channel) -> ChannelKey {
    (
        channel.name().unwrap_or_default().to_string(),
        channel.type_().unwrap_or_default().to_string(),
    )
}

/// What `/api` serves from: the configuration, a fetcher on every channel it may fetch, and a
/// sender on every channel it may send on.  AOS makes fetchers and senders only before the event
/// loop runs, so they're all made up front.
pub struct Fetchers {
    config: Configuration<'static>,
    node: Option<&'static Node>,
    fetchers: HashMap<ChannelKey, RawFetcher>,
    senders: HashMap<ChannelKey, RawSender>,
    session: Option<SessionApp>,
    starter: Option<(&'static aos_configuration::Channel, RawSender)>,
    aos: AosHandle,
    /// What a session's start waits on, while its app gets ready.  Made before the event loop
    /// runs, as AOS's timers must be, and shared by any starts at once.
    ready_timer: Rc<tokio::sync::Mutex<Timer>>,
}

impl Fetchers {
    /// Makes the fetchers and senders on `aos`'s event loop, which mustn't be running yet.
    ///
    /// # Panics
    ///
    /// If a channel `options` names isn't in the configuration.
    pub fn new(aos: &AosHandle, config: Configuration<'static>, options: ApiOptions) -> Self {
        let node = node_of(aos, config);
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
        let mut sendable = options.sendable.clone();
        if let Some(session) = &options.session {
            sendable.push(session.commands.clone());
        }
        let senders = sendable
            .iter()
            .map(|(name, type_)| {
                let channel = config
                    .get_channel(name, type_, "robot_server", node)
                    .unwrap_or_else(|_| panic!("There's no channel {name} {type_} to send on"));
                let key = mapped_key(channel);
                let sender = aos.make_sender(&key.0, &key.1);
                (key, sender)
            })
            .collect();
        let starter = options.session.as_ref().map(|_| {
            let channel = config
                .get_channel("/aos", "aos.starter.StarterRpc", "robot_server", node)
                .expect("There's no /aos aos.starter.StarterRpc to start and stop apps with");
            let (name, type_) = mapped_key(channel);
            (channel, aos.make_sender(&name, &type_))
        });
        Self {
            config,
            node,
            fetchers,
            senders,
            session: options.session,
            starter,
            aos: aos.clone(),
            ready_timer: Rc::new(tokio::sync::Mutex::new(aos.make_timer())),
        }
    }

    /// `name` `type_`, after the configuration's maps, as `/api` keys it.
    fn mapped(
        &self,
        name: &str,
        type_: &str,
    ) -> Option<(&'static aos_configuration::Channel, ChannelKey)> {
        let channel = self
            .config
            .get_channel(name, type_, "robot_server", self.node)
            .ok()?;
        Some((channel, mapped_key(channel)))
    }

    /// Sends `json`, a message, on `name` `type_`.
    fn send(&mut self, name: &str, type_: &str, json: &str) -> Result<(), SendError> {
        let (channel, key) = self.mapped(name, type_).ok_or(SendError::NoChannel)?;
        let sender = self.senders.get_mut(&key).ok_or(SendError::NotSendable)?;
        let message = channel
            .json_to_message(json)
            .ok_or(SendError::NotAMessage)?;
        match sender.send_copy(&message) {
            0 => Ok(()),
            code => Err(SendError::Failed(code)),
        }
    }

    /// Has starter carry out `commands`.
    fn starter(&mut self, commands: &[StarterCommand]) {
        let Some((channel, sender)) = &mut self.starter else {
            return;
        };
        for command in commands {
            log::info!("{} {}", command.command, command.app);
            let message = channel
                .json_to_message(&command.json())
                .expect("A StarterRpc is a StarterRpc");
            if sender.send_copy(&message) != 0 {
                log::error!(
                    "Failed to ask starter to {} {}",
                    command.command,
                    command.app
                );
            }
        }
    }

    /// When the session's app last said it was ready.
    fn ready_time(&mut self) -> i64 {
        let Some(session) = &self.session else {
            return i64::MIN;
        };
        let ready = session.ready.clone();
        let Some((_, key)) = self.mapped(&ready.0, &ready.1) else {
            return i64::MIN;
        };
        let Some(fetcher) = self.fetchers.get_mut(&key) else {
            return i64::MIN;
        };
        fetcher.fetch();
        let message = fetcher.message();
        match message.data() {
            Some(_) => message.monotonic_event_time_ns(),
            None => i64::MIN,
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
            age_ns: self.aos.monotonic_now_ns() - message.monotonic_event_time_ns(),
            json: channel
                .message_to_json(data, max_vector_size)
                .ok_or(LatestError::NotJson)?,
        })
    }
}

/// Starts a session: stops what's in the way, starts its app, and once it says it's ready, sends
/// it `json`.
async fn start_session(
    api: Rc<RefCell<Fetchers>>,
    session: SessionApp,
    name: String,
    type_: String,
    json: String,
    reply: oneshot::Sender<Result<(), SendError>>,
) {
    let before = api.borrow_mut().ready_time();
    api.borrow_mut().starter(&session.starting());
    // On the event loop's clock, which is simulated time in a simulation.
    let (aos, timer) = {
        let api = api.borrow();
        (api.aos.clone(), api.ready_timer.clone())
    };
    let deadline_ns = aos.monotonic_now_ns() + READY_TIMEOUT.as_nanos() as i64;
    while api.borrow_mut().ready_time() <= before {
        let now_ns = aos.monotonic_now_ns();
        if now_ns > deadline_ns {
            log::error!("{} didn't start", session.app);
            let _ = reply.send(Err(SendError::NotReady(session.app.clone())));
            return;
        }
        timer
            .lock()
            .await
            .sleep_until(now_ns + READY_POLL.as_nanos() as i64)
            .await;
    }
    let _ = reply.send(api.borrow_mut().send(&name, &type_, &json));
}

/// Answers `/api`'s requests, on the event loop's thread where the fetchers live.
pub async fn serve(api: Fetchers, mut requests: mpsc::UnboundedReceiver<ApiRequest>) {
    let api = Rc::new(RefCell::new(api));
    // A session robot_server lost track of mustn't leave the robot without the apps it stopped.
    let session = api.borrow().session.clone();
    if let Some(session) = &session {
        api.borrow_mut().starter(&session.ending());
    }
    while let Some(request) = requests.recv().await {
        match request {
            ApiRequest::Channels { reply } => {
                let _ = reply.send(api.borrow().channels());
            }
            ApiRequest::Latest {
                name,
                type_,
                max_vector_size,
                reply,
            } => {
                let _ = reply.send(api.borrow_mut().latest(&name, &type_, max_vector_size));
            }
            ApiRequest::Send {
                name,
                type_,
                json,
                reply,
            } => {
                let effect = match (&session, api.borrow().mapped(&name, &type_)) {
                    (Some(session), Some((_, key))) => serde_json::from_str(&json)
                        .map(|value| session.effect(&key, &value))
                        .unwrap_or(Effect::None),
                    _ => Effect::None,
                };
                match (effect, &session) {
                    (Effect::Starts, Some(session)) => {
                        tokio::task::spawn_local(start_session(
                            api.clone(),
                            session.clone(),
                            name,
                            type_,
                            json,
                            reply,
                        ));
                    }
                    (Effect::Ends, Some(session)) => {
                        let result = api.borrow_mut().send(&name, &type_, &json);
                        api.borrow_mut().starter(&session.ending());
                        let _ = reply.send(result);
                    }
                    _ => {
                        let _ = reply.send(api.borrow_mut().send(&name, &type_, &json));
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt;

    /// Answers requests as a robot with one channel, /constants, whose latest message is
    /// `{"a": 1}`, and an image channel.
    fn fake_robot() -> mpsc::UnboundedSender<ApiRequest> {
        let (requests, mut incoming) = mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Some(request) = incoming.recv().await {
                match request {
                    ApiRequest::Channels { reply } => {
                        let _ = reply.send(vec![ChannelInfo {
                            name: "/constants".into(),
                            type_: "frc.Constants".into(),
                            max_size: 4096,
                            frequency: 1,
                            fetchable: true,
                        }]);
                    }
                    ApiRequest::Latest {
                        name,
                        type_,
                        max_vector_size,
                        reply,
                    } => {
                        let _ = reply.send(match (name.as_str(), type_.as_str()) {
                            ("/constants", "frc.Constants") => Ok(Latest {
                                monotonic_event_time_ns: 12,
                                realtime_event_time_ns: 34,
                                age_ns: 0,
                                json: format!(
                                    "{{\"a\": 1, \"max_vector_size\": {max_vector_size}}}"
                                ),
                            }),
                            ("/camera0", "frc.vision.CameraImage") => {
                                Err(LatestError::NotFetchable("Too big".into()))
                            }
                            ("/empty", _) => Err(LatestError::NoMessage),
                            _ => Err(LatestError::NoChannel),
                        });
                    }
                    ApiRequest::Send {
                        name, json, reply, ..
                    } => {
                        let _ = reply.send(match name.as_str() {
                            "/calibration" if json.starts_with('{') => Ok(()),
                            "/calibration" => Err(SendError::NotAMessage),
                            "/constants" => Err(SendError::NotSendable),
                            _ => Err(SendError::NoChannel),
                        });
                    }
                }
            }
        });
        requests
    }

    async fn get_json(uri: &str) -> (StatusCode, serde_json::Value) {
        let response = router(fake_robot())
            .oneshot(Request::get(uri).body(Body::empty()).unwrap())
            .await
            .unwrap();
        let status = response.status();
        assert_eq!(
            response.headers()[header::CONTENT_TYPE],
            "application/json",
            "{uri}"
        );
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        (status, serde_json::from_slice(&body).unwrap())
    }

    #[tokio::test]
    async fn lists_the_channels() {
        let (status, body) = get_json("/api/channels").await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(
            body,
            json!([{"name": "/constants", "type": "frc.Constants", "max_size": 4096,
                    "frequency": 1, "fetchable": true}])
        );
    }

    #[tokio::test]
    async fn gives_a_channels_latest_message() {
        let (status, body) = get_json("/api/latest?name=%2Fconstants&type=frc.Constants").await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(
            body,
            json!({"name": "/constants", "type": "frc.Constants",
                   "monotonic_event_time_ns": 12, "realtime_event_time_ns": 34,
                   "message": {"a": 1, "max_vector_size": DEFAULT_MAX_VECTOR_SIZE}})
        );
        let (_, body) =
            get_json("/api/latest?name=/constants&type=frc.Constants&max_vector_size=5").await;
        assert_eq!(body["message"]["max_vector_size"], 5);
    }

    #[tokio::test]
    async fn says_why_there_is_nothing() {
        for (uri, status) in [
            ("/api/latest?name=/nope&type=x", StatusCode::NOT_FOUND),
            ("/api/latest?name=/empty&type=x", StatusCode::NOT_FOUND),
            (
                "/api/latest?name=/camera0&type=frc.vision.CameraImage",
                StatusCode::FORBIDDEN,
            ),
            ("/api/latest?name=/constants", StatusCode::BAD_REQUEST),
            (
                "/api/latest?name=/constants&type=frc.Constants&max_vector_size=lots",
                StatusCode::BAD_REQUEST,
            ),
        ] {
            let (got, body) = get_json(uri).await;
            assert_eq!(got, status, "{uri}: {body}");
            assert!(body["error"].is_string(), "{uri}: {body}");
        }
    }

    async fn post_json(uri: &str, body: &str) -> (StatusCode, serde_json::Value) {
        let response = router(fake_robot())
            .oneshot(
                Request::post(uri)
                    .body(Body::from(body.to_string()))
                    .unwrap(),
            )
            .await
            .unwrap();
        let status = response.status();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        (status, serde_json::from_slice(&body).unwrap())
    }

    #[tokio::test]
    async fn sends_where_it_may() {
        let (status, body) = post_json(
            "/api/send?name=/calibration&type=frc.vision.calibration.Command",
            r#"{"action": "UNDO"}"#,
        )
        .await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(body, json!({"sent": true}));
        for (uri, payload, status) in [
            (
                "/api/send?name=/calibration&type=frc.vision.calibration.Command",
                "nope",
                StatusCode::BAD_REQUEST,
            ),
            (
                "/api/send?name=/constants&type=frc.vision.CameraConstants",
                "{}",
                StatusCode::FORBIDDEN,
            ),
            ("/api/send?name=/nope&type=x", "{}", StatusCode::NOT_FOUND),
            ("/api/send?name=/calibration", "{}", StatusCode::BAD_REQUEST),
        ] {
            let (got, body) = post_json(uri, payload).await;
            assert_eq!(got, status, "{uri}: {body}");
            assert!(body["error"].is_string(), "{uri}: {body}");
        }
    }

    #[tokio::test]
    async fn says_when_the_event_loop_is_gone() {
        let (requests, incoming) = mpsc::unbounded_channel();
        drop(incoming);
        let response = router(requests)
            .oneshot(Request::get("/api/channels").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    }
}
