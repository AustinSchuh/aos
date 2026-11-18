//! An MCP server at `/mcp`, so an AI assistant can work with the running robot: list its
//! channels and read their latest messages.  See ADR 0007.
//!
//! It speaks MCP's streamable HTTP transport, the parts a server with only tools needs: each
//! JSON-RPC request is a POST, answered with one JSON response; notifications and responses
//! from the client get 202 Accepted.  There are no sessions, and no stream from the server
//! (GET is 405), which the transport allows.
//!
//! Like `/api`'s, the handlers ask [`ApiRequest`]s of the event loop's thread, where the
//! fetchers are.

use axum::{
    http::{header, HeaderMap, StatusCode},
    response::{IntoResponse, Response},
    routing::post,
    Router,
};
use serde_json::{json, Value};
use tokio::sync::{mpsc, oneshot};

use crate::api::{self, ApiRequest, ChannelInfo, LatestError};

/// The protocol versions this serves, newest first.
const PROTOCOL_VERSIONS: [&str; 3] = ["2025-06-18", "2025-03-26", "2024-11-05"];

/// What the assistant is told about the robot when it connects.
const INSTRUCTIONS: &str = "\
This is an AOS robot.  Its processes talk over channels: each has a name (like /drivetrain) \
and a flatbuffer type (like frc.control_loops.drivetrain.Status), and a name can have several \
types.  Messages carry two send times: monotonic (the robot's clock since boot, for timing and \
comparing messages) and realtime (wall time, for lining up with logs).  On a robot with several \
nodes (computers), each node has its own channels and clocks; this server is on one of them.

Start with list_channels, filtered by a word from what you're looking for, rather than guessing \
names.  get_latest reads a channel's newest message as JSON, with how old it is; an old message \
means whatever sends it may have stopped.  Large channels (images) and channels read by pinning \
can't be fetched.";

/// A JSON-RPC error's code, for each kind of mistake.
const INVALID_REQUEST: i64 = -32600;
const METHOD_NOT_FOUND: i64 = -32601;
const INVALID_PARAMS: i64 = -32602;
const PARSE_ERROR: i64 = -32700;

fn json_response(status: StatusCode, body: Value) -> Response {
    (
        status,
        [(header::CONTENT_TYPE, "application/json")],
        body.to_string(),
    )
        .into_response()
}

fn rpc_error(id: Value, code: i64, message: &str) -> Response {
    json_response(
        StatusCode::OK,
        json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}}),
    )
}

/// A tool's answer: text for the assistant, and whether it's an error.
struct ToolResult {
    text: String,
    is_error: bool,
}

impl ToolResult {
    fn ok(text: impl Into<String>) -> Self {
        Self {
            text: text.into(),
            is_error: false,
        }
    }

    fn error(text: impl Into<String>) -> Self {
        Self {
            text: text.into(),
            is_error: true,
        }
    }
}

fn tools() -> Value {
    json!([
        {
            "name": "list_channels",
            "title": "List channels",
            "description": "Lists the robot's channels: name, type, the most messages a second \
                it may send (frequency), the largest message (max_size, bytes), and whether \
                get_latest can read it.  Filter by a word in the name or type, such as \
                \"drivetrain\" or \"camera\"; matching is case-insensitive.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "filter": {
                        "type": "string",
                        "description": "Only channels whose name or type contains this."
                    }
                }
            },
            "annotations": {"readOnlyHint": true}
        },
        {
            "name": "get_latest",
            "title": "Read a channel's latest message",
            "description": "Reads a channel's newest message as JSON, with when it was sent \
                (monotonic and realtime, in ns) and its age in seconds.  The type can be left \
                out when only one type has that name.  Vectors longer than max_vector_size \
                are left out, so big arrays don't flood the answer.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "channel": {
                        "type": "string",
                        "description": "The channel's name, like /drivetrain."
                    },
                    "type": {
                        "type": "string",
                        "description": "The channel's type, if its name has several."
                    },
                    "max_vector_size": {
                        "type": "integer",
                        "minimum": 0,
                        "description": "Leave out vectors longer than this.  Default 100."
                    }
                },
                "required": ["channel"]
            },
            "annotations": {"readOnlyHint": true}
        }
    ])
}

/// Asks the event loop's thread, or None if it isn't answering.
async fn ask<T>(
    requests: &mpsc::UnboundedSender<ApiRequest>,
    request: impl FnOnce(oneshot::Sender<T>) -> ApiRequest,
) -> Option<T> {
    let (reply, answer) = oneshot::channel();
    requests.send(request(reply)).ok()?;
    answer.await.ok()
}

const GONE: &str = "The robot's event loop isn't answering";

async fn list_channels(
    requests: &mpsc::UnboundedSender<ApiRequest>,
    arguments: &Value,
) -> ToolResult {
    let filter = arguments["filter"].as_str().unwrap_or("").to_lowercase();
    let Some(channels) = ask(requests, |reply| ApiRequest::Channels { reply }).await else {
        return ToolResult::error(GONE);
    };
    let matching: Vec<Value> = channels
        .iter()
        .filter(|channel| {
            channel.name.to_lowercase().contains(&filter)
                || channel.type_.to_lowercase().contains(&filter)
        })
        .map(|channel| {
            json!({
                "name": channel.name,
                "type": channel.type_,
                "frequency": channel.frequency,
                "max_size": channel.max_size,
                "fetchable": channel.fetchable,
            })
        })
        .collect();
    if matching.is_empty() {
        return ToolResult::ok(format!(
            "No channel's name or type contains {filter:?}.  There are {} channels; try a \
             shorter word.",
            channels.len()
        ));
    }
    ToolResult::ok(Value::from(matching).to_string())
}

/// The type of the channel named `name`, if exactly one has that name.
fn only_type(channels: &[ChannelInfo], name: &str) -> Result<String, String> {
    let types: Vec<&str> = channels
        .iter()
        .filter(|channel| channel.name == name)
        .map(|channel| channel.type_.as_str())
        .collect();
    match types.as_slice() {
        [type_] => Ok(type_.to_string()),
        [] => Err(format!(
            "There's no channel named {name}.  list_channels finds them."
        )),
        _ => Err(format!(
            "{name} has several types; say which: {}",
            types.join(", ")
        )),
    }
}

/// The channel and type `arguments` name, filling the type in when it's left out.
async fn channel_of(
    requests: &mpsc::UnboundedSender<ApiRequest>,
    arguments: &Value,
) -> Result<(String, String), ToolResult> {
    let Some(name) = arguments["channel"].as_str() else {
        return Err(ToolResult::error(
            "Say which channel, like {\"channel\": \"/drivetrain\"}",
        ));
    };
    if let Some(type_) = arguments["type"].as_str() {
        return Ok((name.to_string(), type_.to_string()));
    }
    let Some(channels) = ask(requests, |reply| ApiRequest::Channels { reply }).await else {
        return Err(ToolResult::error(GONE));
    };
    only_type(&channels, name)
        .map(|type_| (name.to_string(), type_))
        .map_err(ToolResult::error)
}

async fn get_latest(requests: &mpsc::UnboundedSender<ApiRequest>, arguments: &Value) -> ToolResult {
    let (name, type_) = match channel_of(requests, arguments).await {
        Ok(channel) => channel,
        Err(error) => return error,
    };
    let max_vector_size = match &arguments["max_vector_size"] {
        Value::Null => api::DEFAULT_MAX_VECTOR_SIZE,
        value => match value.as_u64() {
            Some(size) => size as usize,
            None => return ToolResult::error("max_vector_size is a count, like 10"),
        },
    };
    let latest = ask(requests, |reply| ApiRequest::Latest {
        name: name.clone(),
        type_: type_.clone(),
        max_vector_size,
        reply,
    })
    .await;
    match latest {
        None => ToolResult::error(GONE),
        // The message is JSON already: put it in as it is, rather than parse it to write it
        // again.
        Some(Ok(latest)) => ToolResult::ok(format!(
            "{{\"channel\":{},\"type\":{},\"monotonic_event_time_ns\":{},\
             \"realtime_event_time_ns\":{},\"age_s\":{:.3},\"message\":{}}}",
            json!(name),
            json!(type_),
            latest.monotonic_event_time_ns,
            latest.realtime_event_time_ns,
            latest.age_ns as f64 / 1e9,
            latest.json
        )),
        Some(Err(LatestError::NoChannel)) => ToolResult::error(format!(
            "There's no channel {name} {type_}.  list_channels finds them."
        )),
        Some(Err(LatestError::NotFetchable(why))) => {
            ToolResult::error(format!("{name} {type_} can't be read here: {why}"))
        }
        Some(Err(LatestError::NoMessage)) => ToolResult::error(format!(
            "Nothing has been sent on {name} {type_} since this node started: whatever sends \
             it may not be running."
        )),
        Some(Err(LatestError::NotJson)) => ToolResult::error(format!(
            "{name} {type_}'s latest message doesn't match its schema"
        )),
    }
}

/// Answers one JSON-RPC request, by its method.
async fn answer(
    requests: &mpsc::UnboundedSender<ApiRequest>,
    id: Value,
    method: &str,
    params: &Value,
) -> Response {
    let result = match method {
        "initialize" => {
            let asked = params["protocolVersion"].as_str().unwrap_or("");
            let version = PROTOCOL_VERSIONS
                .iter()
                .find(|version| **version == asked)
                .unwrap_or(&PROTOCOL_VERSIONS[0]);
            json!({
                "protocolVersion": version,
                "capabilities": {"tools": {}},
                "serverInfo": {
                    "name": "robot_server",
                    "title": "AOS robot",
                    "version": env!("CARGO_PKG_VERSION"),
                },
                "instructions": INSTRUCTIONS,
            })
        }
        "ping" => json!({}),
        "tools/list" => json!({"tools": tools()}),
        "tools/call" => {
            let arguments = &params["arguments"];
            let result = match params["name"].as_str() {
                Some("list_channels") => list_channels(requests, arguments).await,
                Some("get_latest") => get_latest(requests, arguments).await,
                Some(other) => {
                    return rpc_error(id, INVALID_PARAMS, &format!("There's no tool {other}"))
                }
                None => return rpc_error(id, INVALID_PARAMS, "Say which tool"),
            };
            json!({
                "content": [{"type": "text", "text": result.text}],
                "isError": result.is_error,
            })
        }
        _ => {
            return rpc_error(
                id,
                METHOD_NOT_FOUND,
                &format!("This server doesn't do {method}"),
            )
        }
    };
    json_response(
        StatusCode::OK,
        json!({"jsonrpc": "2.0", "id": id, "result": result}),
    )
}

/// Whether a request from a browser page comes from one of the robot's own: anything else is
/// another site's page, which mustn't drive the robot's tools (DNS rebinding, or a form post).
/// Requests without an Origin aren't from a page.
fn same_origin(headers: &HeaderMap) -> bool {
    let Some(origin) = headers.get(header::ORIGIN) else {
        return true;
    };
    let host = headers
        .get(header::HOST)
        .and_then(|host| host.to_str().ok())
        .unwrap_or("");
    let origin = origin.to_str().unwrap_or("");
    let origin_host = origin
        .strip_prefix("https://")
        .or_else(|| origin.strip_prefix("http://"))
        .unwrap_or("");
    !host.is_empty() && origin_host == host
}

async fn post_mcp(
    requests: mpsc::UnboundedSender<ApiRequest>,
    headers: HeaderMap,
    body: String,
) -> Response {
    if !same_origin(&headers) {
        return (
            StatusCode::FORBIDDEN,
            "MCP only takes requests from this robot's pages",
        )
            .into_response();
    }
    let message: Value = match serde_json::from_str(&body) {
        Ok(message) => message,
        Err(error) => {
            return rpc_error(Value::Null, PARSE_ERROR, &format!("Not JSON: {error}"));
        }
    };
    if message.is_array() {
        return rpc_error(
            Value::Null,
            INVALID_REQUEST,
            "Send one message at a time, not a batch",
        );
    }
    let Some(method) = message["method"].as_str() else {
        // A response to a request of ours: we make none, so there's nothing to do.
        return StatusCode::ACCEPTED.into_response();
    };
    let id = message.get("id").cloned().unwrap_or(Value::Null);
    if id.is_null() {
        // A notification (initialized, cancelled): nothing to answer.
        return StatusCode::ACCEPTED.into_response();
    }
    let empty = Value::Object(Default::default());
    let params = message.get("params").unwrap_or(&empty);
    answer(&requests, id, method, params).await
}

/// The `/mcp` route, asking `requests` for what's on the channels.
pub fn router(requests: mpsc::UnboundedSender<ApiRequest>) -> Router {
    Router::new().route(
        "/mcp",
        post(move |headers: HeaderMap, body: String| post_mcp(requests.clone(), headers, body))
            .get(|| async { StatusCode::METHOD_NOT_ALLOWED })
            .delete(|| async { StatusCode::METHOD_NOT_ALLOWED }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::Latest;
    use axum::body::Body;
    use axum::http::Request;
    use std::collections::HashMap;
    use tower::ServiceExt;

    fn headers(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs
            .iter()
            .map(|(name, value)| (name.to_string(), value.to_string()))
            .collect()
    }

    /// Answers requests as a robot with /constants (one type, latest {"a": 1}), /status (two
    /// types) and an image channel.
    fn fake_robot() -> mpsc::UnboundedSender<ApiRequest> {
        let (requests, mut incoming) = mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Some(request) = incoming.recv().await {
                match request {
                    ApiRequest::Channels { reply } => {
                        let channel = |name: &str, type_: &str, fetchable| ChannelInfo {
                            name: name.into(),
                            type_: type_.into(),
                            max_size: 4096,
                            frequency: 10,
                            fetchable,
                        };
                        let _ = reply.send(vec![
                            channel("/constants", "frc.Constants", true),
                            channel("/status", "frc.StatusA", true),
                            channel("/status", "frc.StatusB", true),
                            channel("/camera0", "frc.vision.CameraImage", false),
                        ]);
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
                                age_ns: 1_500_000_000,
                                json: format!("{{\"a\":1,\"max_vector_size\":{max_vector_size}}}"),
                            }),
                            ("/camera0", _) => Err(LatestError::NotFetchable("Too big".into())),
                            _ => Err(LatestError::NoChannel),
                        });
                    }
                    _ => {}
                }
            }
        });
        requests
    }

    async fn post_to(
        requests: mpsc::UnboundedSender<ApiRequest>,
        body: Value,
        extra: HashMap<String, String>,
    ) -> (StatusCode, String) {
        let mut request = Request::post("/mcp").header("content-type", "application/json");
        for (name, value) in extra {
            request = request.header(name, value);
        }
        let response = router(requests)
            .oneshot(request.body(Body::from(body.to_string())).unwrap())
            .await
            .unwrap();
        let status = response.status();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        (status, String::from_utf8(body.to_vec()).unwrap())
    }

    async fn rpc(method: &str, params: Value) -> Value {
        let (status, body) = post_to(
            fake_robot(),
            json!({"jsonrpc": "2.0", "id": 7, "method": method, "params": params}),
            HashMap::new(),
        )
        .await;
        assert_eq!(status, StatusCode::OK, "{body}");
        let response: Value = serde_json::from_str(&body).unwrap();
        assert_eq!(response["jsonrpc"], "2.0");
        assert_eq!(response["id"], 7);
        response
    }

    /// Calls `tool`, returning its text and whether it's an error.
    async fn call(tool: &str, arguments: Value) -> (String, bool) {
        let response = rpc("tools/call", json!({"name": tool, "arguments": arguments})).await;
        let result = &response["result"];
        assert_eq!(result["content"][0]["type"], "text", "{response}");
        (
            result["content"][0]["text"].as_str().unwrap().to_string(),
            result["isError"].as_bool().unwrap(),
        )
    }

    #[tokio::test]
    async fn initializes() {
        let response = rpc(
            "initialize",
            json!({"protocolVersion": "2025-03-26", "capabilities": {},
                   "clientInfo": {"name": "test", "version": "1"}}),
        )
        .await;
        let result = &response["result"];
        assert_eq!(result["protocolVersion"], "2025-03-26");
        assert_eq!(result["capabilities"], json!({"tools": {}}));
        assert_eq!(result["serverInfo"]["name"], "robot_server");
        assert!(result["instructions"]
            .as_str()
            .unwrap()
            .contains("list_channels"));
        // A version this doesn't know gets the newest it does.
        let response = rpc("initialize", json!({"protocolVersion": "2099-01-01"})).await;
        assert_eq!(response["result"]["protocolVersion"], PROTOCOL_VERSIONS[0]);
    }

    #[tokio::test]
    async fn lists_its_tools() {
        let response = rpc("tools/list", json!({})).await;
        let names: Vec<&str> = response["result"]["tools"]
            .as_array()
            .unwrap()
            .iter()
            .map(|tool| tool["name"].as_str().unwrap())
            .collect();
        assert_eq!(names, ["list_channels", "get_latest"]);
        for tool in response["result"]["tools"].as_array().unwrap() {
            assert_eq!(tool["inputSchema"]["type"], "object", "{tool}");
            assert!(tool["description"].as_str().unwrap().len() > 20, "{tool}");
        }
    }

    #[tokio::test]
    async fn lists_channels() {
        let (text, is_error) = call("list_channels", json!({})).await;
        assert!(!is_error);
        let channels: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(channels.as_array().unwrap().len(), 4);
        assert_eq!(
            channels[0],
            json!({"name": "/constants", "type": "frc.Constants", "frequency": 10,
                   "max_size": 4096, "fetchable": true})
        );

        let (text, _) = call("list_channels", json!({"filter": "CAMERA"})).await;
        let channels: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(channels.as_array().unwrap().len(), 1);
        assert_eq!(channels[0]["fetchable"], false);

        let (text, is_error) = call("list_channels", json!({"filter": "nope"})).await;
        assert!(!is_error);
        assert!(text.contains("There are 4 channels"), "{text}");
    }

    #[tokio::test]
    async fn gets_the_latest() {
        let (text, is_error) = call("get_latest", json!({"channel": "/constants"})).await;
        assert!(!is_error, "{text}");
        assert_eq!(
            serde_json::from_str::<Value>(&text).unwrap(),
            json!({"channel": "/constants", "type": "frc.Constants",
                   "monotonic_event_time_ns": 12, "realtime_event_time_ns": 34, "age_s": 1.5,
                   "message": {"a": 1, "max_vector_size": api::DEFAULT_MAX_VECTOR_SIZE}})
        );
        let (text, _) = call(
            "get_latest",
            json!({"channel": "/constants", "type": "frc.Constants", "max_vector_size": 3}),
        )
        .await;
        assert_eq!(
            serde_json::from_str::<Value>(&text).unwrap()["message"]["max_vector_size"],
            3
        );
    }

    #[tokio::test]
    async fn says_why_it_cant_get_the_latest() {
        for (arguments, wanted) in [
            (json!({}), "Say which channel"),
            (
                json!({"channel": "/nope"}),
                "There's no channel named /nope",
            ),
            (
                json!({"channel": "/status"}),
                "/status has several types; say which: frc.StatusA, frc.StatusB",
            ),
            (
                json!({"channel": "/camera0"}),
                "can't be read here: Too big",
            ),
            (
                json!({"channel": "/constants", "max_vector_size": -1}),
                "max_vector_size is a count",
            ),
        ] {
            let (text, is_error) = call("get_latest", arguments.clone()).await;
            assert!(is_error, "{arguments}: {text}");
            assert!(text.contains(wanted), "{arguments}: {text}");
        }
    }

    #[tokio::test]
    async fn rejects_what_it_doesnt_do() {
        let response = rpc("resources/list", json!({})).await;
        assert_eq!(response["error"]["code"], METHOD_NOT_FOUND);
        let response = rpc("tools/call", json!({"name": "nope"})).await;
        assert_eq!(response["error"]["code"], INVALID_PARAMS);

        let (status, body) =
            post_to(fake_robot(), json!([{"jsonrpc": "2.0"}]), HashMap::new()).await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(
            serde_json::from_str::<Value>(&body).unwrap()["error"]["code"],
            INVALID_REQUEST
        );

        let response = router(fake_robot())
            .oneshot(Request::post("/mcp").body(Body::from("{")).unwrap())
            .await
            .unwrap();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        assert_eq!(
            serde_json::from_slice::<Value>(&body).unwrap()["error"]["code"],
            PARSE_ERROR
        );

        let response = router(fake_robot())
            .oneshot(Request::get("/mcp").body(Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::METHOD_NOT_ALLOWED);
    }

    #[tokio::test]
    async fn accepts_notifications_without_answering() {
        let (status, body) = post_to(
            fake_robot(),
            json!({"jsonrpc": "2.0", "method": "notifications/initialized"}),
            HashMap::new(),
        )
        .await;
        assert_eq!(status, StatusCode::ACCEPTED);
        assert_eq!(body, "");
    }

    #[tokio::test]
    async fn only_takes_pages_from_the_robot() {
        let ping = json!({"jsonrpc": "2.0", "id": 1, "method": "ping"});
        let (status, _) = post_to(
            fake_robot(),
            ping.clone(),
            headers(&[
                ("host", "orin-0-1.local"),
                ("origin", "https://evil.example"),
            ]),
        )
        .await;
        assert_eq!(status, StatusCode::FORBIDDEN);
        let (status, _) = post_to(
            fake_robot(),
            ping,
            headers(&[
                ("host", "orin-0-1.local"),
                ("origin", "https://orin-0-1.local"),
            ]),
        )
        .await;
        assert_eq!(status, StatusCode::OK);
    }

    #[tokio::test]
    async fn says_when_the_event_loop_is_gone() {
        let (requests, incoming) = mpsc::unbounded_channel();
        drop(incoming);
        let (status, body) = post_to(
            requests,
            json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                   "params": {"name": "list_channels", "arguments": {}}}),
            HashMap::new(),
        )
        .await;
        assert_eq!(status, StatusCode::OK);
        let response: Value = serde_json::from_str(&body).unwrap();
        assert_eq!(response["result"]["isError"], true);
        assert_eq!(response["result"]["content"][0]["text"], GONE);
    }
}
