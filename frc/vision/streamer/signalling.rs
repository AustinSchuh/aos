//! The JSON messages that set up a viewer's WebRTC session, over the streamer's websocket.
//!
//! A viewer gets one peer connection carrying every camera it watches, one video track each,
//! all bundled onto one transport.  The browser says which cameras it wants, and the streamer
//! offers them (the robot sends; the browser only receives):
//!
//! ```text
//! streamer -> browser  {"type": "cameras", "cameras": [0, 1, 2, 3]}        on connecting
//! browser -> streamer  {"type": "watch", "cameras": [0, 2]}
//! streamer -> browser  {"type": "offer", "sdp": "v=0...", "cameras": [0, 2]}
//! browser -> streamer  {"type": "answer", "sdp": "v=0..."}
//! both ways            {"type": "ice", "candidate": "candidate:...", "sdpMLineIndex": 0}
//! streamer -> browser  {"type": "error", "message": "..."}
//! ```
//!
//! An offer's `cameras` names the camera each of its video sections (m-lines) carries, in order,
//! so the browser can tell its tracks apart.  A new `watch`, or a camera changing size, brings a
//! new offer for a new session, which replaces the old one.  Closing the websocket ends it.

use serde_json::{json, Value};

/// A message from the browser.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FromBrowser {
    Watch { cameras: Vec<u32> },
    Answer { sdp: String },
    Ice { candidate: String, mline_index: u32 },
}

/// A message to the browser.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ToBrowser {
    Cameras { cameras: Vec<u32> },
    Offer { sdp: String, cameras: Vec<u32> },
    Ice { candidate: String, mline_index: u32 },
    Error { message: String },
}

fn field<'a>(value: &'a Value, name: &str) -> Result<&'a Value, String> {
    value.get(name).ok_or_else(|| format!("no {name:?}"))
}

fn as_u32(value: &Value, name: &str) -> Result<u32, String> {
    value
        .as_u64()
        .and_then(|n| u32::try_from(n).ok())
        .ok_or_else(|| format!("{name:?} isn't a small non-negative integer"))
}

fn u32_field(value: &Value, name: &str) -> Result<u32, String> {
    as_u32(field(value, name)?, name)
}

fn string_field(value: &Value, name: &str) -> Result<String, String> {
    field(value, name)?
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{name:?} isn't a string"))
}

fn cameras_field(value: &Value) -> Result<Vec<u32>, String> {
    let list = field(value, "cameras")?
        .as_array()
        .ok_or_else(|| "\"cameras\" isn't a list".to_string())?;
    let mut cameras = Vec::with_capacity(list.len());
    for camera in list {
        let camera = as_u32(camera, "cameras")?;
        if !cameras.contains(&camera) {
            cameras.push(camera);
        }
    }
    Ok(cameras)
}

impl FromBrowser {
    /// Parses one websocket text message, or says what's wrong with it.
    pub fn parse(text: &str) -> Result<Self, String> {
        let value: Value = serde_json::from_str(text).map_err(|error| error.to_string())?;
        match field(&value, "type")?.as_str() {
            Some("watch") => Ok(Self::Watch {
                cameras: cameras_field(&value)?,
            }),
            Some("answer") => Ok(Self::Answer {
                sdp: string_field(&value, "sdp")?,
            }),
            Some("ice") => Ok(Self::Ice {
                candidate: string_field(&value, "candidate")?,
                mline_index: u32_field(&value, "sdpMLineIndex")?,
            }),
            _ => Err(format!("unknown \"type\" in {text}")),
        }
    }
}

impl ToBrowser {
    /// This message as websocket text.
    pub fn to_text(&self) -> String {
        match self {
            Self::Cameras { cameras } => json!({"type": "cameras", "cameras": cameras}),
            Self::Offer { sdp, cameras } => {
                json!({"type": "offer", "sdp": sdp, "cameras": cameras})
            }
            Self::Ice {
                candidate,
                mline_index,
            } => json!({
                "type": "ice",
                "candidate": candidate,
                "sdpMLineIndex": mline_index,
            }),
            Self::Error { message } => json!({"type": "error", "message": message}),
        }
        .to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_each_browser_message() {
        assert_eq!(
            FromBrowser::parse(r#"{"type": "watch", "cameras": [0, 2]}"#),
            Ok(FromBrowser::Watch {
                cameras: vec![0, 2]
            })
        );
        assert_eq!(
            FromBrowser::parse(r#"{"type": "answer", "sdp": "v=0\r\n"}"#),
            Ok(FromBrowser::Answer {
                sdp: "v=0\r\n".into()
            })
        );
        assert_eq!(
            FromBrowser::parse(
                r#"{"type": "ice", "candidate": "candidate:1 1 UDP 1 10.0.0.1 5000 typ host", "sdpMLineIndex": 1}"#
            ),
            Ok(FromBrowser::Ice {
                candidate: "candidate:1 1 UDP 1 10.0.0.1 5000 typ host".into(),
                mline_index: 1
            })
        );
    }

    #[test]
    fn watching_a_camera_twice_watches_it_once() {
        assert_eq!(
            FromBrowser::parse(r#"{"type": "watch", "cameras": [3, 1, 3]}"#),
            Ok(FromBrowser::Watch {
                cameras: vec![3, 1]
            })
        );
        assert_eq!(
            FromBrowser::parse(r#"{"type": "watch", "cameras": []}"#),
            Ok(FromBrowser::Watch { cameras: vec![] })
        );
    }

    #[test]
    fn says_what_is_wrong() {
        assert!(FromBrowser::parse("not json").is_err());
        assert_eq!(
            FromBrowser::parse(r#"{"cameras": [1]}"#),
            Err("no \"type\"".into())
        );
        assert!(FromBrowser::parse(r#"{"type": "dance"}"#)
            .unwrap_err()
            .contains("unknown"));
        assert_eq!(
            FromBrowser::parse(r#"{"type": "watch", "cameras": [-1]}"#),
            Err("\"cameras\" isn't a small non-negative integer".into())
        );
        assert_eq!(
            FromBrowser::parse(r#"{"type": "watch", "cameras": 1}"#),
            Err("\"cameras\" isn't a list".into())
        );
        assert_eq!(
            FromBrowser::parse(r#"{"type": "answer"}"#),
            Err("no \"sdp\"".into())
        );
    }

    #[test]
    fn writes_each_streamer_message() {
        let round_trip =
            |message: ToBrowser| -> Value { serde_json::from_str(&message.to_text()).unwrap() };
        assert_eq!(
            round_trip(ToBrowser::Cameras {
                cameras: vec![0, 1, 3]
            }),
            json!({"type": "cameras", "cameras": [0, 1, 3]})
        );
        assert_eq!(
            round_trip(ToBrowser::Offer {
                sdp: "v=0".into(),
                cameras: vec![2, 0]
            }),
            json!({"type": "offer", "sdp": "v=0", "cameras": [2, 0]})
        );
        assert_eq!(
            round_trip(ToBrowser::Ice {
                candidate: "c".into(),
                mline_index: 1
            }),
            json!({"type": "ice", "candidate": "c", "sdpMLineIndex": 1})
        );
        assert_eq!(
            round_trip(ToBrowser::Error {
                message: "no".into()
            }),
            json!({"type": "error", "message": "no"})
        );
    }
}
