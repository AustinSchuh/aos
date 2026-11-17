//! Apps that run only while a page uses them: the vision page's calibration, which needs the
//! cameras to itself.  When a command starting a session arrives, robot_server has starter stop
//! the apps in the way and start the session's app, and waits for the app to say it's ready
//! before passing the command on.  When a command ending it arrives, it passes that on, then
//! stops the app and starts the others again.  When robot_server starts, it does the same, so a
//! session it lost track of (it crashed, or was restarted) can't leave the robot without them.

/// A channel, by name and type.
pub type ChannelKey = (String, String);

/// An app the page starts and stops with commands on a channel.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SessionApp {
    /// Where the page's commands for it go, as JSON with an `"action"`.
    pub commands: ChannelKey,
    /// The app, as starter names it.
    pub app: String,
    /// The apps to stop while it runs.
    pub stops: Vec<String>,
    /// A channel the app sends on regularly: a message after it's started means it's ready.
    pub ready: ChannelKey,
    /// The actions that start a session, and those that end it.
    pub start_actions: Vec<String>,
    pub end_actions: Vec<String>,
}

/// What a command does to the session.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Effect {
    Starts,
    Ends,
    None,
}

impl SessionApp {
    /// What `json`, a command on `channel`, does to the session.
    pub fn effect(&self, channel: &ChannelKey, json: &serde_json::Value) -> Effect {
        if *channel != self.commands {
            return Effect::None;
        }
        let Some(action) = json.get("action").and_then(|action| action.as_str()) else {
            return Effect::None;
        };
        if self.start_actions.iter().any(|start| start == action) {
            Effect::Starts
        } else if self.end_actions.iter().any(|end| end == action) {
            Effect::Ends
        } else {
            Effect::None
        }
    }

    /// The starter commands that start a session: stop what's in the way, start the app.
    pub fn starting(&self) -> Vec<StarterCommand> {
        self.stops
            .iter()
            .map(|app| StarterCommand::stop(app))
            .chain([StarterCommand::start(&self.app)])
            .collect()
    }

    /// The starter commands that end a session: stop the app, start the others again.
    pub fn ending(&self) -> Vec<StarterCommand> {
        [StarterCommand::stop(&self.app)]
            .into_iter()
            .chain(self.stops.iter().map(|app| StarterCommand::start(app)))
            .collect()
    }
}

/// One `aos.starter.StarterRpc`.
#[derive(Clone, Debug, PartialEq)]
pub struct StarterCommand {
    pub command: &'static str,
    pub app: String,
}

impl StarterCommand {
    fn start(app: &str) -> Self {
        Self {
            command: "START",
            app: app.to_string(),
        }
    }

    fn stop(app: &str) -> Self {
        Self {
            command: "STOP",
            app: app.to_string(),
        }
    }

    /// As JSON for /aos aos.starter.StarterRpc.
    pub fn json(&self) -> String {
        serde_json::json!({"command": self.command, "name": self.app}).to_string()
    }
}

/// Parses "name:type", as the flags give channels.
pub fn parse_channel(text: &str) -> Option<ChannelKey> {
    let (name, type_) = text.rsplit_once(':')?;
    (!name.is_empty() && !type_.is_empty()).then(|| (name.to_string(), type_.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn calibration() -> SessionApp {
        SessionApp {
            commands: (
                "/calibration".into(),
                "frc.vision.calibration.Command".into(),
            ),
            app: "calibration_server".into(),
            stops: vec!["apriltag_detector0".into(), "apriltag_detector1".into()],
            ready: (
                "/calibration".into(),
                "frc.vision.calibration.Status".into(),
            ),
            start_actions: vec!["START".into()],
            end_actions: vec!["FINISH".into(), "CANCEL".into()],
        }
    }

    #[test]
    fn knows_what_a_command_does() {
        let app = calibration();
        let commands = app.commands.clone();
        assert_eq!(
            app.effect(&commands, &json!({"action": "START", "camera": 1})),
            Effect::Starts
        );
        assert_eq!(
            app.effect(&commands, &json!({"action": "CANCEL"})),
            Effect::Ends
        );
        assert_eq!(
            app.effect(&commands, &json!({"action": "UNDO"})),
            Effect::None
        );
        assert_eq!(app.effect(&commands, &json!({})), Effect::None);
        assert_eq!(
            app.effect(&("/other".into(), "x".into()), &json!({"action": "START"})),
            Effect::None
        );
    }

    #[test]
    fn stops_what_is_in_the_way() {
        let app = calibration();
        let commands = |list: Vec<StarterCommand>| {
            list.iter()
                .map(|c| format!("{} {}", c.command, c.app))
                .collect::<Vec<_>>()
        };
        assert_eq!(
            commands(app.starting()),
            [
                "STOP apriltag_detector0",
                "STOP apriltag_detector1",
                "START calibration_server"
            ]
        );
        assert_eq!(
            commands(app.ending()),
            [
                "STOP calibration_server",
                "START apriltag_detector0",
                "START apriltag_detector1"
            ]
        );
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&app.starting()[0].json()).unwrap(),
            json!({"command": "STOP", "name": "apriltag_detector0"})
        );
    }

    #[test]
    fn parses_channels() {
        assert_eq!(
            parse_channel("/calibration:frc.vision.calibration.Command"),
            Some((
                "/calibration".into(),
                "frc.vision.calibration.Command".into()
            ))
        );
        assert_eq!(parse_channel("/calibration"), None);
        assert_eq!(parse_channel(":x"), None);
    }
}
