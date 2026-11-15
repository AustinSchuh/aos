//! Streams the robot's cameras to the browser: each camera encoded once with H.264 and sent to
//! every viewer over WebRTC.  See the "Video" section of
//! the vision UI design.

pub use camera_streamer_core::{frame, signalling};

pub mod pipeline;
pub mod streamer;
