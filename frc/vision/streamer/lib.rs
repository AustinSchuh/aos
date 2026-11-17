//! Streams the robot's cameras to the browser: each camera encoded once with H.264 and sent to
//! every viewer over WebRTC, with a box drawn on each AprilTag.  See the "Video" section of
//! the vision UI design.

pub use camera_streamer_core::{annotations, draw, frame, join, overlay, signalling, tag_box};

pub mod pipeline;
pub mod streamer;
