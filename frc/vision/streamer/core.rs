//! The parts of camera_streamer that need no GStreamer, so they build and test anywhere: turning
//! camera images into encoder frames, and the signalling messages.

pub mod frame;
pub mod signalling;
