//! The parts of camera_streamer that need no GStreamer, so they build and test anywhere: turning
//! camera images into encoder frames, the boxes on the AprilTags and drawing them, joining frames
//! with their boxes, drawing calibration_server's corners, and the signalling messages.

pub mod annotations;
pub mod draw;
pub mod frame;
pub mod join;
pub mod overlay;
pub mod signalling;
pub mod tag_box;
