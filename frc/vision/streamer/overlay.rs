//! What is drawn on a frame: colored line segments, in the camera image's pixels.

/// An RGBA color, each channel 0 to 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rgba {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

/// Line segments of one color and width.
#[derive(Clone, Debug, PartialEq)]
pub struct Lines {
    /// Each segment's two ends in turn, in the camera image's pixels, from the top-left corner
    /// of the top-left pixel.
    pub points: Vec<(f64, f64)>,
    pub color: Rgba,
    /// Width, in image pixels.
    pub thickness: f64,
}

/// What to draw on one frame.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Overlay {
    /// The frame's capture time, in nanoseconds on the monotonic clock.
    pub capture_ns: i64,
    /// Drawn in order, so later lines cover earlier ones.
    pub lines: Vec<Lines>,
}
