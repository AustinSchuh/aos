//! Drawing overlays into an I420 frame before it is encoded, so every viewer sees them on exactly
//! the frame they belong to.

use crate::overlay::{Overlay, Rgba};

/// An I420 frame: the Y plane, then U and V at half size.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub i420: Vec<u8>,
    pub width: u32,
    pub height: u32,
}

/// `rgba` as limited-range BT.601 Y, U and V, which is what x264 assumes for these frames.
fn yuv(rgba: Rgba) -> (f64, f64, f64) {
    let (r, g, b) = (
        rgba.r.clamp(0.0, 1.0),
        rgba.g.clamp(0.0, 1.0),
        rgba.b.clamp(0.0, 1.0),
    );
    (
        16.0 + 65.481 * r + 128.553 * g + 24.966 * b,
        128.0 - 37.797 * r - 74.203 * g + 112.0 * b,
        128.0 + 112.0 * r - 93.786 * g - 18.214 * b,
    )
}

/// One plane of the frame, and how image coordinates map onto it.
struct Plane<'a> {
    pixels: &'a mut [u8],
    width: usize,
    height: usize,
    /// Plane pixels per camera-image pixel.
    scale: f64,
}

impl Plane<'_> {
    /// Blends `value` at `alpha` over every pixel within `radius` (plane pixels) of the segment
    /// from `a` to `b` (image pixels).
    fn segment(&mut self, a: (f64, f64), b: (f64, f64), radius: f64, value: f64, alpha: f64) {
        let (ax, ay) = (a.0 * self.scale, a.1 * self.scale);
        let (bx, by) = (b.0 * self.scale, b.1 * self.scale);
        let reach = radius + 1.0;
        let x0 = (ax.min(bx) - reach).floor().max(0.0) as usize;
        let y0 = (ay.min(by) - reach).floor().max(0.0) as usize;
        let x1 = ((ax.max(bx) + reach).ceil().max(0.0) as usize).min(self.width);
        let y1 = ((ay.max(by) + reach).ceil().max(0.0) as usize).min(self.height);
        let (dx, dy) = (bx - ax, by - ay);
        let length2 = dx * dx + dy * dy;
        for y in y0..y1 {
            let py = y as f64 + 0.5;
            for x in x0..x1 {
                let px = x as f64 + 0.5;
                // Distance from the pixel's centre to the segment.
                let t = if length2 > 0.0 {
                    (((px - ax) * dx + (py - ay) * dy) / length2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (cx, cy) = (ax + t * dx - px, ay + t * dy - py);
                // Antialiased over the last pixel.
                let coverage = (radius + 0.5 - (cx * cx + cy * cy).sqrt()).clamp(0.0, 1.0);
                // Also skips NaN, from a point that isn't a number.
                if !(coverage > 0.0) {
                    continue;
                }
                let pixel = &mut self.pixels[y * self.width + x];
                let a = alpha * coverage;
                *pixel = (*pixel as f64 * (1.0 - a) + value * a)
                    .round()
                    .clamp(0.0, 255.0) as u8;
            }
        }
    }
}

/// The three planes of `frame`, each with its scale from image coordinates.
fn planes(frame: &mut Frame, scale: f64) -> [Plane<'_>; 3] {
    let (w, h) = (frame.width as usize, frame.height as usize);
    let (cw, ch) = (w / 2, h / 2);
    let (y, chroma) = frame.i420.split_at_mut(w * h);
    let (u, v) = chroma.split_at_mut(cw * ch);
    [
        Plane {
            pixels: y,
            width: w,
            height: h,
            scale,
        },
        Plane {
            pixels: u,
            width: cw,
            height: ch,
            scale: scale / 2.0,
        },
        Plane {
            pixels: v,
            width: cw,
            height: ch,
            scale: scale / 2.0,
        },
    ]
}

/// The thinnest a line is drawn, as a radius in stream pixels: anything thinner is lost to the
/// encoder.
const MIN_LINE_RADIUS: f64 = 1.0;

/// Draws `overlay` into `frame`.  `scale` is the frame's size over the camera image's, which the
/// overlay's coordinates are in.
pub fn draw(frame: &mut Frame, overlay: &Overlay, scale: f64) {
    for lines in &overlay.lines {
        // A line's thickness is its width in image pixels.
        let radius = (lines.thickness.max(0.0) * scale / 2.0).max(MIN_LINE_RADIUS);
        let (y, u, v) = yuv(lines.color);
        for segment in lines.points.chunks_exact(2) {
            for (plane, value) in planes(frame, scale).iter_mut().zip([y, u, v]) {
                // The chroma planes are half size.
                let r = (radius * plane.scale / scale).max(0.5);
                plane.segment(segment[0], segment[1], r, value, lines.color.a);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::overlay::Lines;

    const GREEN: Rgba = Rgba {
        r: 0.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };

    fn gray_frame(width: u32, height: u32) -> Frame {
        let luma = (width * height) as usize;
        let mut i420 = vec![100u8; luma];
        i420.resize(luma + 2 * (luma / 4), 128);
        Frame {
            i420,
            width,
            height,
        }
    }

    fn y(frame: &Frame, x: usize, y: usize) -> u8 {
        frame.i420[y * frame.width as usize + x]
    }

    fn uv(frame: &Frame, x: usize, y: usize) -> (u8, u8) {
        let (w, h) = (frame.width as usize, frame.height as usize);
        let at = (y / 2) * (w / 2) + x / 2;
        (
            frame.i420[w * h + at],
            frame.i420[w * h + (w / 2) * (h / 2) + at],
        )
    }

    fn lines(points: Vec<(f64, f64)>, color: Rgba, thickness: f64) -> Overlay {
        Overlay {
            capture_ns: 0,
            lines: vec![Lines {
                points,
                color,
                thickness,
            }],
        }
    }

    #[test]
    fn draws_each_segment_in_its_color_and_leaves_the_rest() {
        let mut frame = gray_frame(64, 64);
        // The top and left edges of a square.
        let edges = vec![(10.0, 10.0), (50.0, 10.0), (10.0, 10.0), (10.0, 50.0)];
        draw(&mut frame, &lines(edges, GREEN, 4.0), 1.0);
        let (gy, gu, gv) = yuv(GREEN);
        assert_eq!(y(&frame, 30, 10), gy.round() as u8);
        assert_eq!(uv(&frame, 30, 10), (gu.round() as u8, gv.round() as u8));
        assert_eq!(y(&frame, 10, 30), gy.round() as u8);
        // Off the segments: untouched, including where a closing edge would be.
        assert_eq!(y(&frame, 30, 30), 100);
        assert_eq!(uv(&frame, 30, 30), (128, 128));
        assert_eq!(y(&frame, 50, 30), 100);
    }

    #[test]
    fn scales_image_coordinates_to_the_frame() {
        // A 1600-wide camera streamed at 800: image x = 600 is frame x = 300.
        let mut frame = gray_frame(400, 40);
        draw(
            &mut frame,
            &lines(vec![(600.0, 0.0), (600.0, 80.0)], GREEN, 4.0),
            0.5,
        );
        assert_ne!(y(&frame, 300, 20), 100);
        assert_eq!(y(&frame, 290, 20), 100);
        assert_eq!(y(&frame, 310, 20), 100);
    }

    #[test]
    fn thin_lines_stay_visible() {
        // 1 image pixel wide, streamed at half size: still MIN_LINE_RADIUS each side.
        let mut frame = gray_frame(100, 20);
        draw(
            &mut frame,
            &lines(vec![(0.0, 20.0), (200.0, 20.0)], GREEN, 1.0),
            0.5,
        );
        let (gy, _, _) = yuv(GREEN);
        assert_eq!(y(&frame, 50, 9), gy.round() as u8);
        assert_eq!(y(&frame, 50, 10), gy.round() as u8);
        assert_eq!(y(&frame, 50, 13), 100);
    }

    #[test]
    fn off_frame_and_degenerate_segments_are_safe() {
        let mut frame = gray_frame(16, 16);
        let wild = vec![
            (-1e9, -1e9),
            (1e9, 5.0),
            (f64::NAN, 3.0),
            (4.0, 4.0),
            (8.0, 8.0),
        ];
        draw(&mut frame, &lines(wild, GREEN, 2.0), 1.0);
        draw(&mut frame, &lines(vec![], GREEN, 2.0), 1.0);
        // A zero-length segment is a dot.
        draw(
            &mut frame,
            &lines(vec![(8.0, 8.0), (8.0, 8.0)], GREEN, 2.0),
            1.0,
        );
        assert_ne!(y(&frame, 8, 8), 100);
    }

    #[test]
    fn alpha_blends() {
        let mut frame = gray_frame(32, 32);
        let mut translucent = GREEN;
        translucent.a = 0.5;
        draw(
            &mut frame,
            &lines(vec![(4.0, 16.0), (28.0, 16.0)], translucent, 4.0),
            1.0,
        );
        let (gy, _, _) = yuv(GREEN);
        assert_eq!(y(&frame, 16, 16), ((100.0 + gy) / 2.0).round() as u8);
        assert_eq!(y(&frame, 16, 20), 100);
    }
}
