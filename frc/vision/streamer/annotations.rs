//! `foxglove.ImageAnnotations` as an overlay: calibration_server's detected board corners on
//! /cameraN/calibration, which camera_streamer draws into the video.
//!
//! Points are drawn as small crosses, line strips, loops and lists as their segments.  The
//! overlay's capture time is the first annotation's timestamp, which calibration_server sets to
//! its image's capture time.

use foxglove_image_annotations_rust_fbs::foxglove::{
    root_as_image_annotations, Color, PointsAnnotationType,
};

use crate::overlay::{Lines, Overlay, Rgba};

/// How far each arm of a point's cross reaches, in image pixels per unit of its thickness.
const CROSS_ARM_PER_THICKNESS: f64 = 1.5;

fn rgba(color: Option<Color>) -> Rgba {
    match color {
        Some(color) => Rgba {
            r: color.r(),
            g: color.g(),
            b: color.b(),
            a: color.a(),
        },
        None => Rgba {
            r: 0.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        },
    }
}

/// The overlay for `data`, an ImageAnnotations; `None` if it isn't one, or has nothing to draw.
pub fn annotations_overlay(data: &[u8]) -> Option<Overlay> {
    let annotations = root_as_image_annotations(data).ok()?;
    let mut overlay = Overlay::default();
    let mut capture_ns = None;
    for annotation in annotations.points()?.iter() {
        if capture_ns.is_none() {
            if let Some(time) = annotation.timestamp() {
                capture_ns = Some(time.sec() as i64 * 1_000_000_000 + time.nsec() as i64);
            }
        }
        let points: Vec<(f64, f64)> = annotation
            .points()
            .map(|points| points.iter().map(|point| (point.x(), point.y())).collect())
            .unwrap_or_default();
        if points.is_empty() {
            continue;
        }
        let thickness = annotation.thickness().max(1.0);
        let kind = annotation.type_();
        let mut segments = Vec::new();
        if kind == PointsAnnotationType::LINE_LIST {
            for pair in points.chunks_exact(2) {
                segments.extend([pair[0], pair[1]]);
            }
        } else if kind == PointsAnnotationType::LINE_STRIP
            || kind == PointsAnnotationType::LINE_LOOP
        {
            for pair in points.windows(2) {
                segments.extend([pair[0], pair[1]]);
            }
            if kind == PointsAnnotationType::LINE_LOOP && points.len() > 2 {
                segments.extend([points[points.len() - 1], points[0]]);
            }
        } else {
            let arm = thickness * CROSS_ARM_PER_THICKNESS;
            for (x, y) in points {
                segments.extend([(x - arm, y), (x + arm, y), (x, y - arm), (x, y + arm)]);
            }
        }
        overlay.lines.push(Lines {
            points: segments,
            color: rgba(annotation.outline_color()),
            thickness: (thickness / 3.0).max(1.0),
        });
    }
    if overlay.lines.is_empty() {
        return None;
    }
    overlay.capture_ns = capture_ns.unwrap_or(0);
    Some(overlay)
}

#[cfg(test)]
mod tests {
    use super::*;
    use foxglove_image_annotations_rust_fbs::foxglove::{
        ColorT, ImageAnnotationsT, Point2T, PointsAnnotationT, TimeT,
    };

    fn annotations(points: Vec<PointsAnnotationT>) -> Vec<u8> {
        let mut fbb = flatbuffers::FlatBufferBuilder::new();
        let mut annotations = ImageAnnotationsT::default();
        annotations.points = Some(points);
        let annotations = annotations.pack(&mut fbb);
        fbb.finish_minimal(annotations);
        fbb.finished_data().to_vec()
    }

    fn points(
        kind: PointsAnnotationType,
        list: &[(f64, f64)],
        thickness: f64,
    ) -> PointsAnnotationT {
        let mut annotation = PointsAnnotationT::default();
        annotation.type_ = kind;
        annotation.points = Some(
            list.iter()
                .map(|&(x, y)| {
                    let mut point = Point2T::default();
                    point.x = x;
                    point.y = y;
                    point
                })
                .collect(),
        );
        annotation.thickness = thickness;
        annotation
    }

    #[test]
    fn points_become_crosses() {
        let mut annotation = points(
            PointsAnnotationType::POINTS,
            &[(10.0, 20.0), (30.0, 40.0)],
            6.0,
        );
        let mut time = TimeT::default();
        time.sec = 2;
        time.nsec = 5;
        annotation.timestamp = Some(time);
        let mut color = ColorT::default();
        color.r = 1.0;
        color.g = 0.8;
        color.b = 0.0;
        color.a = 1.0;
        annotation.outline_color = Some(Box::new(color));
        let overlay = annotations_overlay(&annotations(vec![annotation])).unwrap();
        assert_eq!(overlay.capture_ns, 2_000_000_005);
        assert_eq!(overlay.lines.len(), 1);
        let lines = &overlay.lines[0];
        // Two segments a point, each two ends.
        assert_eq!(lines.points.len(), 8);
        assert_eq!(lines.points[0], (1.0, 20.0));
        assert_eq!(lines.points[1], (19.0, 20.0));
        assert_eq!(
            lines.color,
            Rgba {
                r: 1.0,
                g: 0.8,
                b: 0.0,
                a: 1.0
            }
        );
    }

    #[test]
    fn loops_close() {
        let data = annotations(vec![points(
            PointsAnnotationType::LINE_LOOP,
            &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            1.0,
        )]);
        let overlay = annotations_overlay(&data).unwrap();
        assert_eq!(overlay.lines[0].points.len(), 6);
        assert_eq!(overlay.lines[0].points[5], (0.0, 0.0));
    }

    #[test]
    fn nothing_to_draw() {
        assert_eq!(annotations_overlay(&annotations(vec![])), None);
        assert_eq!(
            annotations_overlay(&annotations(vec![PointsAnnotationT::default()])),
            None
        );
        assert_eq!(annotations_overlay(&[1, 2, 3]), None);
    }
}
