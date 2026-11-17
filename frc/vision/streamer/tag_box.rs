//! A box standing on each AprilTag the detector found, from its `TargetMap`: a picture of the
//! pose the localizer gets, which shows at a glance when it is wrong.
//!
//! Poses are in the undistorted camera's frame, as the detector estimates them, so drawing one on
//! the raw image means projecting it with the camera's intrinsics and then distorting it with the
//! calibration's model (OpenCV's, or Kannala-Brandt's), as frc/orin/distortion.h's ReDistort does.  The box's edges are split up so they
//! bend with the lens.

use camera_constants_rust_fbs::frc::vision as constants;
use target_map_rust_fbs::frc::vision as target_map;

use crate::overlay::{Lines, Overlay, Rgba};

/// A camera's pinhole intrinsics and distortion, from its calibration.
#[derive(Clone, Debug, PartialEq)]
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// OpenCV's k1, k2, p1, p2, k3, k4, k5, k6: the 5 or 8 parameter model, zeros for those
    /// missing; or with `kannala_brandt`, Kannala-Brandt's k1..k4 and zeros.
    pub dist: [f64; 8],
    pub kannala_brandt: bool,
}

impl Intrinsics {
    /// `camera` on `node`'s calibration, from a `CameraConstants` message.  `None` if it has none,
    /// or it isn't usable.
    pub fn find(constants_data: &[u8], node: &str, camera: u32) -> Option<Self> {
        let constants = constants::root_as_camera_constants(constants_data).ok()?;
        let calibration = constants.calibration()?.iter().find(|calibration| {
            calibration.node_name() == Some(node)
                && u32::try_from(calibration.camera_number()) == Ok(camera)
        })?;
        let k = calibration.intrinsics()?;
        if k.len() != 9 {
            return None;
        }
        let mut dist = [0.0; 8];
        if let Some(coefficients) = calibration.dist_coeffs() {
            for (d, c) in dist.iter_mut().zip(coefficients.iter()) {
                *d = c as f64;
            }
        }
        Some(Self {
            fx: k.get(0) as f64,
            cx: k.get(2) as f64,
            fy: k.get(4) as f64,
            cy: k.get(5) as f64,
            dist,
            kannala_brandt: calibration.distortion_model()
                == constants::calibration::DistortionModel::KANNALA_BRANDT,
        })
    }

    /// Where the point `p`, in the camera's frame, lands on the raw image; `None` if it's behind
    /// the camera.
    fn project(&self, p: [f64; 3]) -> Option<(f64, f64)> {
        if !(p[2] > 1e-3) {
            return None;
        }
        let (x, y) = (p[0] / p[2], p[1] / p[2]);
        if self.kannala_brandt {
            // The distorted angle from the axis, a polynomial in the angle.
            let [k1, k2, k3, k4, ..] = self.dist;
            let r = (x * x + y * y).sqrt();
            let scale = if r < 1e-12 {
                1.0
            } else {
                let theta = r.atan();
                let t2 = theta * theta;
                theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) / r
            };
            return Some((self.fx * x * scale + self.cx, self.fy * y * scale + self.cy));
        }
        let [k1, k2, p1, p2, k3, k4, k5, k6] = self.dist;
        let r2 = x * x + y * y;
        let radial =
            (1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1.0 + r2 * (k4 + r2 * (k5 + r2 * k6)));
        let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        Some((self.fx * xd + self.cx, self.fy * yd + self.cy))
    }
}

/// A tag's pose in the camera's frame.
#[derive(Clone, Copy, Debug)]
struct Pose {
    /// Row-major.
    rotation: [[f64; 3]; 3],
    translation: [f64; 3],
}

impl Pose {
    fn from_quaternion(w: f64, x: f64, y: f64, z: f64, translation: [f64; 3]) -> Option<Self> {
        let n = (w * w + x * x + y * y + z * z).sqrt();
        if !(n > 0.0) {
            return None;
        }
        let (w, x, y, z) = (w / n, x / n, y / n, z / n);
        Some(Self {
            rotation: [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - w * z),
                    2.0 * (x * z + w * y),
                ],
                [
                    2.0 * (x * y + w * z),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - w * x),
                ],
                [
                    2.0 * (x * z - w * y),
                    2.0 * (y * z + w * x),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            translation,
        })
    }

    fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let r = &self.rotation;
        let t = &self.translation;
        [
            r[0][0] * p[0] + r[0][1] * p[1] + r[0][2] * p[2] + t[0],
            r[1][0] * p[0] + r[1][1] * p[1] + r[1][2] * p[2] + t[1],
            r[2][0] * p[0] + r[2][1] * p[1] + r[2][2] * p[2] + t[2],
        ]
    }
}

/// The box's edges are colored by the tag axis they run along, as OpenCV's drawFrameAxes colors
/// axes: x red, y green, z blue.
const X_COLOR: Rgba = Rgba {
    r: 1.0,
    g: 0.0,
    b: 0.0,
    a: 1.0,
};
const Y_COLOR: Rgba = Rgba {
    r: 0.0,
    g: 1.0,
    b: 0.0,
    a: 1.0,
};
const Z_COLOR: Rgba = Rgba {
    r: 0.0,
    g: 0.0,
    b: 1.0,
    a: 1.0,
};
/// Line width, in image pixels.
const THICKNESS: f64 = 3.0;
/// Pieces each edge is split into.
const SUBDIVISIONS: usize = 4;

/// One edge of a box, as the segments it is split into, and how far its middle is from the
/// camera.
struct Edge {
    depth: f64,
    lines: Lines,
}

/// The twelve edges of a cube `tag_size` across standing on the tag at `pose`.  aprilrobotics'
/// tag frame has x right and y down across the tag's face and z into the tag, so the cube stands
/// out of it, towards the camera.  `None` if any of it is behind the camera.
fn tag_box(pose: &Pose, intrinsics: &Intrinsics, tag_size: f64) -> Option<Vec<Edge>> {
    let h = tag_size / 2.0;
    let mut edges = Vec::with_capacity(12);
    // Each edge, split into SUBDIVISIONS segments.
    let mut edge = |color: Rgba, a: [f64; 3], b: [f64; 3]| -> Option<()> {
        let at = |f: f64| {
            [
                a[0] + f * (b[0] - a[0]),
                a[1] + f * (b[1] - a[1]),
                a[2] + f * (b[2] - a[2]),
            ]
        };
        let mut points = Vec::with_capacity(2 * SUBDIVISIONS);
        let mut previous = intrinsics.project(pose.apply(a))?;
        for step in 1..=SUBDIVISIONS {
            let next = intrinsics.project(pose.apply(at(step as f64 / SUBDIVISIONS as f64)))?;
            points.extend([previous, next]);
            previous = next;
        }
        edges.push(Edge {
            depth: pose.apply(at(0.5))[2],
            lines: Lines {
                points,
                color,
                thickness: THICKNESS,
            },
        });
        Some(())
    };
    for z in [0.0, -tag_size] {
        for y in [h, -h] {
            edge(X_COLOR, [-h, y, z], [h, y, z])?;
        }
        for x in [-h, h] {
            edge(Y_COLOR, [x, h, z], [x, -h, z])?;
        }
    }
    for (x, y) in [(-h, h), (h, h), (h, -h), (-h, -h)] {
        edge(Z_COLOR, [x, y, 0.0], [x, y, -tag_size])?;
    }
    Some(edges)
}

/// What to draw for one `TargetMap` message: a box on each tag, if the camera's `intrinsics`
/// are known, stamped with the capture time of the image the tags were found in.  The edges of
/// every box are drawn farthest first, so nearer ones cover them.
pub fn target_map_overlay(
    data: &[u8],
    intrinsics: Option<&Intrinsics>,
    tag_size: f64,
) -> Result<Overlay, flatbuffers::InvalidFlatbuffer> {
    let map = target_map::root_as_target_map(data)?;
    let mut overlay = Overlay {
        capture_ns: map.monotonic_timestamp_ns(),
        lines: vec![],
    };
    let Some(intrinsics) = intrinsics else {
        return Ok(overlay);
    };
    let mut edges = Vec::new();
    for target in map.target_poses().iter().flatten() {
        let (Some(position), Some(orientation)) = (target.position(), target.orientation()) else {
            continue;
        };
        let Some(pose) = Pose::from_quaternion(
            orientation.w(),
            orientation.x(),
            orientation.y(),
            orientation.z(),
            [position.x(), position.y(), position.z()],
        ) else {
            continue;
        };
        if let Some(tag_edges) = tag_box(&pose, intrinsics, tag_size) {
            edges.extend(tag_edges);
        }
    }
    edges.sort_by(|a, b| b.depth.total_cmp(&a.depth));
    overlay.lines = edges.into_iter().map(|edge| edge.lines).collect();
    Ok(overlay)
}

#[cfg(test)]
mod tests {
    use super::*;
    use flatbuffers::FlatBufferBuilder;

    const CAMERA: Intrinsics = Intrinsics {
        fx: 1000.0,
        fy: 1000.0,
        cx: 800.0,
        cy: 650.0,
        dist: [0.0; 8],
        kannala_brandt: false,
    };

    /// A TargetMap of tags at `positions`, each with orientation `wxyz`.
    fn target_map(capture_ns: i64, tags: &[([f64; 3], [f64; 4])]) -> Vec<u8> {
        let mut fbb = FlatBufferBuilder::new();
        let poses: Vec<_> = tags
            .iter()
            .enumerate()
            .map(|(id, &([x, y, z], [qw, qx, qy, qz]))| {
                let position =
                    target_map::Position::create(&mut fbb, &target_map::PositionArgs { x, y, z });
                let orientation = target_map::Quaternion::create(
                    &mut fbb,
                    &target_map::QuaternionArgs {
                        w: qw,
                        x: qx,
                        y: qy,
                        z: qz,
                    },
                );
                target_map::TargetPoseFbs::create(
                    &mut fbb,
                    &target_map::TargetPoseFbsArgs {
                        id: id as u64,
                        position: Some(position),
                        orientation: Some(orientation),
                        ..Default::default()
                    },
                )
            })
            .collect();
        let poses = fbb.create_vector(&poses);
        let map = target_map::TargetMap::create(
            &mut fbb,
            &target_map::TargetMapArgs {
                target_poses: Some(poses),
                monotonic_timestamp_ns: capture_ns,
                ..Default::default()
            },
        );
        fbb.finish_minimal(map);
        fbb.finished_data().to_vec()
    }

    const FACING: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    fn close(a: (f64, f64), b: (f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6
    }

    /// Every edge of `color`, as one line list: those along one of the tag's axes, as OpenCV
    /// colors them.
    fn by_color(overlay: &Overlay, color: Rgba) -> Lines {
        let mut all = Lines {
            points: vec![],
            color,
            thickness: THICKNESS,
        };
        for points in overlay.lines.iter().filter(|p| p.color == color) {
            all.points.extend(&points.points);
        }
        all
    }

    /// Every point of a line list, without repeats.
    fn corners(points: &Lines) -> Vec<(f64, f64)> {
        let mut unique: Vec<(f64, f64)> = Vec::new();
        for &p in &points.points {
            if !unique.iter().any(|&q| close(p, q)) {
                unique.push(p);
            }
        }
        unique
    }

    fn has(points: &Lines, p: (f64, f64)) -> bool {
        points.points.iter().any(|&q| close(p, q))
    }

    #[test]
    fn a_box_stands_out_of_the_tag_towards_the_camera() {
        // 0.2 m across, 2 m straight ahead, squarely facing the camera.
        let data = target_map(123, &[([0.0, 0.0, 2.0], FACING)]);
        let overlay = target_map_overlay(&data, Some(&CAMERA), 0.2).unwrap();
        assert_eq!(overlay.capture_ns, 123);
        assert_eq!(overlay.lines.len(), 12);
        let x = &by_color(&overlay, X_COLOR);
        let y = &by_color(&overlay, Y_COLOR);
        let z = &by_color(&overlay, Z_COLOR);
        for axis in [x, y, z] {
            // 4 edges, each in SUBDIVISIONS segments.
            assert_eq!(axis.points.len(), 4 * SUBDIVISIONS * 2);
        }
        // The tag's corners are 0.1 m from its centre at 2 m: 50 pixels.  The top is 0.2 m
        // nearer, at 1.8 m, so bigger.
        let near = 100.0 / 1.8;
        // Along x: the tag's top and bottom edges, and the top face's.
        assert!(has(x, (750.0, 700.0)) && has(x, (850.0, 600.0)));
        assert!(has(x, (775.0, 700.0)), "split into quarters");
        assert!(has(x, (800.0 + near, 650.0 + near)));
        // Along z: from each of the tag's corners to the same corner of the top.
        assert!(has(z, (750.0, 700.0)) && has(z, (800.0 - near, 650.0 + near)));
        // A straight edge without distortion: its points all on one line, 5 of them.
        let bottom: Vec<_> = corners(x)
            .into_iter()
            .filter(|p| (p.1 - 700.0).abs() < 1e-6)
            .collect();
        assert_eq!(bottom.len(), SUBDIVISIONS + 1);
        // Down the tag's face is y: its left and right edges.
        assert!(y.points.iter().all(|p| (p.0 - 750.0).abs() < 1e-6
            || (p.0 - 850.0).abs() < 1e-6
            || (p.0 - (800.0 - near)).abs() < 1e-6
            || (p.0 - (800.0 + near)).abs() < 1e-6));
    }

    #[test]
    fn turning_the_tag_turns_the_box() {
        // Turned 90 degrees about the camera's axis: the tag's x is the camera's y.
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let data = target_map(0, &[([0.0, 0.0, 2.0], [half, 0.0, 0.0, half])]);
        let overlay = target_map_overlay(&data, Some(&CAMERA), 0.2).unwrap();
        // The tag's x is the camera's y: its x edges run down the image.
        let x = &by_color(&overlay, X_COLOR);
        assert!(x
            .points
            .chunks_exact(2)
            .all(|s| (s[0].0 - s[1].0).abs() < 1e-6));
        assert!(has(x, (750.0, 600.0)));
    }

    #[test]
    fn the_box_bends_with_the_lens() {
        // Off to the side, where barrel distortion pulls it in towards the centre.
        let mut barrel = CAMERA.clone();
        barrel.dist[0] = -0.3;
        let data = target_map(0, &[([1.0, 0.0, 2.0], FACING)]);
        let straight = target_map_overlay(&data, Some(&CAMERA), 0.2).unwrap();
        let bent = target_map_overlay(&data, Some(&barrel), 0.2).unwrap();
        let corner = |a: &Overlay| {
            by_color(a, Z_COLOR)
                .points
                .iter()
                .map(|p| p.0)
                .fold(f64::INFINITY, f64::min)
        };
        assert!(corner(&bent) < corner(&straight));
    }

    #[test]
    fn kannala_brandt_projects_by_the_angle() {
        // With no coefficients, the distorted radius is the angle: a point 45 degrees off the
        // axis lands at pi / 4 focal lengths, not 1.
        let mut lens = CAMERA.clone();
        lens.kannala_brandt = true;
        let (u, v) = lens.project([1.0, 0.0, 1.0]).unwrap();
        assert!((u - (800.0 + 1000.0 * std::f64::consts::FRAC_PI_4)).abs() < 1e-9);
        assert!((v - 650.0).abs() < 1e-9);
        // k1 > 0 pushes it out.
        lens.dist[0] = 0.1;
        assert!(lens.project([1.0, 0.0, 1.0]).unwrap().0 > u);
    }

    #[test]
    fn nearer_edges_are_drawn_over_farther_ones() {
        // Two tags, the second nearer, and turned so its own edges are at different depths.
        let tilt = (0.3f64 / 2.0).cos();
        let data = target_map(
            0,
            &[
                ([0.0, 0.0, 2.0], FACING),
                ([0.1, 0.0, 1.0], [tilt, (0.3f64 / 2.0).sin(), 0.0, 0.0]),
            ],
        );
        let overlay = target_map_overlay(&data, Some(&CAMERA), 0.2).unwrap();
        assert_eq!(overlay.lines.len(), 24);
        // Drawn in order, so farthest first.  An edge's depth is the size of its middle's
        // perspective: nearer edges look longer for the same 0.2 m.
        let depth = |points: &Lines| {
            let (a, b) = (points.points[0], points.points[points.points.len() - 1]);
            0.2 * CAMERA.fx / ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
        };
        let x_edges: Vec<f64> = overlay
            .lines
            .iter()
            .filter(|p| p.color == X_COLOR)
            .map(depth)
            .collect();
        assert!(
            x_edges.windows(2).all(|w| w[0] >= w[1] - 1e-9),
            "{x_edges:?}"
        );
        // The far tag's edges all come before the near one's.
        assert!(x_edges[..4].iter().all(|&d| d > 1.5));
    }

    #[test]
    fn nothing_drawn_without_intrinsics_or_in_front_of_the_lens() {
        let data = target_map(77, &[([0.0, 0.0, 2.0], FACING)]);
        let overlay = target_map_overlay(&data, None, 0.2).unwrap();
        // Still stamped, so its frame doesn't wait for it.
        assert_eq!(overlay.capture_ns, 77);
        assert!(overlay.lines.is_empty());
        // So close the box would poke through the lens.
        let data = target_map(0, &[([0.0, 0.0, 0.1], FACING)]);
        assert!(target_map_overlay(&data, Some(&CAMERA), 0.2)
            .unwrap()
            .lines
            .is_empty());
        assert!(target_map_overlay(&[], Some(&CAMERA), 0.2).is_err());
    }

    fn camera_constants(calibrations: &[(&str, i32, [f32; 9], &[f32])]) -> Vec<u8> {
        camera_constants_with_model(
            &calibrations
                .iter()
                .map(|&(node, camera, k, dist)| {
                    (
                        node,
                        camera,
                        k,
                        dist,
                        constants::calibration::DistortionModel::OPENCV,
                    )
                })
                .collect::<Vec<_>>(),
        )
    }

    fn camera_constants_with_model(
        calibrations: &[(
            &str,
            i32,
            [f32; 9],
            &[f32],
            constants::calibration::DistortionModel,
        )],
    ) -> Vec<u8> {
        use constants::calibration as cal;
        let mut fbb = FlatBufferBuilder::new();
        let calibrations: Vec<_> = calibrations
            .iter()
            .map(|&(node, camera, k, dist, model)| {
                let node_name = fbb.create_string(node);
                let intrinsics = fbb.create_vector(&k);
                let dist_coeffs = fbb.create_vector(dist);
                cal::CameraCalibration::create(
                    &mut fbb,
                    &cal::CameraCalibrationArgs {
                        node_name: Some(node_name),
                        camera_number: camera,
                        intrinsics: Some(intrinsics),
                        dist_coeffs: Some(dist_coeffs),
                        distortion_model: model,
                        ..Default::default()
                    },
                )
            })
            .collect();
        let calibrations = fbb.create_vector(&calibrations);
        let root = constants::CameraConstants::create(
            &mut fbb,
            &constants::CameraConstantsArgs {
                calibration: Some(calibrations),
                ..Default::default()
            },
        );
        fbb.finish_minimal(root);
        fbb.finished_data().to_vec()
    }

    #[test]
    fn finds_each_cameras_calibration() {
        let k = |f: f32| [f, 0.0, 800.0, 0.0, f, 650.0, 0.0, 0.0, 1.0];
        let data = camera_constants(&[
            ("orin", 0, k(1000.0), &[0.1, 0.2, 0.3, 0.4, 0.5]),
            ("orin", 1, k(1100.0), &[]),
            ("imu", 0, k(1200.0), &[]),
        ]);
        let zero = Intrinsics::find(&data, "orin", 0).unwrap();
        assert_eq!(zero.fx, 1000.0);
        assert_eq!(zero.cy, 650.0);
        assert_eq!(zero.dist[..5], [0.1f32, 0.2, 0.3, 0.4, 0.5].map(f64::from));
        assert_eq!(zero.dist[5..], [0.0; 3]);
        assert_eq!(Intrinsics::find(&data, "orin", 1).unwrap().fx, 1100.0);
        assert_eq!(Intrinsics::find(&data, "imu", 0).unwrap().fx, 1200.0);
        assert_eq!(Intrinsics::find(&data, "orin", 2), None);
        assert_eq!(Intrinsics::find(&[], "orin", 0), None);
        assert!(!zero.kannala_brandt);

        let fisheye = camera_constants_with_model(&[(
            "orin",
            0,
            k(650.0),
            &[0.025, 0.001, 0.004, -0.002],
            constants::calibration::DistortionModel::KANNALA_BRANDT,
        )]);
        let lens = Intrinsics::find(&fisheye, "orin", 0).unwrap();
        assert!(lens.kannala_brandt);
        assert_eq!(
            lens.dist[..4],
            [0.025f32, 0.001, 0.004, -0.002].map(f64::from)
        );
    }
}
