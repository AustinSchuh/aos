// How far a camera's lens moves each pixel: OpenCV's distortion model, from a calibration's
// camera matrix and coefficients, run backwards from the image the camera takes.

/** The camera matrix's terms, and OpenCV's coefficients in its order. */
export interface Lens {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  /**
   * OpenCV's k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, τx, τy]]]] (τx and τy are
   * ignored), or with `kannalaBrandt`, Kannala-Brandt's k1..k4.
   */
  dist: number[];
  kannalaBrandt?: boolean;
}

/**
 * Where the model puts the normalized, undistorted point (x, y) in the distorted image: OpenCV's,
 * or Kannala-Brandt's, whose distorted radius is a polynomial in the angle from the axis.
 */
export function distort(
  dist: number[],
  x: number,
  y: number,
  kannalaBrandt = false,
): [number, number] {
  const c = (i: number) => dist[i] ?? 0;
  if (kannalaBrandt) {
    const r = Math.hypot(x, y);
    if (r < 1e-12) {
      return [x, y];
    }
    const theta = Math.atan(r);
    const t2 = theta * theta;
    const scale =
      (theta * (1 + t2 * (c(0) + t2 * (c(1) + t2 * (c(2) + t2 * c(3)))))) / r;
    return [x * scale, y * scale];
  }
  const r2 = x * x + y * y;
  const r4 = r2 * r2;
  const r6 = r4 * r2;
  const radial =
    (1 + c(0) * r2 + c(1) * r4 + c(4) * r6) /
    (1 + c(5) * r2 + c(6) * r4 + c(7) * r6);
  const [p1, p2] = [c(2), c(3)];
  return [
    x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x) + c(8) * r2 + c(9) * r4,
    y * radial +
      p1 * (r2 + 2 * y * y) +
      2 * p2 * x * y +
      c(10) * r2 +
      c(11) * r4,
  ];
}

/**
 * The undistorted normalized point that `distort` takes to (xd, yd), by Newton's method from
 * (xd, yd), or null if it doesn't converge (far outside where the calibration has data, a
 * high-order model can fold back on itself).
 */
export function undistort(
  dist: number[],
  xd: number,
  yd: number,
  kannalaBrandt = false,
): [number, number] | null {
  let [x, y] = [xd, yd];
  const h = 1e-7;
  for (let i = 0; i < 30; ++i) {
    const [fx, fy] = distort(dist, x, y, kannalaBrandt);
    const [ex, ey] = [fx - xd, fy - yd];
    if (Math.hypot(ex, ey) < 1e-10) {
      return [x, y];
    }
    // The Jacobian, by finite differences.
    const [ax, ay] = distort(dist, x + h, y, kannalaBrandt);
    const [bx, by] = distort(dist, x, y + h, kannalaBrandt);
    const [j00, j10, j01, j11] = [
      (ax - fx) / h,
      (ay - fy) / h,
      (bx - fx) / h,
      (by - fy) / h,
    ];
    const det = j00 * j11 - j01 * j10;
    if (!Number.isFinite(det) || Math.abs(det) < 1e-12) {
      return null;
    }
    x -= (j11 * ex - j01 * ey) / det;
    y -= (j00 * ey - j10 * ex) / det;
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return null;
    }
  }
  const [fx, fy] = distort(dist, x, y, kannalaBrandt);
  return Math.hypot(fx - xd, fy - yd) < 1e-6 ? [x, y] : null;
}

/**
 * How far, in pixels, undistorting moves the camera's pixel (u, v): where an ideal pinhole camera
 * would have seen what the lens put at (u, v), less (u, v).  Null where the model doesn't invert.
 */
export function displacement(
  lens: Lens,
  u: number,
  v: number,
): [number, number] | null {
  const point = undistort(
    lens.dist,
    (u - lens.cx) / lens.fx,
    (v - lens.cy) / lens.fy,
    lens.kannalaBrandt ?? false,
  );
  if (point === null) {
    return null;
  }
  return [point[0] * lens.fx + lens.cx - u, point[1] * lens.fy + lens.cy - v];
}

/** `displacement` at the centers of a `columns` by `rows` grid of cells over the image. */
export interface DisplacementField {
  columns: number;
  rows: number;
  /** dx, dy for each cell, row-major; NaN where the model doesn't invert. */
  dx: Float64Array;
  dy: Float64Array;
  /** The largest displacement, in pixels. */
  max: number;
}

export function displacementField(
  lens: Lens,
  width: number,
  height: number,
  columns: number,
  rows: number,
): DisplacementField {
  const dx = new Float64Array(columns * rows);
  const dy = new Float64Array(columns * rows);
  let max = 0;
  for (let row = 0; row < rows; ++row) {
    for (let column = 0; column < columns; ++column) {
      const d = displacement(
        lens,
        ((column + 0.5) * width) / columns,
        ((row + 0.5) * height) / rows,
      );
      const i = row * columns + column;
      dx[i] = d ? d[0] : NaN;
      dy[i] = d ? d[1] : NaN;
      if (d) {
        max = Math.max(max, Math.hypot(d[0], d[1]));
      }
    }
  }
  return {columns, rows, dx, dy, max};
}

/** The cells across the distortion maps' heatmaps; their rows follow the image's shape. */
export const MAP_COLUMNS = 160;

/** The rows of a map `columns` across a `width` by `height` image. */
export function mapRows(
  columns: number,
  width: number,
  height: number,
): number {
  return Math.max(1, Math.round((columns * height) / width));
}

/**
 * The size of the image `lens` is calibrated for: `width` and `height` if known, or else as if
 * the principal point were in the middle.
 */
export function imageSize(
  lens: Lens,
  width: number | null,
  height: number | null,
): [number, number] {
  return [width ?? Math.round(2 * lens.cx), height ?? Math.round(2 * lens.cy)];
}

/**
 * The largest move on any of the maps of `lenses`, so they can share one scale; 0 if they have
 * none.
 */
export function sharedMax(
  lenses: Lens[],
  width: number | null,
  height: number | null,
): number {
  let max = 0;
  for (const lens of lenses) {
    const [w, h] = imageSize(lens, width, height);
    if (w > 0 && h > 0) {
      max = Math.max(
        max,
        displacementField(lens, w, h, MAP_COLUMNS, mapRows(MAP_COLUMNS, w, h))
          .max,
      );
    }
  }
  return max;
}
