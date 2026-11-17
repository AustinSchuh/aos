// The cameras' calibrations, as the Intrinsics tab shows them, from the robot's CameraConstants.

/** Above this, a calibration's reprojection error is worth a second look. */
export const REPROJECTION_WARNING_PX = 0.5;

export interface Calibration {
  camera: number;
  cameraId: string;
  node: string;
  team: number | null;
  /** When it was calibrated, if it says. */
  date: Date | null;
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  /** The camera matrix, row-major. */
  k: number[];
  /** The distortion coefficients, without the zeros on the end. */
  dist: number[];
  reprojectionError: number | null;
}

/** `values` without the zeros on the end: OpenCV's 14 coefficients, most of them unused. */
export function withoutTrailingZeros(values: number[]): number[] {
  let end = values.length;
  while (end > 0 && values[end - 1] === 0) {
    --end;
  }
  return values.slice(0, end);
}

function numbers(value: unknown): number[] | null {
  return Array.isArray(value) && value.every((v) => typeof v === 'number')
    ? (value as number[])
    : null;
}

/**
 * Each camera's calibration in a CameraConstants message (as /api/latest gives it), for `node`,
 * by camera number.  Calibrations without a full camera matrix are left out: /api/latest elides
 * long vectors, so ask it for at least 14 elements.
 */
export function calibrations(constants: unknown, node: string): Calibration[] {
  if (typeof constants !== 'object' || constants === null) {
    return [];
  }
  const list = (constants as Record<string, unknown>)['calibration'];
  if (!Array.isArray(list)) {
    return [];
  }
  const result: Calibration[] = [];
  for (const entry of list) {
    const calibration = parseCalibration(entry);
    if (calibration !== null && calibration.node === node) {
      result.push(calibration);
    }
  }
  return result.sort((a, b) => a.camera - b.camera);
}

/**
 * One CameraCalibration, as /api/latest gives it, or null without a full camera matrix (or a
 * node).
 */
export function parseCalibration(entry: unknown): Calibration | null {
  if (typeof entry !== 'object' || entry === null) {
    return null;
  }
  const c = entry as Record<string, unknown>;
  const k = numbers(c['intrinsics']);
  const node = c['node_name'];
  if (typeof node !== 'string' || k === null || k.length !== 9) {
    return null;
  }
  const timestamp = c['calibration_timestamp'];
  const error = c['reprojection_error'];
  const team = c['team_number'];
  return {
    camera: typeof c['camera_number'] === 'number' ? c['camera_number'] : 0,
    cameraId: typeof c['camera_id'] === 'string' ? c['camera_id'] : '',
    node,
    team: typeof team === 'number' ? team : null,
    date:
      typeof timestamp === 'number' && timestamp > 0
        ? new Date(timestamp / 1e6)
        : null,
    fx: k[0]!,
    cx: k[2]!,
    fy: k[4]!,
    cy: k[5]!,
    k,
    dist: withoutTrailingZeros(numbers(c['dist_coeffs']) ?? []),
    reprojectionError: typeof error === 'number' ? error : null,
  };
}
