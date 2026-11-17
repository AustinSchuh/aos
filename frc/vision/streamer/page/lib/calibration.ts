// A calibration session, as the Calibrate tab shows and drives it: calibration_server's
// /calibration Status and Session messages as /api/latest gives them, the commands the page
// sends, and what's drawn from them.

import {Calibration, parseCalibration} from './intrinsics.js';

export type SessionState =
  'IDLE' | 'COLLECTING' | 'SOLVING' | 'SOLVED' | 'VALIDATING';

export type ValidationCheck = 'STRAIGHTNESS' | 'RIGIDITY' | 'TWO_TAG';

/** One measure of one check, over the frames looked at. */
export interface CheckResult {
  check: ValidationCheck;
  measure: string;
  frames: number;
  median: number;
  worst: number;
  columns: number;
  rows: number;
  /** Each cell's mean, row-major; negative where nothing was measured. */
  means: number[];
}

/** A calibration's checks. */
export interface LensValidation {
  name: string;
  results: CheckResult[];
  tagIds: number[];
  tagDistanceM: number;
}

export type FrameVerdict =
  | 'NO_IMAGE'
  | 'TOO_FEW_CORNERS'
  | 'NO_POSE'
  | 'MOVE_TO_NEW_POSE'
  | 'HOLD_STILL'
  | 'CAPTURED'
  | 'NOT_COLLECTING'
  | 'TARGET_REACHED';

/** calibration_server's live summary, at 5 Hz. */
export interface SessionStatus {
  sessionId: string;
  state: SessionState;
  camera: number;
  verdict: FrameVerdict;
  corners: number;
  minCorners: number;
  /** How far the board has moved from the last capture, as a fraction of what's needed. */
  moved: number;
  /** How much it moved since the last frame, as a fraction of the still limit. */
  motion: number;
  captures: number;
  targetCaptures: number;
  error: string;
  /** Changes when there's a new Session to fetch. */
  sessionSequence: number;
}

/** A ChArUco board; lengths in meters. */
export interface Board {
  squaresX: number;
  squaresY: number;
  squareLength: number;
  markerLength: number;
  dictionary: string;
}

export interface SavedBoard {
  name: string;
  board: Board;
  preset: boolean;
}

export interface Corner {
  x: number;
  y: number;
  id: number;
}

export interface Capture {
  timestampNs: number;
  corners: Corner[];
  /** The board's pose, board to camera: angle-axis, then meters. */
  rvec: number[];
  tvec: number[];
}

export interface Solve {
  calibration: Calibration | null;
  /** The CameraCalibration as calibration_server gave it, to save as it is. */
  raw: unknown;
  reprojectionError: number;
  captureErrors: number[];
  /** Each capture's corners' errors, in pixels; empty if left out. */
  residuals: {dx: number; dy: number}[][];
  valid: boolean;
  validationError: string;
  savedPath: string;
  downloadPath: string;
  /** How many of the captures it was solved from. */
  captures: number;
  error: string;
}

/** The whole session, as calibration_server sends it after each change. */
export interface Session {
  sessionId: string;
  state: SessionState;
  camera: number;
  cameraId: string;
  imageWidth: number;
  imageHeight: number;
  board: Board | null;
  boardImagePath: string;
  boards: SavedBoard[];
  targetCaptures: number;
  captures: Capture[];
  /** The camera's calibration when the session started. */
  current: Calibration | null;
  solve: Solve | null;
  validation: LensValidation[];
  checks: ValidationCheck[];
  /** Where robot_server serves the last export of the captures' images. */
  imagesDownloadPath: string;
  imagesExported: number;
}

type Json = Record<string, unknown>;

function object(value: unknown): Json | null {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
    ? (value as Json)
    : null;
}

function num(value: unknown, fallback = 0): number {
  return typeof value === 'number' && Number.isFinite(value) ? value : fallback;
}

function str(value: unknown): string {
  return typeof value === 'string' ? value : '';
}

function list(value: unknown): unknown[] {
  return Array.isArray(value) ? value : [];
}

const STATES: SessionState[] = [
  'IDLE',
  'COLLECTING',
  'SOLVING',
  'SOLVED',
  'VALIDATING',
];

function state(value: unknown): SessionState {
  return STATES.includes(value as SessionState)
    ? (value as SessionState)
    : 'IDLE';
}

export function parseStatus(message: unknown): SessionStatus | null {
  const m = object(message);
  if (m === null) {
    return null;
  }
  return {
    sessionId: str(m['session_id']),
    state: state(m['state']),
    camera: num(m['camera'], -1),
    verdict: (str(m['verdict']) || 'NOT_COLLECTING') as FrameVerdict,
    corners: num(m['corners']),
    minCorners: num(m['min_corners'], 15),
    moved: num(m['moved']),
    motion: num(m['motion']),
    captures: num(m['captures']),
    targetCaptures: num(m['target_captures'], 70),
    error: str(m['error']),
    sessionSequence: num(m['session_sequence']),
  };
}

export function parseBoard(value: unknown): Board | null {
  const b = object(value);
  if (b === null) {
    return null;
  }
  return {
    squaresX: num(b['squares_x']),
    squaresY: num(b['squares_y']),
    squareLength: num(b['square_length']),
    markerLength: num(b['marker_length']),
    dictionary: str(b['dictionary']),
  };
}

/** A board as calibration.fbs's BoardSpec, as JSON. */
export function boardJson(board: Board): Json {
  return {
    squares_x: board.squaresX,
    squares_y: board.squaresY,
    square_length: board.squareLength,
    marker_length: board.markerLength,
    dictionary: board.dictionary,
  };
}

function parseSolve(value: unknown): Solve | null {
  const s = object(value);
  if (s === null) {
    return null;
  }
  return {
    calibration: parseCalibration(s['calibration']),
    raw: s['calibration'] ?? null,
    reprojectionError: num(s['reprojection_error']),
    captureErrors: list(s['capture_errors']).map((e) => num(e)),
    residuals: list(s['residuals']).map((capture) =>
      list(object(capture)?.['residuals']).map((r) => ({
        dx: num(object(r)?.['dx']) / 100,
        dy: num(object(r)?.['dy']) / 100,
      })),
    ),
    valid: s['valid'] === true,
    validationError: str(s['validation_error']),
    savedPath: str(s['saved_path']),
    downloadPath: str(s['download_path']),
    captures: num(s['captures']),
    error: str(s['error']),
  };
}

export function parseSession(message: unknown): Session | null {
  const m = object(message);
  if (m === null) {
    return null;
  }
  return {
    sessionId: str(m['session_id']),
    state: state(m['state']),
    camera: num(m['camera'], -1),
    cameraId: str(m['camera_id']),
    imageWidth: num(m['image_width']),
    imageHeight: num(m['image_height']),
    board: parseBoard(m['board']),
    boardImagePath: str(m['board_image_path']),
    boards: list(m['boards'])
      .map((entry) => {
        const e = object(entry);
        const board = parseBoard(e?.['board']);
        return e && board
          ? {name: str(e['name']), board, preset: e['preset'] === true}
          : null;
      })
      .filter((b): b is SavedBoard => b !== null),
    targetCaptures: num(m['target_captures'], 70),
    captures: list(m['captures']).map((entry) => {
      const c = object(entry);
      return {
        timestampNs: num(c?.['monotonic_timestamp_ns']),
        corners: list(c?.['corners']).map((corner) => ({
          x: num(object(corner)?.['x']),
          y: num(object(corner)?.['y']),
          id: num(object(corner)?.['id']),
        })),
        rvec: list(c?.['rvec']).map((v) => num(v)),
        tvec: list(c?.['tvec']).map((v) => num(v)),
      };
    }),
    current: parseCalibration(m['current']),
    solve: parseSolve(m['solve']),
    validation: list(m['validation']).map((entry) => {
      const v = object(entry);
      return {
        name: str(v?.['name']),
        results: list(v?.['results']).map((result) => {
          const r = object(result);
          return {
            check: str(r?.['check']) as ValidationCheck,
            measure: str(r?.['measure']),
            frames: num(r?.['frames']),
            median: num(r?.['median']),
            worst: num(r?.['worst']),
            columns: num(r?.['columns']),
            rows: num(r?.['rows']),
            means: list(r?.['means']).map((mean) => num(mean, -1)),
          };
        }),
        tagIds: list(v?.['tag_ids']).map((id) => num(id)),
        tagDistanceM: num(v?.['tag_distance_m']),
      };
    }),
    checks: list(m['checks']).map((check) => str(check) as ValidationCheck),
    imagesDownloadPath: str(m['images_download_path']),
    imagesExported: num(m['images_exported']),
  };
}

/**
 * The measures validation made, each with every calibration's result for it side by side, in
 * the order they came.
 */
export function validationRows(
  validation: LensValidation[],
): {measure: string; check: ValidationCheck; byLens: (CheckResult | null)[]}[] {
  const rows: {
    measure: string;
    check: ValidationCheck;
    byLens: (CheckResult | null)[];
  }[] = [];
  validation.forEach((lens, index) => {
    for (const result of lens.results) {
      let row = rows.find((r) => r.measure === result.measure);
      if (!row) {
        row = {
          measure: result.measure,
          check: result.check,
          byLens: validation.map(() => null),
        };
        rows.push(row);
      }
      row.byLens[index] = result;
    }
  });
  return rows;
}

/** What to tell the person holding the board, from the last frame. */
export function guidance(status: SessionStatus): string {
  switch (status.verdict) {
    case 'NO_IMAGE':
      return 'No images from this camera.';
    case 'TOO_FEW_CORNERS':
      return status.corners === 0
        ? 'Show the camera the board.'
        : `Found ${status.corners} corners; need ${status.minCorners}. Show more of the board.`;
    case 'NO_POSE':
      return "Found the corners, but can't tell where the board is. Hold it flatter to the camera.";
    case 'MOVE_TO_NEW_POSE':
      return `Move or tilt the board to a new pose (${Math.round(Math.min(status.moved, 1) * 100)}% of the way).`;
    case 'HOLD_STILL':
      return 'Hold still.';
    case 'CAPTURED':
      return 'Captured. Move to a new pose.';
    case 'TARGET_REACHED':
      return `${status.captures} captures: solve, or capture more by hand.`;
    case 'NOT_COLLECTING':
      return status.state === 'SOLVING'
        ? 'Solving…'
        : status.state === 'VALIDATING'
          ? 'Validating: move the board all around the image, near and far, and tilted.'
          : '';
  }
}

/** The ArUco dictionaries calibration_server knows, and their sizes. */
export const DICTIONARIES: ReadonlyArray<[string, number]> = [
  ['DICT_4X4_50', 50],
  ['DICT_4X4_100', 100],
  ['DICT_4X4_250', 250],
  ['DICT_4X4_1000', 1000],
  ['DICT_5X5_50', 50],
  ['DICT_5X5_100', 100],
  ['DICT_5X5_250', 250],
  ['DICT_5X5_1000', 1000],
  ['DICT_6X6_50', 50],
  ['DICT_6X6_100', 100],
  ['DICT_6X6_250', 250],
  ['DICT_6X6_1000', 1000],
  ['DICT_7X7_50', 50],
  ['DICT_7X7_100', 100],
  ['DICT_7X7_250', 250],
  ['DICT_7X7_1000', 1000],
  ['DICT_ARUCO_ORIGINAL', 1024],
];

/** Why `board` won't do, as calibration_server would refuse it, or null if it will. */
export function boardProblem(board: Board): string | null {
  if (
    !Number.isInteger(board.squaresX) ||
    !Number.isInteger(board.squaresY) ||
    board.squaresX < 3 ||
    board.squaresY < 3
  ) {
    return 'A board needs at least 3×3 squares.';
  }
  if (!(board.squareLength > 0)) {
    return 'A square must be bigger than 0.';
  }
  if (!(board.markerLength > 0) || board.markerLength >= board.squareLength) {
    return 'A marker must be smaller than its square.';
  }
  const size = DICTIONARIES.find(([name]) => name === board.dictionary)?.[1];
  if (size === undefined) {
    return `No ArUco dictionary is named ${board.dictionary}.`;
  }
  const markers = Math.floor((board.squaresX * board.squaresY) / 2);
  if (markers > size) {
    return `A ${board.squaresX}×${board.squaresY} board needs ${markers} markers, more than ${board.dictionary}'s ${size}.`;
  }
  return null;
}

export function sameBoard(a: Board | null, b: Board | null): boolean {
  return (
    a !== null &&
    b !== null &&
    a.squaresX === b.squaresX &&
    a.squaresY === b.squaresY &&
    Math.abs(a.squareLength - b.squareLength) < 1e-9 &&
    Math.abs(a.markerLength - b.markerLength) < 1e-9 &&
    a.dictionary === b.dictionary
  );
}

/**
 * How many captures put a corner in each of `columns` × `rows` cells over the image, row-major:
 * a capture counts once a cell, however many of its corners are there.
 */
export function coverage(
  captures: Capture[],
  width: number,
  height: number,
  columns: number,
  rows: number,
): number[] {
  const counts = new Array<number>(columns * rows).fill(0);
  if (!(width > 0) || !(height > 0)) {
    return counts;
  }
  for (const capture of captures) {
    const cells = new Set<number>();
    for (const corner of capture.corners) {
      const column = Math.floor((corner.x / width) * columns);
      const row = Math.floor((corner.y / height) * rows);
      if (column >= 0 && column < columns && row >= 0 && row < rows) {
        cells.add(row * columns + column);
      }
    }
    for (const cell of cells) {
      counts[cell]! += 1;
    }
  }
  return counts;
}

/** A capture's board: how far away, and how tilted from facing the camera. */
export function pose(capture: Capture): {distance: number; tiltDeg: number} {
  const [rx = 0, ry = 0, rz = 0] = capture.rvec;
  const [tx = 0, ty = 0, tz = 0] = capture.tvec;
  const angle = Math.hypot(rx, ry, rz);
  // How much the board's normal (its z axis) points along the camera's: the rotation's bottom
  // right element, cos θ + (1 - cos θ) k_z² by Rodrigues' formula.
  let nz = 1;
  if (angle > 0) {
    const c = Math.cos(angle);
    nz = c + (1 - c) * (rz / angle) * (rz / angle);
  }
  return {
    distance: Math.hypot(tx, ty, tz),
    tiltDeg: (Math.acos(Math.min(1, Math.abs(nz))) * 180) / Math.PI,
  };
}

/** A command for calibration_server, as JSON for robot_server's /api/send. */
export interface Command {
  action: string;
  session_id?: string;
  camera?: number;
  board?: Json;
  board_name?: string;
  target_captures?: number;
  drop?: number[];
  force?: boolean;
  camera_id?: string;
  checks?: ValidationCheck[];
  validate_solved?: boolean;
}

/** A new session's id. */
export function newSessionId(): string {
  return `${Date.now().toString(36)}-${Math.floor(Math.random() * 1e6).toString(36)}`;
}

export const CAMERA_ID_PATTERN = /^[0-9]{2}-[0-9]{2}$/;
