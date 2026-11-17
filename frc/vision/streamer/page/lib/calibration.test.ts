import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {
  Board,
  boardProblem,
  Capture,
  coverage,
  guidance,
  parseSession,
  parseStatus,
  pose,
  sameBoard,
  validationRows,
} from './calibration.js';

// What /api/latest gave for /calibration Status on orin-1868-1.
const STATUS = {
  session_id: 'test1',
  state: 'COLLECTING',
  camera: 2,
  verdict: 'TOO_FEW_CORNERS',
  corners: 0,
  min_corners: 15,
  moved: 0,
  motion: 0,
  captures: 0,
  target_captures: 70,
  error: '',
  session_sequence: 3,
};

const BOARD = {
  squares_x: 14,
  squares_y: 9,
  square_length: 0.04,
  marker_length: 0.03,
  dictionary: 'DICT_5X5_100',
};

test('a status says what to do', () => {
  const status = parseStatus(STATUS)!;
  assert.equal(status.state, 'COLLECTING');
  assert.equal(status.camera, 2);
  assert.equal(status.sessionSequence, 3);
  assert.equal(guidance(status), 'Show the camera the board.');
  assert.match(
    guidance({...status, verdict: 'TOO_FEW_CORNERS', corners: 9}),
    /Found 9 corners; need 15/,
  );
  assert.equal(guidance({...status, verdict: 'HOLD_STILL'}), 'Hold still.');
  assert.match(
    guidance({...status, verdict: 'MOVE_TO_NEW_POSE', moved: 0.42}),
    /42%/,
  );
  assert.equal(parseStatus('nope'), null);
});

test('a session parses, solve and all', () => {
  const session = parseSession({
    session_id: 'test1',
    state: 'SOLVED',
    camera: 2,
    camera_id: '25-11',
    image_width: 1600,
    image_height: 1304,
    board: BOARD,
    board_image_path: '/calibration/board.png?v=1',
    boards: [{name: '14x9', board: BOARD, preset: true}],
    target_captures: 70,
    captures: [
      {
        monotonic_timestamp_ns: 5,
        corners: [{x: 10.5, y: 20.25, id: 3}],
        rvec: [0, 0, 0],
        tvec: [0, 0, 1],
      },
    ],
    current: {
      node_name: 'orin',
      camera_number: 2,
      intrinsics: [1000, 0, 800, 0, 1000, 650, 0, 0, 1],
      dist_coeffs: [0.1, 0, 0, 0, 0],
    },
    solve: {
      calibration: {
        node_name: 'orin',
        camera_number: 2,
        intrinsics: [1001, 0, 801, 0, 1001, 651, 0, 0, 1],
      },
      reprojection_error: 0.3,
      capture_errors: [0.3],
      residuals: [{residuals: [{dx: 25, dy: -50}]}],
      valid: true,
      captures: 1,
    },
  })!;
  assert.equal(session.cameraId, '25-11');
  assert.deepEqual(session.board, {
    squaresX: 14,
    squaresY: 9,
    squareLength: 0.04,
    markerLength: 0.03,
    dictionary: 'DICT_5X5_100',
  });
  assert.equal(session.boards[0]!.preset, true);
  assert.deepEqual(session.captures[0]!.corners, [{x: 10.5, y: 20.25, id: 3}]);
  assert.equal(session.current!.fx, 1000);
  assert.equal(session.solve!.calibration!.fx, 1001);
  // Hundredths of a pixel.
  assert.deepEqual(session.solve!.residuals, [[{dx: 0.25, dy: -0.5}]]);
  assert.equal(session.solve!.valid, true);
});

test('boards are checked as calibration_server checks them', () => {
  const board: Board = {
    squaresX: 14,
    squaresY: 9,
    squareLength: 0.04,
    markerLength: 0.03,
    dictionary: 'DICT_5X5_100',
  };
  assert.equal(boardProblem(board), null);
  assert.match(boardProblem({...board, squaresX: 2})!, /3×3/);
  assert.match(boardProblem({...board, markerLength: 0.04})!, /smaller/);
  assert.match(boardProblem({...board, dictionary: 'DICT_9X9'})!, /No ArUco/);
  assert.match(
    boardProblem({...board, squaresX: 15, squaresY: 15})!,
    /112 markers/,
  );
  assert.ok(sameBoard(board, {...board}));
  assert.ok(!sameBoard(board, {...board, squaresX: 13}));
});

function capture(points: [number, number][], rvec = [0, 0, 0]): Capture {
  return {
    timestampNs: 0,
    corners: points.map(([x, y], id) => ({x, y, id})),
    rvec,
    tvec: [0, 0, 2],
  };
}

test('coverage counts captures, not corners', () => {
  const counts = coverage(
    [
      capture([
        [10, 10],
        [20, 20],
        [150, 90],
      ]),
      capture([[15, 15]]),
    ],
    200,
    100,
    2,
    1,
  );
  assert.deepEqual(counts, [2, 1]);
});

test('a pose has a distance and a tilt', () => {
  assert.deepEqual(pose(capture([])), {distance: 2, tiltDeg: 0});
  const tilted = pose(capture([], [Math.PI / 6, 0, 0]));
  assert.ok(Math.abs(tilted.tiltDeg - 30) < 1e-9);
});

test('validation lines up each measure across calibrations', () => {
  const result = (measure: string, median: number) => ({
    check: 'RIGIDITY',
    measure,
    frames: 10,
    median,
    worst: median * 2,
    columns: 2,
    rows: 1,
    means: [median, -1],
  });
  const session = parseSession({
    validation: [
      {name: 'current', results: [result('mm', 3), result('deg', 0.5)]},
      {name: 'solved', results: [result('mm', 1)]},
    ],
    checks: ['RIGIDITY'],
  })!;
  assert.deepEqual(session.checks, ['RIGIDITY']);
  const rows = validationRows(session.validation);
  assert.equal(rows.length, 2);
  assert.equal(rows[0]!.measure, 'mm');
  assert.equal(rows[0]!.byLens[0]!.median, 3);
  assert.equal(rows[0]!.byLens[1]!.median, 1);
  assert.equal(rows[1]!.byLens[1], null);
  assert.deepEqual(rows[0]!.byLens[0]!.means, [3, -1]);
});
