import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {calibrations, withoutTrailingZeros} from './intrinsics.js';

// What /api/latest gave for orin-1868-1's /constants, cut down.
const CONSTANTS = {
  calibration: [
    {
      node_name: 'orin',
      team_number: 4646,
      intrinsics: [1997.4701, 0, 810.7046, 0, 1996.0, 700.1, 0, 0, 1],
      dist_coeffs: [0.1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      calibration_timestamp: 1741280928754949431,
      camera_id: '25-02',
      camera_number: 1,
      reprojection_error: 0.4,
    },
    {
      node_name: 'orin',
      team_number: 4646,
      intrinsics: [1975.3914, 0, 845.55774, 0, 1974.9246, 714.27136, 0, 0, 1],
      dist_coeffs: [
        4.96, 28.2, 0.0017, 0.0028, -7.03, 4.88, 27.27, -6.78, 0, 0, 0, 0, 0, 0,
      ],
      calibration_timestamp: 1741280928754949431,
      camera_id: '25-01',
      camera_number: 0,
      reprojection_error: 0.54337317,
    },
    // Another node's camera, and one whose matrix /api/latest elided.
    {
      node_name: 'imu',
      camera_number: 0,
      intrinsics: [1, 0, 0, 0, 1, 0, 0, 0, 1],
    },
    {node_name: 'orin', camera_number: 2, intrinsics: ['... 9 elements ...']},
  ],
};

test("reads each of the node's cameras' calibrations, in camera order", () => {
  const cards = calibrations(CONSTANTS, 'orin');
  assert.deepEqual(
    cards.map((c) => c.camera),
    [0, 1],
  );
  const zero = cards[0]!;
  assert.equal(zero.cameraId, '25-01');
  assert.equal(zero.fx, 1975.3914);
  assert.equal(zero.fy, 1974.9246);
  assert.equal(zero.cx, 845.55774);
  assert.equal(zero.cy, 714.27136);
  assert.equal(zero.team, 4646);
  assert.equal(zero.reprojectionError, 0.54337317);
  assert.deepEqual(
    zero.dist,
    [4.96, 28.2, 0.0017, 0.0028, -7.03, 4.88, 27.27, -6.78],
  );
  assert.equal(zero.date?.toISOString(), '2025-03-06T17:08:48.754Z');
  assert.deepEqual(cards[1]!.dist, [0.1, 0.2]);
});

test('anything else reads as no calibrations', () => {
  assert.deepEqual(calibrations(null, 'orin'), []);
  assert.deepEqual(calibrations({calibration: 'no'}, 'orin'), []);
  assert.deepEqual(calibrations(CONSTANTS, 'roborio'), []);
});

test('trailing zeros go, the others stay', () => {
  assert.deepEqual(withoutTrailingZeros([1, 0, 2, 0, 0]), [1, 0, 2]);
  assert.deepEqual(withoutTrailingZeros([0, 0]), []);
});
