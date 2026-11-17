import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {
  calibrations,
  parseCalibration,
  withoutTrailingZeros,
} from './intrinsics.js';

// The test robot's /constants (team 0), as /api/latest gives them, cut
// down.  Its calibrations have no ID, date or error; these are made up.
const CONSTANTS = {
  calibration: [
    {
      node_name: 'orin',
      team_number: 0,
      intrinsics: [1997.4701, 0, 810.7046, 0, 1996.0, 700.1, 0, 0, 1],
      dist_coeffs: [0.1, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      calibration_timestamp: 1741280928754949431,
      camera_id: '25-02',
      camera_number: 1,
      reprojection_error: 0.4,
    },
    {
      node_name: 'orin',
      team_number: 0,
      intrinsics: [957.48346, 0, 706.72394, 0, 957.3993, 520.59955, 0, 0, 1],
      dist_coeffs: [
        14.946887, 21.288757, 0.00048430284, 0.00064143946, 19.788067,
        14.776229, 18.951693, 20.182613, 0, 0, 0, 0, 0, 0,
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
  assert.equal(zero.fx, 957.48346);
  assert.equal(zero.fy, 957.3993);
  assert.equal(zero.cx, 706.72394);
  assert.equal(zero.cy, 520.59955);
  assert.equal(zero.team, 0);
  assert.equal(zero.reprojectionError, 0.54337317);
  assert.deepEqual(
    zero.dist,
    [
      14.946887, 21.288757, 0.00048430284, 0.00064143946, 19.788067, 14.776229,
      18.951693, 20.182613,
    ],
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

test('a calibration says which model it is', () => {
  const k = [648, 0, 766, 0, 648, 557, 0, 0, 1];
  assert.equal(
    parseCalibration({node_name: 'orin', intrinsics: k})!.kannalaBrandt,
    false,
  );
  assert.equal(
    parseCalibration({
      node_name: 'orin',
      intrinsics: k,
      distortion_model: 'KANNALA_BRANDT',
      dist_coeffs: [0.025, 0.001, 0.004, -0.002],
    })!.kannalaBrandt,
    true,
  );
});
