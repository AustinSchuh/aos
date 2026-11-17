import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {
  displacement,
  displacementField,
  imageSize,
  sharedMax,
  distort,
  Lens,
  undistort,
} from './distortion.js';

// Camera 0 of the test robot (team 0): a rational model with large terms.
const RATIONAL: Lens = {
  fx: 957.48346,
  fy: 957.3993,
  cx: 706.72394,
  cy: 520.59955,
  dist: [
    14.946887, 21.288757, 0.00048430284, 0.00064143946, 19.788067, 14.776229,
    18.951693, 20.182613,
  ],
};

test('no coefficients move nothing', () => {
  const lens: Lens = {fx: 1000, fy: 1000, cx: 800, cy: 650, dist: []};
  assert.deepEqual(displacement(lens, 10, 1200), [0, 0]);
});

test('undistort inverts distort', () => {
  for (const [x, y] of [
    [0, 0],
    [0.3, -0.2],
    [-0.4, 0.35],
  ] as [number, number][]) {
    const [xd, yd] = distort(RATIONAL.dist, x, y);
    const back = undistort(RATIONAL.dist, xd, yd);
    assert.ok(back !== null);
    assert.ok(Math.abs(back[0] - x) < 1e-9 && Math.abs(back[1] - y) < 1e-9);
  }
});

test('barrel distortion moves the corners out', () => {
  // k1 < 0 pulls the image in, so undistorting pushes each corner away from the center.
  const lens: Lens = {fx: 1000, fy: 1000, cx: 800, cy: 650, dist: [-0.05]};
  const d = displacement(lens, 0, 0);
  assert.ok(d !== null);
  assert.ok(d[0] < 0 && d[1] < 0);
  assert.deepEqual(displacement(lens, 800, 650), [0, 0]);
});

test('the field covers the image, and finds the largest move', () => {
  const lens: Lens = {fx: 1000, fy: 1000, cx: 800, cy: 650, dist: [-0.05]};
  const field = displacementField(lens, 1600, 1300, 4, 2);
  assert.equal(field.dx.length, 8);
  const corner = displacement(lens, 200, 325)!;
  assert.ok(Math.abs(field.dx[0]! - corner[0]) < 1e-9);
  assert.ok(Math.abs(field.max - Math.hypot(...corner)) < 1e-9);
});

test('the maps share the largest move of any of them', () => {
  const mild: Lens = {fx: 1000, fy: 1000, cx: 800, cy: 650, dist: [-0.01]};
  const strong: Lens = {...mild, dist: [-0.05]};
  const max = sharedMax([mild, strong], 1600, 1300);
  assert.ok(max > 0);
  assert.equal(max, sharedMax([strong], 1600, 1300));
  assert.ok(sharedMax([mild], 1600, 1300) < max);
  assert.equal(sharedMax([], null, null), 0);
});

test('without a size, the principal point is the middle', () => {
  const lens: Lens = {fx: 1000, fy: 1000, cx: 800.4, cy: 650.2, dist: []};
  assert.deepEqual(imageSize(lens, null, null), [1601, 1300]);
  assert.deepEqual(imageSize(lens, 1600, 1304), [1600, 1304]);
});

test('kannala-brandt distorts by the angle, and inverts', () => {
  // No coefficients: a point 45 degrees off the axis lands at pi / 4.
  const [xd, yd] = distort([], 1, 0, true);
  assert.ok(Math.abs(xd - Math.PI / 4) < 1e-12);
  assert.equal(yd, 0);
  const dist = [0.025, 0.001, 0.004, -0.002];
  for (const [x, y] of [
    [0.3, -0.2],
    [2.5, 1.5],
  ] as [number, number][]) {
    const [dx, dy] = distort(dist, x, y, true);
    const back = undistort(dist, dx, dy, true);
    assert.ok(back !== null);
    assert.ok(Math.abs(back[0] - x) < 1e-6 && Math.abs(back[1] - y) < 1e-6);
  }
  // The wide lens's corners move a long way: 75 degrees off the axis.
  const lens: Lens = {
    fx: 648,
    fy: 648,
    cx: 766,
    cy: 557,
    dist,
    kannalaBrandt: true,
  };
  const d = displacement(lens, 0, 0);
  assert.ok(d !== null);
  assert.ok(d[0] < -100 && d[1] < -100);
});
