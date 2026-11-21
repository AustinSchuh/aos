import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {
  detectionDelayMs,
  FrameWatch,
  RateWindow,
  robotFps,
  streamRates,
  tagCount,
} from './rates.js';

const sample = (
  timestampMs: number,
  framesDecoded: number,
  bytesReceived: number,
) => ({
  timestampMs,
  framesDecoded,
  bytesReceived,
  frameWidth: 800,
  frameHeight: 652,
});

test('stream rates are over the time between samples', () => {
  // Half a second: 15 frames and 65 kB is 30 fps and 1040 kbit/s.
  assert.deepEqual(
    streamRates(sample(1000, 100, 1_000_000), sample(1500, 115, 1_065_000)),
    {
      fps: 30,
      kbps: 1040,
    },
  );
});

test('no rates from samples that can not say', () => {
  assert.equal(streamRates(sample(1000, 100, 0), sample(1000, 101, 10)), null);
  // A new track restarts its counters.
  assert.equal(streamRates(sample(1000, 100, 1000), sample(2000, 3, 10)), null);
});

test('a window gives the rates over its last second, however often it is sampled', () => {
  const window = new RateWindow(1000);
  assert.equal(window.add(sample(0, 0, 0)), null);
  // 30 fps and 1 Mbit/s, sampled every 400 ms.
  assert.deepEqual(window.add(sample(400, 12, 50_000)), {fps: 30, kbps: 1000});
  assert.deepEqual(window.add(sample(800, 24, 100_000)), {fps: 30, kbps: 1000});
  assert.deepEqual(window.add(sample(1200, 36, 150_000)), {
    fps: 30,
    kbps: 1000,
  });
  // A frame late: over the last second and more (from 400 ms), not just since 1200.
  assert.deepEqual(window.add(sample(1600, 47, 200_000)), {
    fps: 35 / 1.2,
    kbps: 1000,
  });
  // A new track starts again.
  assert.equal(window.add(sample(2000, 2, 1_000)), null);
});

test('a frame watch says how long since the last new frame', () => {
  const watch = new FrameWatch();
  assert.equal(watch.add(10, 0), 0);
  assert.equal(watch.add(22, 400), 0);
  // Stopped at 22.
  assert.equal(watch.add(22, 800), 400);
  assert.equal(watch.add(22, 3200), 2800);
  // Coming again.
  assert.equal(watch.add(23, 3600), 0);
  // A new track's counters start again: that's frames too.
  assert.equal(watch.add(2, 4000), 0);
  assert.equal(watch.add(2, 5000), 1000);
});

test("the robot's frame rate is decodes over the time between statuses", () => {
  assert.equal(
    robotFps(null, {sentTime: 10, successfulDecodes: 60, periodSeconds: 0}),
    null,
  );
  assert.equal(
    robotFps(
      {sentTime: 10, successfulDecodes: 61, periodSeconds: 0},
      {sentTime: 11, successfulDecodes: 60, periodSeconds: 0},
    ),
    60,
  );
  assert.equal(
    robotFps(
      {sentTime: 10, successfulDecodes: 61, periodSeconds: 0},
      {sentTime: 10.5, successfulDecodes: 15, periodSeconds: 0},
    ),
    30,
  );
  assert.equal(
    robotFps(
      {sentTime: 10, successfulDecodes: 61, periodSeconds: 0},
      {sentTime: 10, successfulDecodes: 60, periodSeconds: 0},
    ),
    null,
  );
});

test('a status that says its period gives the rate by itself', () => {
  assert.equal(
    robotFps(null, {sentTime: 10, successfulDecodes: 30, periodSeconds: 0.5}),
    60,
  );
  // Its own period, not the time since another, which may have been lost.
  assert.equal(
    robotFps(
      {sentTime: 7, successfulDecodes: 61, periodSeconds: 1},
      {sentTime: 10, successfulDecodes: 60, periodSeconds: 1},
    ),
    60,
  );
});

test("the detector's delay is from capture to its TargetMap", () => {
  assert.equal(
    detectionDelayMs({
      monotonic_event_time_ns: 16_082_152_524_930,
      message: {monotonic_timestamp_ns: 16_082_140_024_930},
    }),
    12.5,
  );
  assert.equal(
    detectionDelayMs({monotonic_event_time_ns: 5, message: {}}),
    null,
  );
  assert.equal(
    detectionDelayMs({
      monotonic_event_time_ns: 5,
      message: {monotonic_timestamp_ns: 9},
    }),
    null,
  );
  assert.equal(detectionDelayMs(null), null);
});

test('tags are counted from the TargetMap', () => {
  assert.equal(tagCount({target_poses: [{id: 2}, {id: 3}]}), 2);
  assert.equal(tagCount({}), 0);
  assert.equal(tagCount(null), 0);
});
