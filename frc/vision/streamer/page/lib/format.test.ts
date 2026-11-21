import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {
  EXPOSURE_MAX_100US,
  EXPOSURE_MIN_100US,
  EXPOSURE_SLIDER_STEPS,
  exposureMsText,
  exposureSlider,
  exposureText,
  sliderExposure,
  fixedWidth,
  gainText,
  sizeText,
  stoppedText,
  streamSettings,
  streamSizeText,
  tileLine,
} from './format.js';

const F = '\u2007';

test('exposure is in ms with the raw 100 us value, or auto', () => {
  assert.equal(exposureText({exposure100us: 60}), '6.0 ms (60)');
  assert.equal(exposureText({exposure100us: 0}), '0.0 ms (0)');
  assert.equal(exposureText({}), 'auto');
  assert.equal(exposureText(null), 'auto');
});

test('gain is the value, or the driver default', () => {
  assert.equal(gainText({gain: 0}), '0');
  assert.equal(gainText({}), 'driver default');
});

test('size is width by height, or the camera default', () => {
  assert.equal(sizeText({imageWidth: 1600, imageHeight: 1304}), '1600×1304');
  assert.equal(sizeText({imageWidth: 1600}), 'camera default');
  assert.equal(sizeText(null), 'camera default');
});

test('reads the settings from CameraConstants JSON', () => {
  const constants = {
    calibration: [],
    default_camera_stream_settings: {
      image_width: 1600,
      image_height: 1304,
      exposure_100us: 60,
      gain: 0,
    },
  };
  assert.deepEqual(streamSettings(constants), {
    imageWidth: 1600,
    imageHeight: 1304,
    exposure100us: 60,
    gain: 0,
  });
  // A robot that leaves them to the driver.
  assert.deepEqual(streamSettings({default_camera_stream_settings: {}}), {});
  assert.equal(streamSettings({}), null);
  assert.equal(streamSettings('nonsense'), null);
});

test('numbers keep their width', () => {
  assert.equal(fixedWidth(60, 5, 1), `${F}60.0`);
  assert.equal(fixedWidth(9.54, 5, 1), `${F}${F}9.5`);
  assert.equal(fixedWidth(120, 5, 1), '120.0');
  assert.equal(fixedWidth(null, 4), `${F}${F}${F}–`);
  assert.equal(fixedWidth(NaN, 2), `${F}–`);
});

test("a tile's line holds still as its numbers change", () => {
  const line = tileLine({
    robotFps: 60,
    drawnFps: 30,
    kbps: 1040.4,
    delayMs: 12.4,
    tags: 2,
  });
  assert.equal(
    line,
    `${F}60.0 → ${F}30.0 FPS · 1040 kbit/s · ${F}12 ms · ${F}2 tags`,
  );
  const other = tileLine({
    robotFps: 9,
    drawnFps: null,
    kbps: 87,
    delayMs: 104,
    tags: 12,
  });
  assert.equal(other.length, line.length);
  assert.equal(
    tileLine({
      robotFps: null,
      drawnFps: null,
      kbps: null,
      delayMs: null,
      tags: null,
    }),
    `${F}${F}${F}${F}– → ${F}${F}${F}${F}– FPS · ${F}${F}${F}– kbit/s · ${F}${F}– ms · ${F}– tags`,
  );
});

test("a stream's size, once it has one", () => {
  assert.equal(streamSizeText(800, 652), '800×652');
  assert.equal(streamSizeText(null, 652), '–');
});

test('the exposure slider spans 0.1 ms to 33.3 ms, logarithmically', () => {
  assert.equal(sliderExposure(0), EXPOSURE_MIN_100US);
  assert.equal(sliderExposure(EXPOSURE_SLIDER_STEPS), EXPOSURE_MAX_100US);
  // Halfway is the geometric mean, about 1.8 ms.
  assert.equal(sliderExposure(EXPOSURE_SLIDER_STEPS / 2), 18);
  // Out of range clamps.
  assert.equal(sliderExposure(-5), EXPOSURE_MIN_100US);
  assert.equal(exposureSlider(10000), EXPOSURE_SLIDER_STEPS);
  assert.equal(exposureSlider(0), 0);
});

test('an exposure comes back from its slider position', () => {
  for (const exposure of [1, 2, 15, 60, 100, 333]) {
    assert.equal(sliderExposure(exposureSlider(exposure)), exposure);
  }
});

test("a camera's stopped frames say so after a second, in whole seconds", () => {
  assert.equal(stoppedText(0), null);
  assert.equal(stoppedText(800), null);
  assert.equal(stoppedText(1000), 'No image for 1 s');
  assert.equal(stoppedText(12_400), 'No image for 12 s');
});

test('an exposure is in ms, or auto for 0', () => {
  assert.equal(exposureMsText(60), '6.0 ms');
  assert.equal(exposureMsText(0), 'auto');
});
