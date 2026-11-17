// End-to-end tests of the vision page: the built page, in headless Chrome,
// against a fake robot_server (see fake_robot.mjs).  The robot's own pieces
// are tested on their own; these check that the page shows what the robot
// says and sends what the robot expects.
//
// Each test saves a screenshot to the test's undeclared outputs, to look at.

import {strict as assert} from "node:assert";
import fs from "node:fs";
import path from "node:path";
import {after, before, beforeEach, test} from "node:test";

import {Browser} from "./browser.mjs";
import {FakeRobot} from "./fake_robot.mjs";

const RUNFILES = process.env.JS_BINARY__RUNFILES;
const runfile = (rlocation) => path.join(RUNFILES, rlocation);
const OUTPUTS = process.env.TEST_UNDECLARED_OUTPUTS_DIR;

// What the robot's /constants says, cut down: two cameras' settings and one
// calibration.
const CONSTANTS = {
  default_camera_stream_settings: {
    image_width: 1600,
    image_height: 1304,
    exposure_100us: 60,
    gain: 0,
  },
  calibration: [
    {
      node_name: "orin",
      intrinsics: [1975.39, 0, 845.56, 0, 1974.92, 714.27, 0, 0, 1],
      dist_coeffs: [-0.02, 0.05, 0, 0, -0.03],
      distortion_model: "KANNALA_BRANDT",
      calibration_timestamp: 1741280928754949431,
      camera_id: "25-01",
      camera_number: 0,
      reprojection_error: 0.21,
    },
  ],
};

const BOARD = {
  name: "12x9 5x5",
  squares_x: 12,
  squares_y: 9,
  square_length: 0.06,
  marker_length: 0.0436,
  dictionary: "DICT_5X5_1000",
};

let robot;
let browser;
let page;

before(async () => {
  robot = new FakeRobot({
    visionDir: runfile(process.env.VISION_DIR),
    rootDir: path.dirname(runfile(process.env.STARTER_HTML)),
    certDir: path.dirname(runfile(process.env.TEST_CERT)),
    cameras: [0, 2],
  });
  await robot.start();
  // The sysroot holding SYSROOT_FILE, usr/bin/ptxas.
  const sysroot = path.resolve(
    path.dirname(runfile(process.env.SYSROOT_FILE)),
    "../..",
  );
  browser = await Browser.launch(runfile(process.env.CHROME), sysroot);
});

after(async () => {
  await browser?.close();
  await robot?.stop();
});

beforeEach(async () => {
  robot.sent = [];
  robot.latest.clear();
  robot.failNextSend = null;
  robot.setLatest("/constants", "frc.vision.CameraConstants", CONSTANTS);
  await page?.close();
  page = await browser.open(`${robot.origin}/vision/`);
  await page.waitFor(`document.querySelector('vision-app nav')`, {
    what: "the page to load",
  });
});

async function screenshot(name) {
  if (OUTPUTS) {
    fs.writeFileSync(
      path.join(OUTPUTS, `${name}.png`),
      await page.screenshot(),
    );
  }
}

/** Waits until the fake robot has `count` sends on `name`, and returns them. */
async function sends(name, count) {
  const deadline = Date.now() + 5000;
  while (robot.sentOn(name).length < count) {
    if (Date.now() > deadline) {
      assert.fail(
        `Expected ${count} sends on ${name}; got ${JSON.stringify(robot.sentOn(name))}`,
      );
    }
    await new Promise((resolve) => setTimeout(resolve, 50));
  }
  return robot.sentOn(name);
}

function tab(label) {
  return page.clickButton(label);
}

test("the feeds show a tile for each camera the streamer has", async () => {
  const names = await page.waitFor(
    `(() => {
      const names = [...document.querySelectorAll('.tile-name')]
        .map((e) => e.textContent.trim());
      return names.length === 2 ? names : null;
    })()`,
    {what: "two camera tiles"},
  );
  assert.deepEqual(names, ["Camera 0", "Camera 2"]);
  // No video from the fake robot.
  assert.equal(await page.text(".no-signal"), "No signal");
  await screenshot("feeds");
});

test("every tab shows its own view", async () => {
  const views = {
    Feeds: "main.feeds",
    Calibrate: "main.calibrate",
    Settings: "main.settings",
    Field: 'main.embed iframe[src="/field.html"]',
    Starter: 'main.embed iframe[src="/starter.html"]',
  };
  for (const [label, selector] of Object.entries(views)) {
    await tab(label);
    await page.waitFor(`document.querySelector(${JSON.stringify(selector)})`, {
      what: `the ${label} tab`,
    });
    const active = await page.text("nav button.active");
    assert.equal(active, label);
    assert.equal(
      await page.evaluate(`document.querySelectorAll('main').length`),
      1,
      `only the ${label} tab's view shows`,
    );
  }
});

test("the starter tab embeds the robot’s starter page", async () => {
  await tab("Starter");
  // The real starter.html, waiting: web_proxy, which would carry starter's
  // status, is down on the fake robot.
  const headings = await page.waitFor(
    `(() => {
      const doc = document.querySelector('main.embed iframe').contentDocument;
      const headings = doc && [...doc.querySelectorAll('th')]
        .map((th) => th.textContent.trim());
      return headings && headings.length > 0 ? headings : null;
    })()`,
    {what: "the starter page in its frame"},
  );
  assert.deepEqual(headings, [
    "Name",
    "Node",
    "State",
    "Exit code",
    "PID",
    "Last start",
    "Last stop reason",
    "Binary",
  ]);
  // Fixed column widths, so a state changing length moves nothing.
  assert.equal(
    await page.evaluate(`(() => {
      const doc = document.querySelector('main.embed iframe').contentDocument;
      return getComputedStyle(doc.querySelector('table')).tableLayout;
    })()`),
    "fixed",
  );
  await screenshot("starter");
});

test("settings show the constants until the cameras have been sent any", async () => {
  await tab("Settings");
  await page.waitFor(`document.querySelector('camera-settings-card')`);
  // The constants' exposure, once they've come ("–" until then).
  assert.equal(
    await page.waitFor(
      `(() => {
        const text = document.querySelector('.camera-settings .setting-value')
          ?.textContent.trim();
        return text && text !== '–' ? text : null;
      })()`,
      {what: "the constants' exposure"},
    ),
    "6.0 ms",
  );
  assert.equal(
    await page.evaluate(`document.querySelector('#gain').value`),
    "0",
  );
  assert.match(
    await page.text(".camera-settings .source"),
    /exposure 6\.0 ms, gain 0/,
  );
  // The calibration's card.
  assert.equal(
    await page.waitFor(
      `document.querySelector('intrinsics-card .card-id')?.textContent.trim()`,
    ),
    "25-01",
  );
  await screenshot("settings");
});

test("settings show what the cameras were last sent, over the constants", async () => {
  robot.setLatest("/camera0", "frc.vision.CameraStreamSettings", {
    exposure_100us: 125,
    gain: 3,
  });
  await tab("Settings");
  await page.waitFor(
    `document.querySelector('.camera-settings .setting-value')?.textContent.trim() === '12.5 ms'`,
    {what: "the exposure the cameras were sent"},
  );
  await page.waitFor(`document.querySelector('#gain').value === '3'`, {
    what: "the gain the cameras were sent",
  });
});

test("dragging the exposure sends it to every camera, throttled", async () => {
  await tab("Settings");
  await page.waitFor(`document.querySelector('#exposure')`);
  // A drag: many positions in quick succession.
  for (const position of [400, 450, 500, 550, 600]) {
    await page.setInput("#exposure", position);
  }
  // The last position is sent, though not every one in between.
  await page.waitFor(
    `document.querySelector('.camera-settings .setting-value').textContent.trim() === '3.3 ms'`,
    {what: "the dragged exposure"},
  );
  const deadline = Date.now() + 3000;
  while (robot.sentOn("/camera_settings").at(-1)?.exposure_100us !== 33) {
    assert.ok(Date.now() < deadline, JSON.stringify(robot.sent));
    await new Promise((resolve) => setTimeout(resolve, 50));
  }
  assert.ok(robot.sentOn("/camera_settings").length < 5);
  for (const message of robot.sentOn("/camera_settings")) {
    assert.deepEqual(Object.keys(message), ["exposure_100us"]);
  }
  // As the adjuster would, the robot has sent it on to the cameras, and the
  // page keeps showing it.
  await new Promise((resolve) => setTimeout(resolve, 3000));
  assert.equal(await page.text(".camera-settings .setting-value"), "3.3 ms");
});

test("auto exposure sends 0, and leaving it goes back to a manual one", async () => {
  await tab("Settings");
  await page.waitFor(`document.querySelector('.setting-auto input')`);
  assert.equal(
    await page.evaluate(`document.querySelector('#exposure').disabled`),
    false,
  );
  await page.click(".setting-auto input");
  assert.deepEqual(await sends("/camera_settings", 1), [{exposure_100us: 0}]);
  await page.waitFor(`document.querySelector('#exposure').disabled`, {
    what: "the slider to disable",
  });
  assert.equal(await page.text(".camera-settings .setting-value"), "auto");

  await page.click(".setting-auto input");
  const sent = await sends("/camera_settings", 2);
  // The constants' exposure, the last manual one there was.
  assert.deepEqual(sent[1], {exposure_100us: 60});
});

test("a gain sends only the gain; a bad one sends nothing", async () => {
  await tab("Settings");
  await page.waitFor(`document.querySelector('#gain')`);
  await page.setInput("#gain", "2", "change");
  assert.deepEqual(await sends("/camera_settings", 1), [{gain: 2}]);
  await page.setInput("#gain", "-1", "change");
  await page.setInput("#gain", "1.5", "change");
  await new Promise((resolve) => setTimeout(resolve, 500));
  assert.equal(robot.sentOn("/camera_settings").length, 1);
});

test("a refused change says why", async () => {
  robot.failNextSend = "The event loop is gone";
  await tab("Settings");
  await page.waitFor(`document.querySelector('#gain')`);
  await page.setInput("#gain", "1", "change");
  assert.equal(
    await page.waitFor(
      `document.querySelector('.camera-settings .error')?.textContent.trim()`,
      {
        what: "the error",
      },
    ),
    "The event loop is gone",
  );
});

/** calibration_server's Status, as when collecting `captures`. */
function collecting(sessionId, captures, sequence) {
  return {
    session_id: sessionId,
    state: "COLLECTING",
    camera: 2,
    verdict: "NO_BOARD",
    corners: 0,
    min_corners: 15,
    captures,
    target_captures: 70,
    session_sequence: sequence,
  };
}

test("calibrating: start, see captures, and download their images", async () => {
  robot.setLatest("/calibration", "frc.vision.calibration.Status", {
    state: "IDLE",
    camera: -1,
  });
  await tab("Calibrate");
  await page.waitFor(
    `[...document.querySelectorAll('.calibrate-start option')].length === 2`,
    {what: "the cameras to pick from"},
  );
  // Pick camera 2, and 40 captures.
  await page.evaluate(`(() => {
    const select = document.querySelector('.calibrate-start select');
    select.selectedIndex = 1;
    select.dispatchEvent(new Event('change'));
  })()`);
  await page.setInput(".calibrate-start input[type=number]", "40");
  await page.clickButton("Start");
  const [start] = await sends("/calibration", 1);
  assert.equal(start.action, "START");
  assert.equal(start.camera, 2);
  assert.equal(start.target_captures, 40);
  assert.match(start.session_id, /\S/);

  // calibration_server starts, and has three captures.
  const id = start.session_id;
  robot.setLatest("/calibration", "frc.vision.calibration.Status", {
    ...collecting(id, 3, 1),
    target_captures: 40,
  });
  robot.setLatest("/calibration", "frc.vision.calibration.Session", {
    session_id: id,
    state: "COLLECTING",
    camera: 2,
    image_width: 1600,
    image_height: 1304,
    board: BOARD,
    boards: [BOARD],
    target_captures: 40,
    // Each capture's corners, where it saw them, and its pose.
    captures: [0, 1, 2].map((i) => ({
      monotonic_timestamp_ns: i * 1e9,
      corners: [0, 1, 2, 3].map((id) => ({
        id,
        x: 400 + 300 * i + 40 * id,
        y: 600 + 10 * id,
      })),
      rvec: [0.1 * i, 0, 0],
      tvec: [0, 0, 1 + 0.5 * i],
    })),
  });
  await page.waitFor(
    `document.querySelector('.calibrate-live h2')?.textContent.trim() === 'Calibrating camera 2'`,
    {what: "the live session"},
  );
  assert.match(
    await page.evaluate(`document.body.innerText`),
    /3 of 40 captures/,
  );

  // Export, and the robot's zip shows as a link.
  await page.clickButton("Export the images");
  const exported = await sends("/calibration", 2);
  assert.equal(exported[1].action, "EXPORT_IMAGES");
  assert.equal(exported[1].session_id, id);
  robot.setLatest("/calibration", "frc.vision.calibration.Status", {
    ...collecting(id, 3, 2),
    target_captures: 40,
  });
  robot.latest.get(
    "/calibration frc.vision.calibration.Session",
  ).images_download_path = `/calibration/camera2-${id}.zip`;
  robot.latest.get(
    "/calibration frc.vision.calibration.Session",
  ).images_exported = 3;
  const link = await page.waitFor(
    `(() => {
      const a = [...document.querySelectorAll('a[download]')]
        .find((a) => /Download 3 images/.test(a.textContent));
      return a ? a.getAttribute('href') : null;
    })()`,
    {what: "the download link"},
  );
  assert.equal(link, `/calibration/camera2-${id}.zip`);
  await screenshot("calibrating");

  // Other tabs say a session is running, and offer to cancel it.
  await tab("Feeds");
  await page.waitFor(`document.querySelector('.banner')`, {what: "the banner"});
  await page.clickButton("Cancel");
  const cancelled = await sends("/calibration", 3);
  assert.equal(cancelled[2].action, "CANCEL");
});

test("the page throws nothing", () => {
  // web_proxy's refused socket and the missing video are expected noise.
  const unexpected = browser.errors.filter(
    (e) => !/WebSocket|wss:|\/ws|net::ERR|404|Failed to load resource/.test(e),
  );
  assert.deepEqual(unexpected, []);
});
