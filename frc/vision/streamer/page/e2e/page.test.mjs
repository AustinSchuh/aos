// End to end tests of the vision page: the built page, in headless Chrome,
// against the robot simulated (//frc/vision/sim:vision_robot_sim), which runs
// robot_server, web_proxy, calibration_server and the exposure adjuster as on
// the robot, with fake cameras and a fake starter.
//
// camera_streamer can't run on x86 until the amd64 sysroot has its GStreamer
// plugins (frc/amd64/build_rootfs.py), so a stand-in on its port tells the page
// which cameras there are, through robot_server's /cameras/ws as on the robot;
// there's no video.
//
// The robot keeps its state across the tests, as a robot does, so they run in
// order.  Each saves screenshots, and the robot's log, to the test's
// undeclared outputs.

import {strict as assert} from "node:assert";
import {spawn} from "node:child_process";
import crypto from "node:crypto";
import fs from "node:fs";
import http from "node:http";
import net from "node:net";
import os from "node:os";
import path from "node:path";
import {after, before, beforeEach, test} from "node:test";

import {Browser} from "./browser.mjs";

const RUNFILES = process.env.JS_BINARY__RUNFILES;
const runfile = (rlocation) => path.join(RUNFILES, rlocation);
const OUTPUTS = process.env.TEST_UNDECLARED_OUTPUTS_DIR;
const CAMERAS = [0, 2];

// In the page: the Starter tab's rows, {name: state}, or null before any.
const STARTER_STATES = `(() => {
  const doc = document.querySelector('main.embed iframe')?.contentDocument;
  if (!doc) return null;
  const states = {};
  for (const row of doc.querySelectorAll('#status_list tr')) {
    const cells = row.querySelectorAll('td');
    states[cells[0].textContent.trim()] = cells[2].textContent.trim();
  }
  return Object.keys(states).length > 0 ? states : null;
})()`;

let robot;
let robotLog = "";
let cameraStreamer;
let browser;
let page;
let origin;
let stateDir;

/** A port nothing is listening on. */
async function freePort() {
  const server = net.createServer();
  await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
  const {port} = server.address();
  await new Promise((resolve) => server.close(resolve));
  return port;
}

/** Copies every file in `from` (a runfiles directory) into `to`. */
function copyFiles(from, to) {
  for (const name of fs.readdirSync(from)) {
    const file = path.join(from, name);
    if (fs.statSync(file).isFile()) {
      fs.copyFileSync(file, path.join(to, name));
    }
  }
}

/**
 * camera_streamer's signalling, as far as saying which cameras it has: what the
 * page needs before it asks for video.
 */
function startCameraStreamerStandIn(port) {
  const server = http.createServer((request, response) => {
    response.writeHead(404).end();
  });
  server.on("upgrade", (request, socket) => {
    const accept = crypto
      .createHash("sha1")
      .update(
        request.headers["sec-websocket-key"] +
          "258EAFA5-E914-47DA-95CA-C5AB0DC85B11",
      )
      .digest("base64");
    socket.write(
      "HTTP/1.1 101 Switching Protocols\r\n" +
        "Upgrade: websocket\r\nConnection: Upgrade\r\n" +
        `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
    );
    const payload = Buffer.from(
      JSON.stringify({type: "cameras", cameras: CAMERAS}),
    );
    socket.write(Buffer.concat([Buffer.from([0x81, payload.length]), payload]));
    socket.on("data", () => {});
    socket.on("error", () => {});
  });
  return new Promise((resolve) =>
    server.listen(port, "127.0.0.1", () => resolve(server)),
  );
}

before(async () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), "vision-robot-"));
  const www = path.join(root, "www");
  stateDir = path.join(root, "state");
  const tls = path.join(root, "tls");
  for (const dir of [www, stateDir, tls]) {
    fs.mkdirSync(dir, {recursive: true});
  }
  // The pages as deployed: the vision page, and the robot's own.
  fs.cpSync(runfile(process.env.VISION_DIR), path.join(www, "vision"), {
    recursive: true,
    dereference: true,
  });
  copyFiles(path.dirname(runfile(process.env.STARTER_HTML)), www);
  copyFiles(path.dirname(runfile(process.env.FIELD_HTML)), www);
  fs.copyFileSync(
    runfile(process.env.TEST_CERT),
    path.join(tls, "server-chain.crt"),
  );
  fs.copyFileSync(
    runfile(process.env.TEST_CERT).replace(/\.crt$/, ".key"),
    path.join(tls, "server.key"),
  );

  const [httpsPort, httpPort, proxyPort, streamerPort] = await Promise.all([
    freePort(),
    freePort(),
    freePort(),
    freePort(),
  ]);
  cameraStreamer = await startCameraStreamerStandIn(streamerPort);
  robot = spawn(
    runfile(process.env.ROBOT_SIM),
    [
      `--config=${runfile(process.env.ROBOT_CONFIG)}`,
      `--constants=${runfile(process.env.ROBOT_CONSTANTS)}`,
      `--data_dir=${www}`,
      `--state_dir=${stateDir}`,
      `--tls_dir=${tls}`,
      `--https_port=${httpsPort}`,
      `--http_port=${httpPort}`,
      `--proxy_port=${proxyPort}`,
      `--camera_streamer_port=${streamerPort}`,
      `--cameras=${CAMERAS.join(",")}`,
    ],
    {stdio: ["ignore", "pipe", "pipe"]},
  );
  robot.stderr.on("data", (chunk) => (robotLog += chunk));
  await new Promise((resolve, reject) => {
    const timeout = setTimeout(
      () => reject(new Error(`The robot didn't start:\n${robotLog}`)),
      60000,
    );
    robot.stdout.on("data", (chunk) => {
      if (/Serving/.test(chunk)) {
        clearTimeout(timeout);
        resolve();
      }
    });
    robot.on("exit", (code) => {
      clearTimeout(timeout);
      reject(new Error(`The robot exited with ${code}:\n${robotLog}`));
    });
  });
  origin = `https://localhost:${httpsPort}`;

  const sysroot = path.resolve(
    path.dirname(runfile(process.env.SYSROOT_FILE)),
    "../..",
  );
  browser = await Browser.launch(runfile(process.env.CHROME), sysroot);
});

after(async () => {
  await browser?.close();
  robot?.kill("SIGINT");
  cameraStreamer?.close();
  if (OUTPUTS) {
    fs.writeFileSync(path.join(OUTPUTS, "robot.log"), robotLog);
    fs.writeFileSync(
      path.join(OUTPUTS, "interfaces.json"),
      JSON.stringify(os.networkInterfaces(), null, 1),
    );
  }
});

beforeEach(async () => {
  await page?.close();
  page = await browser.open(`${origin}/vision/`);
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

function tab(label) {
  return page.clickButton(label);
}

/** The exposure and gain the robot saved, or null before any. */
function savedSettings() {
  // Read, not checked for first: the robot may remove it in between.
  let text;
  try {
    text = fs.readFileSync(path.join(stateDir, "camera_settings.json"), "utf8");
  } catch (error) {
    if (error.code === "ENOENT") {
      return null;
    }
    throw error;
  }
  return JSON.parse(text);
}

/** Waits until `predicate` holds of what the robot saved. */
async function waitForSaved(predicate, what) {
  const deadline = Date.now() + 10000;
  while (!predicate(savedSettings())) {
    assert.ok(
      Date.now() < deadline,
      `Timed out waiting for ${what}; saved: ${JSON.stringify(savedSettings())}`,
    );
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
}

test("the feeds show a tile for each camera", async () => {
  const names = await page.waitFor(
    `(() => {
      const names = [...document.querySelectorAll('.tile-name')]
        .map((e) => e.textContent.trim());
      return names.length === 2 ? names : null;
    })()`,
    {what: "two camera tiles"},
  );
  assert.deepEqual(names, ["Camera 0", "Camera 2"]);
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
    assert.equal(await page.text("nav button.active"), label);
    assert.equal(
      await page.evaluate(`document.querySelectorAll('main').length`),
      1,
      `only the ${label} tab's view shows`,
    );
  }
});

test("the starter tab shows starter's applications, through web_proxy", async () => {
  await tab("Starter");
  const states = await page.waitFor(STARTER_STATES, {
    what: "starter's status in its page",
    timeoutMs: 20000,
  });
  assert.equal(states["robot_server"], "RUNNING");
  assert.equal(states["apriltag_detector0"], "RUNNING");
  assert.equal(states["calibration_server"], "STOPPED");
  assert.equal(
    await page.evaluate(`getComputedStyle(document.querySelector('main.embed iframe')
      .contentDocument.querySelector('table')).tableLayout`),
    "fixed",
  );
  await screenshot("starter");
});

test("settings show the robot's constants until something is set", async () => {
  assert.equal(savedSettings(), null);
  await tab("Settings");
  // The test robot's constants: 6 ms, gain 0, once they've come ("–" until then).
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
  // A card for each of its calibrations.
  assert.equal(
    await page.waitFor(`document.querySelectorAll('intrinsics-card').length`),
    4,
  );
  await screenshot("settings");
});

test("the exposure dragged is saved on the robot and sent to the cameras", async () => {
  await tab("Settings");
  await page.waitFor(`document.querySelector('#exposure')`);
  for (const position of [400, 450, 500, 550, 600]) {
    await page.setInput("#exposure", position);
  }
  await waitForSaved((s) => s?.exposure_100us === 33, "the exposure");

  // A new page reads it back from the cameras' channel.
  await page.close();
  page = await browser.open(`${origin}/vision/`);
  await page.waitFor(`document.querySelector('vision-app nav')`);
  await tab("Settings");
  await page.waitFor(
    `document.querySelector('.camera-settings .setting-value')?.textContent.trim() === '3.3 ms'`,
    {what: "the saved exposure"},
  );
});

test("auto exposure is saved as 0, and the gain alongside", async () => {
  await tab("Settings");
  await page.waitFor(
    `document.querySelector('.camera-settings .setting-value')?.textContent.trim() === '3.3 ms'`,
    {what: "the saved exposure"},
  );
  await page.click(".setting-auto input");
  await waitForSaved((s) => s?.exposure_100us === 0, "auto exposure");
  await page.setInput("#gain", "2", "change");
  await waitForSaved(
    (s) => s?.exposure_100us === 0 && s?.gain === 2,
    "the gain, keeping auto",
  );
  // Leaving auto goes back to the last manual exposure this page set, or,
  // on a page that set none, the constants' 6 ms.
  await page.click(".setting-auto input");
  await waitForSaved((s) => s?.exposure_100us === 60, "back to manual");
});

test("the robot's own settings download to check in, or go back to constants.json's", async () => {
  await tab("Settings");
  // What the last test left: its own exposure and gain.
  await page.waitFor(`document.querySelector(".camera-settings .own a")`, {
    what: "the offer to check them in",
  });
  const offered = await page.evaluate(`(async () => {
    const href = document.querySelector(".camera-settings .own a").getAttribute("href");
    return await (await fetch(href)).json();
  })()`);
  // Whole, as constants.jinja2.json includes it: the test robot's image
  // size, with the exposure and gain set.
  assert.equal(offered.image_width, 1456);
  assert.equal(offered.image_height, 1088);
  assert.equal(offered.exposure_100us, 60);
  assert.equal(offered.gain, 2);
  await screenshot("own-settings");

  await page.clickButton("use constants.json's");
  await waitForSaved((s) => s === null, "the robot to forget them");
  await page.waitFor(
    `!document.querySelector(".camera-settings .own") &&
     document.querySelector("#gain").value === "0"`,
    {what: "constants.json's settings"},
  );
});

test("calibrating: start, captures, export, download, cancel", async () => {
  await tab("Calibrate");
  await page.waitFor(
    `[...document.querySelectorAll('.calibrate-start option')].length === 2`,
    {what: "the cameras to pick from"},
  );
  await page.evaluate(`(() => {
    const select = document.querySelector('.calibrate-start select');
    select.selectedIndex = 1;
    select.dispatchEvent(new Event('change'));
  })()`);
  await page.setInput(".calibrate-start input[type=number]", "10");
  await page.clickButton("Start");

  // robot_server stops the detectors, starts calibration_server, and it
  // captures the fake camera's board as it moves.
  await page.waitFor(
    `document.querySelector('.calibrate-live h2')?.textContent.trim() === 'Calibrating camera 2'`,
    {what: "the session", timeoutMs: 20000},
  );
  await page.waitFor(
    `(() => {
      const m = /(\\d+) of 10 captures/.exec(document.body.innerText);
      return m && Number(m[1]) >= 3;
    })()`,
    {what: "three captures", timeoutMs: 30000},
  );
  await screenshot("calibrating");

  await page.clickButton("Export the images");
  const href = await page.waitFor(
    `(() => {
      const a = [...document.querySelectorAll('a[download]')]
        .find((a) => /Download \\d+ images/.test(a.textContent));
      return a ? a.getAttribute('href') : null;
    })()`,
    {what: "the download link", timeoutMs: 15000},
  );
  // The zip, from robot_server, as the browser downloads it: its directory
  // lists a PNG for each capture exported, and the note.
  const zip = await page.evaluate(`(async () => {
    const response = await fetch(${JSON.stringify(href)});
    const bytes = new Uint8Array(await response.arrayBuffer());
    const view = new DataView(bytes.buffer);
    const end = bytes.length - 22;
    return {
      status: response.status,
      magic: String.fromCharCode(bytes[0], bytes[1]),
      entries: view.getUint16(end + 10, true),
    };
  })()`);
  assert.equal(zip.status, 200);
  assert.equal(zip.magic, "PK");
  const exported = Number(
    /Download (\d+) images/.exec(
      await page.evaluate(`document.body.innerText`),
    )[1],
  );
  assert.ok(exported >= 3);
  assert.equal(zip.entries, exported + 1);

  // While it runs, starter says so.
  await tab("Starter");
  await page.waitFor(`${STARTER_STATES}?.calibration_server === 'RUNNING'`, {
    what: "calibration_server running",
    timeoutMs: 20000,
  });
  assert.equal(
    (await page.evaluate(STARTER_STATES))["apriltag_detector0"],
    "STOPPED",
  );

  // Cancelling from the banner puts the detectors back.
  await page.waitFor(`document.querySelector('.banner')`, {what: "the banner"});
  await page.clickButton("Cancel");
  await page.waitFor(
    `(() => {
      const s = ${STARTER_STATES};
      return s?.calibration_server === 'STOPPED' && s?.apriltag_detector0 === 'RUNNING';
    })()`,
    {what: "the detectors back", timeoutMs: 20000},
  );
  await screenshot("cancelled");
});

test("the page throws nothing", () => {
  // Video can't start without camera_streamer.
  const unexpected = browser.errors.filter(
    (e) => !/WebSocket|wss:|net::ERR|Failed to load resource/.test(e),
  );
  assert.deepEqual(unexpected, []);
});
