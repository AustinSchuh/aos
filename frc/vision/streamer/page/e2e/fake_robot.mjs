// A fake robot for the vision page's end-to-end tests: robot_server's HTTPS
// front door, serving the page and the robot's other pages as deployed, its
// /api for channels' latest messages and sends, and camera_streamer's
// /cameras/ws saying which cameras there are.  There's no video: tiles show
// "No signal".  web_proxy's /ws is refused, as when it's down.
//
// What the page sends lands in `sent`; what it reads is `latest`, by
// "name type", which tests set.  Sends with an effect on the robot, a camera
// setting or a calibration command, update `latest` as the robot would.

import crypto from "node:crypto";
import fs from "node:fs";
import https from "node:https";
import path from "node:path";

const TYPES = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript",
  ".css": "text/css",
  ".map": "application/json",
  ".png": "image/png",
};

export class FakeRobot {
  /**
   * @param {object} options
   * @param {string} options.visionDir the built page, served at /vision/.
   * @param {string} options.rootDir the robot's other pages, served at /.
   * @param {string} options.certDir holds localhost.key and localhost.crt.
   * @param {number[]} options.cameras the cameras camera_streamer has.
   */
  constructor({visionDir, rootDir, certDir, cameras}) {
    this.visionDir = visionDir;
    this.rootDir = rootDir;
    this.cameras = cameras;
    /** Every POST /api/send: {name, type, message}. */
    this.sent = [];
    /** Each channel's latest message, by `${name} ${type}`. */
    this.latest = new Map();
    /** Channels the page may send on, as robot_server's --sendable-channels. */
    this.sendable = new Set([
      "/camera_settings frc.vision.CameraStreamSettings",
      "/calibration frc.vision.calibration.Command",
    ]);
    /** Answered as robot_server's error for the next send, when set. */
    this.failNextSend = null;
    this.server = https.createServer(
      {
        key: fs.readFileSync(path.join(certDir, "localhost.key")),
        cert: fs.readFileSync(path.join(certDir, "localhost.crt")),
      },
      (request, response) => this.handle(request, response),
    );
    this.server.on("upgrade", (request, socket) =>
      this.upgrade(request, socket),
    );
    this.sockets = new Set();
  }

  async start() {
    await new Promise((resolve) => this.server.listen(0, "127.0.0.1", resolve));
    this.origin = `https://localhost:${this.server.address().port}`;
    return this.origin;
  }

  async stop() {
    for (const socket of this.sockets) {
      socket.destroy();
    }
    await new Promise((resolve) => this.server.close(resolve));
  }

  setLatest(name, type, message) {
    this.latest.set(`${name} ${type}`, message);
  }

  /** The sends on `name`, oldest first. */
  sentOn(name) {
    return this.sent.filter((s) => s.name === name).map((s) => s.message);
  }

  handle(request, response) {
    const url = new URL(request.url, "https://localhost");
    if (url.pathname === "/api/latest") {
      return this.apiLatest(url, response);
    }
    if (url.pathname === "/api/send" && request.method === "POST") {
      let body = "";
      request.on("data", (chunk) => (body += chunk));
      request.on("end", () => this.apiSend(url, body, response));
      return;
    }
    let file;
    if (url.pathname === "/vision" || url.pathname === "/vision/") {
      file = path.join(this.visionDir, "index.html");
    } else if (url.pathname.startsWith("/vision/")) {
      file = path.join(this.visionDir, url.pathname.slice("/vision/".length));
    } else {
      file = path.join(this.rootDir, url.pathname);
    }
    if (!fs.existsSync(file) || !fs.statSync(file).isFile()) {
      response.writeHead(404).end("Not found");
      return;
    }
    response.writeHead(200, {
      "Content-Type": TYPES[path.extname(file)] ?? "application/octet-stream",
    });
    fs.createReadStream(file).pipe(response);
  }

  json(response, status, body) {
    response.writeHead(status, {"Content-Type": "application/json"});
    response.end(JSON.stringify(body));
  }

  apiLatest(url, response) {
    const name = url.searchParams.get("name");
    const type = url.searchParams.get("type");
    const message = this.latest.get(`${name} ${type}`);
    if (message === undefined) {
      return this.json(response, 404, {error: `Nothing on ${name} ${type}`});
    }
    // A new message every time, as a status sent periodically is.
    const now = Number(process.hrtime.bigint());
    return this.json(response, 200, {
      name,
      type,
      monotonic_event_time_ns: now,
      realtime_event_time_ns: Date.now() * 1e6,
      message,
    });
  }

  apiSend(url, body, response) {
    const name = url.searchParams.get("name");
    const type = url.searchParams.get("type");
    if (!this.sendable.has(`${name} ${type}`)) {
      return this.json(response, 403, {
        error: `robot_server may not send on ${name} ${type}`,
      });
    }
    let message;
    try {
      message = JSON.parse(body);
    } catch {
      return this.json(response, 400, {error: "Not JSON"});
    }
    if (this.failNextSend !== null) {
      const error = this.failNextSend;
      this.failNextSend = null;
      return this.json(response, 500, {error});
    }
    this.sent.push({name, type, message});
    if (name === "/camera_settings") {
      this.applyCameraSettings(message);
    }
    return this.json(response, 200, {sent: true});
  }

  /** As field_side_exposure_adjuster: what's set goes to every camera. */
  applyCameraSettings(message) {
    for (const camera of this.cameras) {
      const key = `/camera${camera} frc.vision.CameraStreamSettings`;
      this.latest.set(key, {...(this.latest.get(key) ?? {}), ...message});
    }
  }

  upgrade(request, socket) {
    const url = new URL(request.url, "https://localhost");
    if (url.pathname !== "/cameras/ws") {
      socket.destroy();
      return;
    }
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
    this.sockets.add(socket);
    socket.on("close", () => this.sockets.delete(socket));
    socket.on("error", () => this.sockets.delete(socket));
    // What the page says back (watch, answers) isn't needed: no video.
    socket.on("data", () => {});
    sendText(socket, JSON.stringify({type: "cameras", cameras: this.cameras}));
  }
}

/** One unmasked text frame, as a server sends. */
function sendText(socket, text) {
  const payload = Buffer.from(text);
  let header;
  if (payload.length < 126) {
    header = Buffer.from([0x81, payload.length]);
  } else {
    header = Buffer.alloc(4);
    header[0] = 0x81;
    header[1] = 126;
    header.writeUInt16BE(payload.length, 2);
  }
  socket.write(Buffer.concat([header, payload]));
}
