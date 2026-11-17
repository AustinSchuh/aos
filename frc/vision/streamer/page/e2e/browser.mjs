// Drives headless Chrome over the DevTools protocol, with Node's own
// WebSocket: enough to load a page, run script in it, and wait for what it
// shows, without Cypress or Puppeteer.

import {spawn} from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

export class Browser {
  /**
   * Starts Chrome at `chromePath`, headless.  `sysroot`, when given, is a
   * Debian root to take its libraries and fonts from, for machines without.
   */
  static async launch(chromePath, sysroot = null) {
    const profile = fs.mkdtempSync(path.join(os.tmpdir(), "chrome-"));
    const env = {...process.env};
    if (sysroot) {
      env.LD_LIBRARY_PATH = [
        path.join(sysroot, "usr/lib/x86_64-linux-gnu"),
        path.join(sysroot, "lib/x86_64-linux-gnu"),
        process.env.LD_LIBRARY_PATH,
      ]
        .filter(Boolean)
        .join(":");
      const fontsConf = path.join(profile, "fonts.conf");
      fs.writeFileSync(
        fontsConf,
        `<?xml version="1.0"?>
<!DOCTYPE fontconfig SYSTEM "fonts.dtd">
<fontconfig>
  <dir>${path.join(sysroot, "usr/share/fonts")}</dir>
  <cachedir>${path.join(profile, "fontconfig")}</cachedir>
</fontconfig>
`,
      );
      env.FONTCONFIG_FILE = fontsConf;
    }
    const chrome = spawn(
      chromePath,
      [
        "--headless=new",
        "--no-sandbox",
        "--disable-gpu",
        "--disable-dev-shm-usage",
        "--no-first-run",
        "--no-default-browser-check",
        // The fake robot's certificate is self-signed.
        "--ignore-certificate-errors",
        "--remote-debugging-port=0",
        `--user-data-dir=${profile}`,
        "--window-size=1280,900",
        "about:blank",
      ],
      {stdio: ["ignore", "ignore", "pipe"], env},
    );
    let log = "";
    const url = await new Promise((resolve, reject) => {
      const timeout = setTimeout(
        () => reject(new Error(`Chrome didn't start:\n${log}`)),
        30000,
      );
      chrome.stderr.on("data", (chunk) => {
        log += chunk;
        const match = /DevTools listening on (ws:\/\/\S+)/.exec(log);
        if (match) {
          clearTimeout(timeout);
          resolve(match[1]);
        }
      });
      chrome.on("exit", (code) => {
        clearTimeout(timeout);
        reject(new Error(`Chrome exited with ${code}:\n${log}`));
      });
    });
    const browser = new Browser(chrome, profile);
    await browser.connect(url);
    return browser;
  }

  constructor(chrome, profile) {
    this.chrome = chrome;
    this.profile = profile;
    this.nextId = 1;
    this.pending = new Map();
    /** Console messages and uncaught exceptions, from every page. */
    this.errors = [];
  }

  async connect(url) {
    this.socket = new WebSocket(url);
    await new Promise((resolve, reject) => {
      this.socket.onopen = resolve;
      this.socket.onerror = reject;
    });
    this.socket.onmessage = (event) => {
      const message = JSON.parse(event.data);
      if (message.id !== undefined) {
        const {resolve, reject} = this.pending.get(message.id);
        this.pending.delete(message.id);
        if (message.error) {
          reject(new Error(message.error.message));
        } else {
          resolve(message.result);
        }
      } else if (message.method === "Runtime.exceptionThrown") {
        this.errors.push(
          message.params.exceptionDetails.exception?.description ??
            message.params.exceptionDetails.text,
        );
      } else if (
        message.method === "Runtime.consoleAPICalled" &&
        message.params.type === "error"
      ) {
        this.errors.push(
          message.params.args.map((a) => a.value ?? a.description).join(" "),
        );
      }
    };
  }

  send(method, params = {}, sessionId = undefined) {
    const id = this.nextId++;
    this.socket.send(JSON.stringify({id, method, params, sessionId}));
    return new Promise((resolve, reject) =>
      this.pending.set(id, {resolve, reject}),
    );
  }

  /** A new tab at `url`, loaded. */
  async open(url) {
    const {targetId} = await this.send("Target.createTarget", {
      url: "about:blank",
    });
    const {sessionId} = await this.send("Target.attachToTarget", {
      targetId,
      flatten: true,
    });
    const page = new Page(this, sessionId, targetId);
    await this.send("Runtime.enable", {}, sessionId);
    await this.send("Page.enable", {}, sessionId);
    await this.send("Page.navigate", {url}, sessionId);
    return page;
  }

  async close() {
    try {
      await this.send("Browser.close");
    } catch {
      // It's going anyway.
    }
    this.socket.close();
    await new Promise((resolve) => {
      if (this.chrome.exitCode !== null) {
        resolve();
      } else {
        this.chrome.on("exit", resolve);
        setTimeout(() => this.chrome.kill("SIGKILL"), 5000);
      }
    });
    fs.rmSync(this.profile, {recursive: true, force: true});
  }
}

export class Page {
  constructor(browser, sessionId, targetId) {
    this.browser = browser;
    this.sessionId = sessionId;
    this.targetId = targetId;
  }

  /** `expression`'s value in the page, awaited if it's a promise. */
  async evaluate(expression) {
    const result = await this.browser.send(
      "Runtime.evaluate",
      {expression, awaitPromise: true, returnByValue: true},
      this.sessionId,
    );
    if (result.exceptionDetails) {
      throw new Error(
        `${expression}: ${result.exceptionDetails.exception?.description ?? result.exceptionDetails.text}`,
      );
    }
    return result.result.value;
  }

  /** Waits until `expression` is truthy in the page, and returns it. */
  async waitFor(expression, {timeoutMs = 10000, what = expression} = {}) {
    const deadline = Date.now() + timeoutMs;
    let last;
    for (;;) {
      try {
        // An element found is true: elements don't serialize.
        last = await this.evaluate(
          `(() => { const v = (${expression}); return v instanceof Node ? true : v; })()`,
        );
        if (last) {
          return last;
        }
      } catch (error) {
        last = error.message;
      }
      if (Date.now() > deadline) {
        throw new Error(`Timed out waiting for ${what}; last: ${last}`);
      }
      await new Promise((resolve) => setTimeout(resolve, 100));
    }
  }

  /** The text of the first element matching `selector`, or null. */
  text(selector) {
    return this.evaluate(
      `document.querySelector(${JSON.stringify(selector)})?.textContent?.trim() ?? null`,
    );
  }

  /** Clicks the button whose text is `label`. */
  clickButton(label) {
    return this.evaluate(`(() => {
      const button = [...document.querySelectorAll('button')]
        .find((b) => b.textContent.trim() === ${JSON.stringify(label)});
      if (!button) throw new Error('No button ' + ${JSON.stringify(label)});
      button.click();
      return true;
    })()`);
  }

  /** Sets an input's value and fires `event` on it, as typing or dragging does. */
  setInput(selector, value, event = "input") {
    return this.evaluate(`(() => {
      const input = document.querySelector(${JSON.stringify(selector)});
      if (!input) throw new Error('No input ' + ${JSON.stringify(selector)});
      input.value = ${JSON.stringify(String(value))};
      input.dispatchEvent(new Event(${JSON.stringify(event)}, {bubbles: true}));
      return true;
    })()`);
  }

  /** Clicks a checkbox, as a person does. */
  click(selector) {
    return this.evaluate(`(() => {
      const element = document.querySelector(${JSON.stringify(selector)});
      if (!element) throw new Error('No element ' + ${JSON.stringify(selector)});
      element.click();
      return true;
    })()`);
  }

  /** A PNG of the page, as a Buffer. */
  async screenshot() {
    const {data} = await this.browser.send(
      "Page.captureScreenshot",
      {format: "png"},
      this.sessionId,
    );
    return Buffer.from(data, "base64");
  }

  close() {
    return this.browser.send("Target.closeTarget", {targetId: this.targetId});
  }
}
