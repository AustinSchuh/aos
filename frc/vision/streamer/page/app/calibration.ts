import {Injectable, NgZone, OnDestroy} from '@angular/core';

import {
  Command,
  newSessionId,
  parseSession,
  parseStatus,
  Session,
  SessionStatus,
} from '../lib/calibration.js';

const COMMANDS = {
  name: '/calibration',
  type: 'frc.vision.calibration.Command',
};

/** How long without a new Status before calibration_server counts as stopped. */
const ALIVE_MS = 2500;

/**
 * The robot's calibration session: calibration_server's Status, polled with /api/latest (5 Hz
 * while the Calibrate tab shows, 1 Hz otherwise, for the banner), and its Session, fetched when
 * Status says there's a new one.  Commands go through robot_server's /api/send, which starts and
 * stops calibration_server and the AprilTag detectors around a session.
 */
@Injectable({providedIn: 'root'})
export class CalibrationSession implements OnDestroy {
  status: SessionStatus | null = null;
  session: Session | null = null;
  /** The last command's error, from robot_server or calibration_server. */
  error = '';
  /** A command is on its way. */
  busy = false;

  private active = false;
  private timer: ReturnType<typeof setTimeout> | null = null;
  private lastStatusNs = -1;
  private lastChangeMs = 0;
  private fetchedSequence = -1;
  private destroyed = false;
  private polling = false;

  constructor(private readonly zone: NgZone) {
    void this.poll();
  }

  ngOnDestroy(): void {
    this.destroyed = true;
    if (this.timer !== null) {
      clearTimeout(this.timer);
    }
  }

  /** Whether the Calibrate tab is showing: poll faster. */
  setActive(active: boolean): void {
    if (active !== this.active) {
      this.active = active;
      if (active) {
        if (this.timer !== null) {
          clearTimeout(this.timer);
        }
        void this.poll();
      }
    }
  }

  /** calibration_server is running, and saying so. */
  get alive(): boolean {
    return this.status !== null && Date.now() - this.lastChangeMs < ALIVE_MS;
  }

  /** A session is running: the AprilTag detectors are off. */
  get running(): boolean {
    return this.alive && this.status?.state !== 'IDLE';
  }

  private async poll(): Promise<void> {
    if (this.destroyed || this.polling) {
      return;
    }
    this.polling = true;
    if (this.timer !== null) {
      clearTimeout(this.timer);
      this.timer = null;
    }
    try {
      const status = await latest(
        '/calibration',
        'frc.vision.calibration.Status',
        0,
      );
      if (status !== null) {
        const parsed = parseStatus(status.message);
        if (status.monotonic_event_time_ns !== this.lastStatusNs) {
          this.lastStatusNs = status.monotonic_event_time_ns;
          this.lastChangeMs = Date.now();
        }
        this.zone.run(() => (this.status = parsed));
        if (
          parsed !== null &&
          this.alive &&
          parsed.sessionSequence !== this.fetchedSequence
        ) {
          const session = await latest(
            '/calibration',
            'frc.vision.calibration.Session',
            100000,
          );
          if (session !== null) {
            this.fetchedSequence = parsed.sessionSequence;
            this.zone.run(() => (this.session = parseSession(session.message)));
          }
        }
      }
    } finally {
      this.polling = false;
      if (!this.destroyed) {
        this.timer = setTimeout(
          () => void this.poll(),
          this.active || this.running ? 200 : 1000,
        );
      }
    }
  }

  /** Starts a session on `camera`, stopping the AprilTag detectors. */
  start(camera: number, targetCaptures?: number): Promise<boolean> {
    // A new session's Session must be fetched, whatever its sequence.
    this.fetchedSequence = -1;
    return this.send({
      action: 'START',
      session_id: newSessionId(),
      camera,
      ...(targetCaptures ? {target_captures: targetCaptures} : {}),
    });
  }

  /** Sends `command` to the running session. */
  send(command: Command): Promise<boolean> {
    const full: Command = {
      session_id: this.status?.sessionId ?? '',
      camera: this.status?.camera ?? 0,
      ...command,
    };
    return this.post(full);
  }

  private async post(command: Command): Promise<boolean> {
    this.busy = true;
    this.error = '';
    const query = new URLSearchParams(COMMANDS);
    try {
      const response = await fetch(`/api/send?${query}`, {
        method: 'POST',
        body: JSON.stringify(command),
      });
      if (!response.ok) {
        const body = (await response.json().catch(() => ({}))) as {
          error?: string;
        };
        this.zone.run(
          () => (this.error = body.error ?? `HTTP ${response.status}`),
        );
        return false;
      }
      // What calibration_server made of it shows in the next Status.
      setTimeout(() => void this.poll(), 50);
      return true;
    } catch (error) {
      this.zone.run(() => (this.error = String(error)));
      return false;
    } finally {
      this.zone.run(() => (this.busy = false));
    }
  }
}

interface Latest {
  monotonic_event_time_ns: number;
  message: unknown;
}

async function latest(
  name: string,
  type: string,
  maxVectorSize: number,
): Promise<Latest | null> {
  const query = new URLSearchParams({
    name,
    type,
    max_vector_size: String(maxVectorSize),
  });
  try {
    const response = await fetch(`/api/latest?${query}`);
    return response.ok ? ((await response.json()) as Latest) : null;
  } catch {
    return null;
  }
}
