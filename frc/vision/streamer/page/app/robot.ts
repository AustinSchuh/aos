import {Injectable, NgZone, OnDestroy} from '@angular/core';
import {ByteBuffer} from 'flatbuffers';

import {Connection} from '../../../../../aos/network/www/proxy';
import {TurboJpegDecoderStatus} from '../../../turbojpeg_decoder_status_ts_fbs/frc/vision';
import {calibrations, Calibration} from '../lib/intrinsics.js';
import {streamSettings, StreamSettings} from '../lib/format.js';
import {
  DecoderStatus,
  detectionDelayMs,
  robotFps,
  tagCount,
} from '../lib/rates.js';

/** /api/latest's answer. */
interface Latest {
  monotonic_event_time_ns: number;
  message: unknown;
}

async function latest(
  name: string,
  type: string,
  maxVectorSize = 100,
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

/**
 * Where robot_server serves the robot's own camera settings, written by
 * field_side_exposure_adjuster's --www_file while it has any.
 */
const OWN_SETTINGS_PATH = '/camera_settings/camera_settings.json';

/** The robot's node, whose cameras' calibrations to show. */
export const NODE = 'orin';

/**
 * What the robot says about its cameras, besides their video:
 *   - their calibrations and settings, from /constants, by GET (/api/latest);
 *   - their frame rates, from their decoders' 1 Hz status, over web_proxy's WebRTC data
 *     channels (as the localizer's page gets its data);
 *   - their exposure and gain, one for every camera: what field_side_exposure_adjuster last sent
 *     a camera, else the constants'; set by POSTing a CameraStreamSettings to /camera_settings,
 *     which it saves;
 *   - how many tags each sees, and how long its detector takes, by GETting its latest TargetMap
 *     three times a second while `polling`.
 *     web_proxy would send every one, 60 a second a camera, which is more of the link than a
 *     count is worth.
 */
@Injectable({providedIn: 'root'})
export class Robot implements OnDestroy {
  settings: StreamSettings | null = null;
  calibrations: Calibration[] = [];
  readonly robotFps = new Map<number, number | null>();
  readonly tags = new Map<number, number | null>();
  /** How long each camera's detector took with its latest frame, in ms. */
  readonly delayMs = new Map<number, number | null>();
  /** What the cameras were last sent, when they have been. */
  sent: {exposure100us?: number; gain?: number} = {};
  /** Why the last change of exposure or gain failed. */
  settingsError = '';
  /**
   * The robot's own camera settings, which differ from constants.json, as
   * field_side_exposure_adjuster offers them to check in; null while it has
   * none.
   */
  ownSettingsPath: string | null = null;

  private polling = true;
  private watchingSettings = false;
  private readonly settingsTimer: ReturnType<typeof setInterval>;
  private readonly lastStatus = new Map<number, DecoderStatus>();
  private readonly tagTimer: ReturnType<typeof setInterval>;
  private constantsTimer: ReturnType<typeof setTimeout> | null = null;
  private cameras: number[] = [];

  constructor(private readonly zone: NgZone) {
    void this.loadConstants();
    this.tagTimer = setInterval(() => void this.pollTags(), 333);
    this.settingsTimer = setInterval(() => void this.pollSettings(), 1000);
    this.connectProxy();
  }

  ngOnDestroy(): void {
    clearInterval(this.tagTimer);
    clearInterval(this.settingsTimer);
    if (this.constantsTimer !== null) {
      clearTimeout(this.constantsTimer);
    }
  }

  setCameras(cameras: number[]): void {
    this.cameras = cameras;
  }

  setPolling(polling: boolean): void {
    this.polling = polling;
  }

  /** Whether to keep the exposure and gain current: while the settings are shown. */
  setWatchingSettings(watching: boolean): void {
    this.watchingSettings = watching;
    if (watching) {
      void this.pollSettings();
    }
  }

  /** Every camera's exposure, in 100 us units, 0 for auto. */
  get exposure(): number {
    return this.sent.exposure100us ?? this.settings?.exposure100us ?? 0;
  }

  /** Every camera's gain; null for the driver's default. */
  get gain(): number | null {
    return this.sent.gain ?? this.settings?.gain ?? null;
  }

  /**
   * Sets every camera's exposure (in 100 us units, 0 for auto) or gain.  They're saved on the
   * robot, so they hold across restarts.
   */
  async setCameraSettings(change: {
    exposure100us?: number;
    gain?: number;
  }): Promise<void> {
    const message: Record<string, number> = {};
    if (change.exposure100us !== undefined) {
      message['exposure_100us'] = Math.round(change.exposure100us);
    }
    if (change.gain !== undefined) {
      message['gain'] = Math.round(change.gain);
    }
    const query = new URLSearchParams({
      name: '/camera_settings',
      type: 'frc.vision.CameraStreamSettings',
    });
    let error = '';
    try {
      const response = await fetch(`/api/send?${query}`, {
        method: 'POST',
        body: JSON.stringify(message),
      });
      if (!response.ok) {
        const body = (await response.json().catch(() => ({}))) as {
          error?: string;
        };
        error = body.error ?? `HTTP ${response.status}`;
      }
    } catch (e) {
      error = String(e);
    }
    this.zone.run(() => {
      this.settingsError = error;
      if (!error) {
        this.sent = {...this.sent, ...change};
      }
    });
  }

  /** What the cameras were last sent, and whether the robot has its own. */
  private async pollSettings(): Promise<void> {
    if (!this.watchingSettings) {
      return;
    }
    void this.pollOwnSettings();
    const cameras = this.cameras.length > 0 ? this.cameras : [0, 1, 2, 3];
    for (const camera of cameras) {
      const sent = await latest(
        `/camera${camera}`,
        'frc.vision.CameraStreamSettings',
        0,
      );
      const message = sent?.message as Record<string, unknown> | undefined;
      if (!message) {
        continue;
      }
      const exposure = message['exposure_100us'];
      const gain = message['gain'];
      this.zone.run(() => {
        this.sent = {
          exposure100us: typeof exposure === 'number' ? exposure : undefined,
          gain: typeof gain === 'number' ? gain : undefined,
        };
      });
      return;
    }
  }

  /** Whether the robot offers its own settings, as long as it has any. */
  private async pollOwnSettings(): Promise<void> {
    let own: string | null = null;
    try {
      const response = await fetch(OWN_SETTINGS_PATH, {
        method: 'HEAD',
        cache: 'no-store',
      });
      own = response.ok ? OWN_SETTINGS_PATH : null;
    } catch {
      own = null;
    }
    this.zone.run(() => (this.ownSettingsPath = own));
  }

  /** Forgets the robot's own exposure and gain: back to constants.json's. */
  useConstantsSettings(): Promise<void> {
    return this.setCameraSettings({}).then(() => this.pollSettings());
  }

  /** The constants, retried until the robot has them. */
  private async loadConstants(): Promise<void> {
    // 14 elements, for every distortion coefficient.
    const constants = await latest(
      '/constants',
      'frc.vision.CameraConstants',
      16,
    );
    if (!constants) {
      this.constantsTimer = setTimeout(() => void this.loadConstants(), 5000);
      return;
    }
    // In the zone, so it shows: see CameraStreams.
    this.zone.run(() => {
      this.settings = streamSettings(constants.message);
      this.calibrations = calibrations(constants.message, NODE);
    });
  }

  private async pollTags(): Promise<void> {
    if (!this.polling) {
      return;
    }
    await Promise.all(
      this.cameras.map(async (camera) => {
        const map = await latest(
          `/camera${camera}/gray`,
          'frc.vision.TargetMap',
          0,
        );
        this.zone.run(() => {
          this.tags.set(camera, map ? tagCount(map.message) : null);
          this.delayMs.set(camera, detectionDelayMs(map));
        });
      }),
    );
  }

  private connectProxy(): void {
    const connection = new Connection();
    connection.addConfigHandler((config) => {
      for (let i = 0; i < config.channelsLength(); ++i) {
        const channel = config.channels(i);
        const match = /^\/camera(\d+)\/gray$/.exec(channel?.name() ?? '');
        if (!match || channel?.type() !== 'frc.vision.TurboJpegDecoderStatus') {
          continue;
        }
        const camera = Number(match[1]);
        connection.addHandler(
          channel.name()!,
          channel.type()!,
          (data, sentTime) => {
            const status =
              TurboJpegDecoderStatus.getRootAsTurboJpegDecoderStatus(
                new ByteBuffer(data),
              );
            const next = {
              sentTime,
              successfulDecodes: status.successfulDecodes(),
              periodSeconds: Number(status.periodNs()) / 1e9,
            };
            const last = this.lastStatus.get(camera) ?? null;
            // The same status can come more than once; only a newer one says anything.
            if (last !== null && next.sentTime <= last.sentTime) {
              return;
            }
            this.lastStatus.set(camera, next);
            const fps = robotFps(last, next);
            if (fps !== null) {
              this.zone.run(() => this.robotFps.set(camera, fps));
            }
          },
        );
      }
    });
    connection.connect();
  }
}
