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

/** The robot's node, whose cameras' calibrations to show. */
export const NODE = 'orin';

/**
 * What the robot says about its cameras, besides their video:
 *   - their calibrations and settings, from /constants, by GET (/api/latest);
 *   - their frame rates, from their decoders' 1 Hz status, over web_proxy's WebRTC data
 *     channels (as the localizer's page gets its data);
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

  private polling = true;
  private readonly lastStatus = new Map<number, DecoderStatus>();
  private readonly tagTimer: ReturnType<typeof setInterval>;
  private constantsTimer: ReturnType<typeof setTimeout> | null = null;
  private cameras: number[] = [];

  constructor(private readonly zone: NgZone) {
    void this.loadConstants();
    this.tagTimer = setInterval(() => void this.pollTags(), 333);
    this.connectProxy();
  }

  ngOnDestroy(): void {
    clearInterval(this.tagTimer);
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
