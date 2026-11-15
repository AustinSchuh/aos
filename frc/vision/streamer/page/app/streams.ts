import {Injectable, NgZone, OnDestroy} from '@angular/core';

import {
  cameraForTransceiver,
  FromStreamer,
  parseFromStreamer,
  rejectsEveryTrack,
  ToStreamer,
} from '../lib/signalling.js';
import {InboundSample, RateWindow, StreamRates} from '../lib/rates.js';

/** What the page knows of one camera's video. */
export interface CameraVideo {
  stream: MediaStream | null;
  rates: StreamRates | null;
  width: number | null;
  height: number | null;
}

/**
 * Everything here that changes after an `await` does so inside Angular's zone: the bundle keeps
 * native async functions, whose continuations zone.js can't see, so changes made there would
 * only show at the next unrelated event.
 *
 * The cameras' video from camera_streamer: one WebRTC peer connection carrying every camera,
 * set up over wss://<robot>/cameras/ws.  Watches every camera while `watching`, and none
 * otherwise, so a hidden page costs the robot nothing.
 */
@Injectable({providedIn: 'root'})
export class CameraStreams implements OnDestroy {
  /** The cameras camera_streamer has. */
  cameras: number[] = [];
  /** 'connecting', 'connected', 'disconnected', or what went wrong. */
  status = 'connecting';
  readonly video = new Map<number, CameraVideo>();

  private socket: WebSocket | null = null;
  private pc: RTCPeerConnection | null = null;
  /** The cameras the current offer carries, in m-line order. */
  private offerCameras: number[] = [];
  private watching = true;
  private readonly windows = new Map<number, RateWindow>();
  private readonly statsTimer: ReturnType<typeof setInterval>;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private destroyed = false;
  /**
   * The signalling messages, handled one at a time in order: camera_streamer sends its ICE
   * candidates right after its offer, and a candidate added before the offer is set is
   * refused.  Without the robot's candidates, and with Chrome's hidden behind mDNS names the
   * robot can't resolve, the connection never gets past 'connecting'.
   */
  private messages: Promise<void> = Promise.resolve();

  constructor(private readonly zone: NgZone) {
    this.connect();
    this.statsTimer = setInterval(() => void this.sampleStats(), 400);
  }

  ngOnDestroy(): void {
    this.destroyed = true;
    clearInterval(this.statsTimer);
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
    }
    this.closeSession();
    this.socket?.close();
  }

  /** Whether to have camera_streamer send the cameras at all. */
  setWatching(watching: boolean): void {
    if (watching === this.watching) {
      return;
    }
    this.watching = watching;
    if (!watching) {
      this.closeSession();
    }
    this.sendWatch();
  }

  videoFor(camera: number): CameraVideo {
    let video = this.video.get(camera);
    if (!video) {
      video = {stream: null, rates: null, width: null, height: null};
      this.video.set(camera, video);
    }
    return video;
  }

  private send(message: ToStreamer): void {
    if (this.socket?.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify(message));
    }
  }

  private sendWatch(): void {
    this.send({type: 'watch', cameras: this.watching ? this.cameras : []});
  }

  private connect(): void {
    this.status = 'connecting';
    const socket = new WebSocket(`wss://${location.host}/cameras/ws`);
    this.socket = socket;
    socket.onmessage = (event) => {
      const message = parseFromStreamer(String(event.data));
      if (message) {
        this.messages = this.messages
          .then(() => this.onMessage(message))
          .catch((error) => console.warn('camera_streamer message:', error));
      }
    };
    socket.onclose = () => {
      this.status = 'disconnected';
      this.closeSession();
      if (!this.destroyed) {
        this.reconnectTimer = setTimeout(() => this.connect(), 2000);
      }
    };
  }

  private async onMessage(message: FromStreamer): Promise<void> {
    switch (message.type) {
      case 'cameras':
        this.cameras = message.cameras;
        for (const camera of this.cameras) {
          this.videoFor(camera);
        }
        this.sendWatch();
        break;
      case 'offer':
        try {
          await this.onOffer(message.sdp, message.cameras);
        } catch (error) {
          this.zone.run(() => (this.status = `failed: ${String(error)}`));
        }
        break;
      case 'ice':
        try {
          await this.pc?.addIceCandidate({
            candidate: message.candidate,
            sdpMLineIndex: message.sdpMLineIndex,
          });
        } catch (error) {
          console.warn('ICE candidate refused:', error);
        }
        break;
      case 'error':
        console.warn('camera_streamer:', message.message);
        this.status = message.message;
        break;
    }
  }

  private closeSession(): void {
    this.pc?.close();
    this.pc = null;
    this.offerCameras = [];
    this.windows.clear();
    for (const video of this.video.values()) {
      video.stream = null;
      video.rates = null;
    }
  }

  /** One offer carries every camera in `cameras`, its m-lines in the same order. */
  private async onOffer(sdp: string, cameras: number[]): Promise<void> {
    // A new offer (a camera restarted, or changed size) replaces the old session.
    this.closeSession();
    // Everything is on the robot's network: no STUN or TURN.
    const pc = new RTCPeerConnection({iceServers: []});
    this.pc = pc;
    this.offerCameras = cameras;
    pc.ontrack = (event) => {
      const camera = cameraForTransceiver(
        cameras,
        pc.getTransceivers().indexOf(event.transceiver),
      );
      if (camera !== null) {
        this.zone.run(
          () => (this.videoFor(camera).stream = new MediaStream([event.track])),
        );
      }
    };
    pc.onicecandidate = (event) => {
      if (event.candidate?.candidate) {
        this.send({
          type: 'ice',
          candidate: event.candidate.candidate,
          sdpMLineIndex: event.candidate.sdpMLineIndex ?? 0,
        });
      }
    };
    pc.onconnectionstatechange = () => {
      this.zone.run(() => (this.status = pc.connectionState));
    };
    await pc.setRemoteDescription({type: 'offer', sdp});
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    const answerSdp = pc.localDescription?.sdp ?? answer.sdp ?? '';
    if (rejectsEveryTrack(answerSdp)) {
      // Otherwise it would sit at 'connecting' with nothing to say why.
      const profiles = sdp.match(/profile-level-id=[0-9a-f]+/gi) ?? [];
      console.warn(
        'This browser refused every camera track; offered',
        [...new Set(profiles)].join(', '),
      );
      this.zone.run(
        () => (this.status = "failed: this browser can't decode the video"),
      );
    }
    this.send({type: 'answer', sdp: answerSdp});
  }

  /**
   * Each camera's frame and bit rates, from one stats report for the whole connection, so they
   * all change together.
   */
  private async sampleStats(): Promise<void> {
    const pc = this.pc;
    if (!pc) {
      return;
    }
    // Which camera each m-line (by its mid) carries.
    const cameraForMid = new Map<string, number>();
    pc.getTransceivers().forEach((transceiver, index) => {
      const camera = cameraForTransceiver(this.offerCameras, index);
      if (transceiver.mid !== null && camera !== null) {
        cameraForMid.set(transceiver.mid, camera);
      }
    });
    const report = await pc.getStats();
    const updates: [number, InboundSample, StreamRates | null][] = [];
    report.forEach((stats: Record<string, unknown>) => {
      if (stats['type'] !== 'inbound-rtp' || stats['kind'] !== 'video') {
        return;
      }
      const camera =
        typeof stats['mid'] === 'string'
          ? cameraForMid.get(stats['mid'])
          : undefined;
      if (camera === undefined) {
        return;
      }
      const number = (name: string) =>
        typeof stats[name] === 'number' ? (stats[name] as number) : null;
      const sample: InboundSample = {
        timestampMs: number('timestamp') ?? performance.now(),
        framesDecoded: number('framesDecoded') ?? 0,
        bytesReceived: number('bytesReceived') ?? 0,
        frameWidth: number('frameWidth'),
        frameHeight: number('frameHeight'),
      };
      let window = this.windows.get(camera);
      if (!window) {
        window = new RateWindow();
        this.windows.set(camera, window);
      }
      updates.push([camera, sample, window.add(sample)]);
    });
    if (pc !== this.pc) {
      return;
    }
    this.zone.run(() => {
      for (const [camera, sample, rates] of updates) {
        const video = this.videoFor(camera);
        video.rates = rates;
        video.width = sample.frameWidth;
        video.height = sample.frameHeight;
      }
    });
  }
}
