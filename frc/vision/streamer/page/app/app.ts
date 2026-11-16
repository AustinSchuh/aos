import {Component, HostListener, OnInit} from '@angular/core';

import {
  exposureText,
  fixedWidth,
  gainText,
  sizeText,
  streamSizeText,
  tileLine,
} from '../lib/format.js';
import {sharedMax} from '../lib/distortion.js';
import {NODE, Robot} from './robot';
import {CameraStreams} from './streams';

type Tab = 'feeds' | 'settings';

/** The vision page: the cameras' feeds, and their settings and calibrations. */
@Component({
  selector: 'vision-app',
  templateUrl: './app.ng.html',
})
export class App implements OnInit {
  tab: Tab = 'feeds';
  readonly node = NODE;
  private distortionKey = '';
  private distortionMax = 0;

  constructor(
    readonly streams: CameraStreams,
    readonly robot: Robot,
  ) {}

  ngOnInit(): void {
    this.updateWatching();
  }

  show(tab: Tab): void {
    this.tab = tab;
    this.updateWatching();
  }

  /**
   * Stream while the page is visible, whichever tab shows, so going back to the feeds doesn't
   * set up the connection again; only a hidden page stops it.  Poll only for the feeds.
   */
  @HostListener('document:visibilitychange')
  updateWatching(): void {
    const visible = document.visibilityState === 'visible';
    this.streams.setWatching(visible);
    this.robot.setPolling(visible && this.tab === 'feeds');
  }

  get connected(): boolean {
    return this.streams.status === 'connected';
  }

  get size(): string {
    return sizeText(this.robot.settings);
  }

  /** One scale for every camera's distortion map, so they can be compared. */
  get distortionScale(): number {
    const width = this.robot.settings?.imageWidth ?? null;
    const height = this.robot.settings?.imageHeight ?? null;
    const lenses = this.robot.calibrations;
    const key = JSON.stringify([
      width,
      height,
      lenses.map((c) => [c.k, c.dist]),
    ]);
    if (key !== this.distortionKey) {
      this.distortionKey = key;
      this.distortionMax = sharedMax(lenses, width, height);
    }
    return this.distortionMax;
  }

  get exposure(): string {
    return exposureText(this.robot.settings);
  }

  get gain(): string {
    return gainText(this.robot.settings);
  }

  get cameras(): number[] {
    this.robot.setCameras(this.streams.cameras);
    return this.streams.cameras;
  }

  /** The size camera_streamer sends `camera` at. */
  streamSize(camera: number): string {
    const video = this.streams.videoFor(camera);
    return streamSizeText(video.width, video.height);
  }

  line(camera: number): string {
    const video = this.streams.videoFor(camera);
    return tileLine({
      robotFps: this.robot.robotFps.get(camera) ?? null,
      drawnFps: video.rates?.fps ?? null,
      kbps: video.rates?.kbps ?? null,
      delayMs: this.robot.delayMs.get(camera) ?? null,
      tags: this.robot.tags.get(camera) ?? null,
    });
  }

  /** All the cameras' video together, in a fixed width. */
  get totalLine(): string | null {
    const total = this.totalKbps;
    return total === null ? null : `${fixedWidth(total, 5)} kbit/s`;
  }

  /** All the cameras' video together, against the link's budget. */
  get totalKbps(): number | null {
    let total = 0;
    let any = false;
    for (const camera of this.streams.cameras) {
      const kbps = this.streams.videoFor(camera).rates?.kbps;
      if (kbps !== undefined) {
        total += kbps;
        any = true;
      }
    }
    return any ? total : null;
  }
}
