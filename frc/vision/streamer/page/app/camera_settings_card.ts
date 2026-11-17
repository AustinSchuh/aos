import {Component, OnDestroy} from '@angular/core';

import {
  EXPOSURE_SLIDER_STEPS,
  exposureMsText,
  exposureSlider,
  sliderExposure,
} from '../lib/format.js';
import {Robot} from './robot';

/** How long a dragged exposure shows over what the robot last said, in ms. */
const HOLD_MS = 2500;
/** The fewest ms between exposures sent while dragging. */
const SEND_EVERY_MS = 150;
/** A manual exposure to start from when leaving auto with none set: 10 ms. */
const DEFAULT_MANUAL_100US = 100;

/**
 * Every camera's exposure and gain.  Dragging the exposure sends it as it goes, so the feeds
 * show the change; field_side_exposure_adjuster saves what's sent, so it holds across restarts.
 */
@Component({
  selector: 'camera-settings-card',
  templateUrl: './camera_settings_card.ng.html',
})
export class CameraSettingsCard implements OnDestroy {
  readonly steps = EXPOSURE_SLIDER_STEPS;

  /** The exposure being dragged to, shown until the robot catches up. */
  private held: number | null = null;
  private heldAt = 0;
  /** The last manual exposure, to go back to from auto. */
  private manual: number | null = null;
  private sendTimer: ReturnType<typeof setTimeout> | null = null;
  private lastSentAt = 0;

  constructor(readonly robot: Robot) {}

  ngOnDestroy(): void {
    if (this.sendTimer !== null) {
      clearTimeout(this.sendTimer);
    }
  }

  /**
   * Whether the robot's constants have come, with the settings to start from.  Until they do
   * there's nothing to show, and nothing to change.
   */
  get known(): boolean {
    return this.robot.settings !== null;
  }

  /** The exposure to show, in 100 us units, 0 for auto. */
  get exposure(): number {
    if (this.held !== null && Date.now() - this.heldAt < HOLD_MS) {
      return this.held;
    }
    return this.robot.exposure;
  }

  get auto(): boolean {
    return this.exposure <= 0;
  }

  get slider(): number {
    const exposure = this.exposure;
    return exposureSlider(exposure > 0 ? exposure : this.manualExposure());
  }

  get exposureText(): string {
    return this.known ? exposureMsText(this.exposure) : '–';
  }

  get gain(): number | null {
    return this.robot.gain;
  }

  get defaults(): string {
    const settings = this.robot.settings;
    if (settings === null) {
      return 'loading';
    }
    const gain = settings?.gain;
    return (
      `exposure ${exposureMsText(settings?.exposure100us ?? 0)}, ` +
      `gain ${gain === undefined ? 'the driver default' : gain}`
    );
  }

  slide(event: Event): void {
    const exposure = sliderExposure(
      Number((event.target as HTMLInputElement).value),
    );
    this.manual = exposure;
    this.hold(exposure);
  }

  setAuto(event: Event): void {
    const auto = (event.target as HTMLInputElement).checked;
    this.hold(auto ? 0 : this.manualExposure());
  }

  setGain(event: Event): void {
    const input = event.target as HTMLInputElement;
    const gain = Number(input.value);
    if (input.value === '' || !Number.isInteger(gain) || gain < 0) {
      input.value = this.gain === null ? '' : String(this.gain);
      return;
    }
    void this.robot.setCameraSettings({gain});
  }

  useConstants(): void {
    this.held = null;
    this.manual = null;
    void this.robot.useConstantsSettings();
  }

  private manualExposure(): number {
    return (
      this.manual ??
      (this.robot.settings?.exposure100us || DEFAULT_MANUAL_100US)
    );
  }

  /** Shows `exposure` and sends it, no more often than SEND_EVERY_MS. */
  private hold(exposure: number): void {
    this.held = exposure;
    this.heldAt = Date.now();
    if (this.sendTimer !== null) {
      return;
    }
    const wait = Math.max(0, this.lastSentAt + SEND_EVERY_MS - Date.now());
    this.sendTimer = setTimeout(() => {
      this.sendTimer = null;
      this.lastSentAt = Date.now();
      this.heldAt = Date.now();
      void this.robot.setCameraSettings({exposure100us: this.held ?? 0});
    }, wait);
  }
}
