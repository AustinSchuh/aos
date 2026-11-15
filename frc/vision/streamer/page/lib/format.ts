// How the camera page writes camera settings and rates, as change 1749's dashboard writes them.

/** Settings from a robot's `default_camera_stream_settings` in /constants. */
export interface StreamSettings {
  imageWidth?: number;
  imageHeight?: number;
  /** Manual exposure in units of 100 us; absent means auto exposure, as camera_reader reads it. */
  exposure100us?: number;
  gain?: number;
}

function numberField(
  object: Record<string, unknown>,
  name: string,
): number | undefined {
  const value = object[name];
  return typeof value === 'number' ? value : undefined;
}

/** The settings in a CameraConstants message, as /api/latest gives it. */
export function streamSettings(constants: unknown): StreamSettings | null {
  if (typeof constants !== 'object' || constants === null) {
    return null;
  }
  const settings = (constants as Record<string, unknown>)[
    'default_camera_stream_settings'
  ];
  if (typeof settings !== 'object' || settings === null) {
    return null;
  }
  const s = settings as Record<string, unknown>;
  const result: StreamSettings = {};
  const width = numberField(s, 'image_width');
  const height = numberField(s, 'image_height');
  const exposure = numberField(s, 'exposure_100us');
  const gain = numberField(s, 'gain');
  if (width !== undefined) result.imageWidth = width;
  if (height !== undefined) result.imageHeight = height;
  if (exposure !== undefined) result.exposure100us = exposure;
  if (gain !== undefined) result.gain = gain;
  return result;
}

/** "6.0 ms (60)", or "auto". */
export function exposureText(settings: StreamSettings | null): string {
  if (!settings || settings.exposure100us === undefined) {
    return 'auto';
  }
  return `${(settings.exposure100us / 10).toFixed(1)} ms (${settings.exposure100us})`;
}

/** "0", or "driver default" when constants don't set it. */
export function gainText(settings: StreamSettings | null): string {
  if (!settings || settings.gain === undefined) {
    return 'driver default';
  }
  return `${settings.gain}`;
}

/** "1600×1304", or "camera default" when constants don't set it. */
export function sizeText(settings: StreamSettings | null): string {
  if (
    settings?.imageWidth === undefined ||
    settings.imageHeight === undefined
  ) {
    return 'camera default';
  }
  return `${settings.imageWidth}×${settings.imageHeight}`;
}

/** A figure space: as wide as a digit, in a font with tabular digits. */
const FIGURE_SPACE = '\u2007';

/**
 * `value` with `decimals` places, padded with figure spaces to `width` characters, or a dash so
 * padded if it isn't known: so a number's digits don't move as it changes.
 */
export function fixedWidth(
  value: number | null,
  width: number,
  decimals = 0,
): string {
  const text =
    value === null || !Number.isFinite(value) ? '–' : value.toFixed(decimals);
  return text.padStart(width, FIGURE_SPACE);
}

/** What a camera tile's header says about its stream. */
export interface TileRates {
  /** Frames a second the robot decodes, if known. */
  robotFps: number | null;
  /** Frames a second this browser draws, if it has any. */
  drawnFps: number | null;
  kbps: number | null;
  /** How long the detector took with its latest frame, in ms, if it is running. */
  delayMs: number | null;
  /** AprilTags the detector found in its latest frame, if it is running. */
  tags: number | null;
}

/**
 * " 60.0 →  30.0 FPS · 1040 kbit/s ·  12 ms ·  2 tags", each number in a fixed width so the
 * line holds still as they change, in a monospaced font.
 */
export function tileLine(rates: TileRates): string {
  return [
    `${fixedWidth(rates.robotFps, 5, 1)} → ${fixedWidth(rates.drawnFps, 5, 1)} FPS`,
    `${fixedWidth(rates.kbps, 4)} kbit/s`,
    `${fixedWidth(rates.delayMs, 3)} ms`,
    `${fixedWidth(rates.tags, 2)} tags`,
  ].join(' · ');
}

/** "800×652", or "–" before the stream has said. */
export function streamSizeText(
  width: number | null,
  height: number | null,
): string {
  return width !== null && height !== null ? `${width}×${height}` : '–';
}
