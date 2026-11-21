// Frame and bit rates, from successive samples of counters.

/** The counters of one video track's `inbound-rtp` WebRTC stats that the rates come from. */
export interface InboundSample {
  /** When it was sampled, in ms. */
  timestampMs: number;
  framesDecoded: number;
  bytesReceived: number;
  frameWidth: number | null;
  frameHeight: number | null;
}

export interface StreamRates {
  fps: number;
  kbps: number;
}

/** The rates between two samples of the same track, or null if they can't say. */
export function streamRates(
  previous: InboundSample,
  next: InboundSample,
): StreamRates | null {
  const seconds = (next.timestampMs - previous.timestampMs) / 1000;
  const frames = next.framesDecoded - previous.framesDecoded;
  const bytes = next.bytesReceived - previous.bytesReceived;
  // Counters that went backwards are a new track.
  if (!(seconds > 0) || frames < 0 || bytes < 0) {
    return null;
  }
  return {fps: frames / seconds, kbps: (bytes * 8) / 1000 / seconds};
}

/**
 * A track's rates over about the last `windowMs`, from samples taken more often than that: the
 * page can update often without a short window's jitter (a frame more or less in 0.4 s is 8% at
 * 30 fps).
 */
export class RateWindow {
  private samples: InboundSample[] = [];

  constructor(private readonly windowMs = 1000) {}

  /** Adds a sample, and gives the rates from the oldest one in the window to it. */
  add(sample: InboundSample): StreamRates | null {
    const newest = this.samples[this.samples.length - 1];
    if (
      newest &&
      (sample.framesDecoded < newest.framesDecoded ||
        sample.bytesReceived < newest.bytesReceived)
    ) {
      // A new track: its counters started again.
      this.samples = [];
    }
    this.samples.push(sample);
    // Keep the newest sample at least windowMs old, as the window's start.
    while (
      this.samples.length > 2 &&
      sample.timestampMs - this.samples[1]!.timestampMs >= this.windowMs
    ) {
      this.samples.shift();
    }
    return this.samples.length < 2
      ? null
      : streamRates(this.samples[0]!, sample);
  }
}

/**
 * How long a track's frames have stopped for, from its `framesDecoded` sampled now and then.
 * When a camera is unplugged, or its camera_reader stops, camera_streamer has nothing to send,
 * and the video keeps showing its last frame as though nothing happened.
 */
export class FrameWatch {
  private decoded: number | null = null;
  private lastFrameMs = 0;

  /** Adds a sample, and gives how long, in ms, since the last new frame: 0 while they come. */
  add(framesDecoded: number, timestampMs: number): number {
    // Any change is a frame, or a new track whose counters started again.
    if (framesDecoded !== this.decoded) {
      this.decoded = framesDecoded;
      this.lastFrameMs = timestampMs;
      return 0;
    }
    return Math.max(0, timestampMs - this.lastFrameMs);
  }
}

/** One `frc.vision.TurboJpegDecoderStatus`: the frames decoded since the last, and when. */
export interface DecoderStatus {
  /** When it was sent, in seconds on the robot's monotonic clock. */
  sentTime: number;
  successfulDecodes: number;
  /** How long its counts cover, in seconds, if it says (0 from older decoders). */
  periodSeconds: number;
}

/**
 * The robot's frame rate for a camera, from its decoder's status messages: each counts the
 * frames decoded since the one before, about a second earlier.  One that says how long its
 * counts cover is enough by itself; otherwise it takes the time since `previous`.
 */
export function robotFps(
  previous: DecoderStatus | null,
  next: DecoderStatus,
): number | null {
  if (next.periodSeconds > 0) {
    return next.successfulDecodes / next.periodSeconds;
  }
  if (previous === null) {
    return null;
  }
  const seconds = next.sentTime - previous.sentTime;
  if (!(seconds > 0)) {
    return null;
  }
  return next.successfulDecodes / seconds;
}

/**
 * How long the detector took with a frame, in ms: from the frame's capture (the TargetMap's
 * monotonic_timestamp_ns) to sending its TargetMap, both on the robot's monotonic clock.  Null
 * if `latest` (as /api/latest gives it) doesn't say.
 */
export function detectionDelayMs(latest: unknown): number | null {
  if (typeof latest !== 'object' || latest === null) {
    return null;
  }
  const l = latest as Record<string, unknown>;
  const sent = l['monotonic_event_time_ns'];
  const message = l['message'];
  if (
    typeof sent !== 'number' ||
    typeof message !== 'object' ||
    message === null
  ) {
    return null;
  }
  const captured = (message as Record<string, unknown>)[
    'monotonic_timestamp_ns'
  ];
  if (typeof captured !== 'number' || captured <= 0 || sent < captured) {
    return null;
  }
  return (sent - captured) / 1e6;
}

/** How many tags a TargetMap (as /api/latest gives it) has. */
export function tagCount(targetMap: unknown): number {
  if (typeof targetMap !== 'object' || targetMap === null) {
    return 0;
  }
  const poses = (targetMap as Record<string, unknown>)['target_poses'];
  return Array.isArray(poses) ? poses.length : 0;
}
