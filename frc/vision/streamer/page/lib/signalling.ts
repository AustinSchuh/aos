// camera_streamer's signalling messages (frc/vision/streamer/signalling.rs), over
// wss://<robot>/cameras/ws.

export type FromStreamer =
  | {type: 'cameras'; cameras: number[]}
  | {type: 'offer'; sdp: string; cameras: number[]}
  | {type: 'ice'; candidate: string; sdpMLineIndex: number}
  | {type: 'error'; message: string};

export type ToStreamer =
  | {type: 'watch'; cameras: number[]}
  | {type: 'answer'; sdp: string}
  | {type: 'ice'; candidate: string; sdpMLineIndex: number};

function isCameras(value: unknown): value is number[] {
  return (
    Array.isArray(value) && value.every((v) => Number.isInteger(v) && v >= 0)
  );
}

/** One websocket message from camera_streamer, or null if it isn't one. */
export function parseFromStreamer(text: string): FromStreamer | null {
  let value: unknown;
  try {
    value = JSON.parse(text);
  } catch {
    return null;
  }
  if (typeof value !== 'object' || value === null) {
    return null;
  }
  const m = value as Record<string, unknown>;
  switch (m['type']) {
    case 'cameras':
      return isCameras(m['cameras'])
        ? {type: 'cameras', cameras: m['cameras']}
        : null;
    case 'offer':
      return typeof m['sdp'] === 'string' && isCameras(m['cameras'])
        ? {type: 'offer', sdp: m['sdp'], cameras: m['cameras']}
        : null;
    case 'ice':
      return typeof m['candidate'] === 'string' &&
        typeof m['sdpMLineIndex'] === 'number'
        ? {
            type: 'ice',
            candidate: m['candidate'],
            sdpMLineIndex: m['sdpMLineIndex'],
          }
        : null;
    case 'error':
      return typeof m['message'] === 'string'
        ? {type: 'error', message: m['message']}
        : null;
    default:
      return null;
  }
}

/**
 * The camera a track carries: an offer's `cameras` lists them in its m-lines' order, which is
 * the order of the transceivers made from it.
 */
export function cameraForTransceiver(
  offerCameras: number[],
  index: number,
): number | null {
  return index >= 0 && index < offerCameras.length
    ? offerCameras[index]!
    : null;
}

/**
 * Whether an answer refuses every track: each m-line's port is 0, as a browser answers an
 * offer it can't decode any of.  The connection then never gets past 'connecting'.
 */
export function rejectsEveryTrack(answerSdp: string): boolean {
  const mLines = answerSdp
    .split(/\r?\n/)
    .filter((line) => line.startsWith('m='));
  return (
    mLines.length > 0 && mLines.every((line) => line.split(' ')[1] === '0')
  );
}
