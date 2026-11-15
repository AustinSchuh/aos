import {strict as assert} from 'node:assert';
import {test} from 'node:test';

import {
  cameraForTransceiver,
  parseFromStreamer,
  rejectsEveryTrack,
} from './signalling.js';

test("reads each of camera_streamer's messages", () => {
  assert.deepEqual(
    parseFromStreamer('{"type": "cameras", "cameras": [0, 1, 2, 3]}'),
    {
      type: 'cameras',
      cameras: [0, 1, 2, 3],
    },
  );
  assert.deepEqual(
    parseFromStreamer('{"type": "offer", "sdp": "v=0", "cameras": [2, 0]}'),
    {
      type: 'offer',
      sdp: 'v=0',
      cameras: [2, 0],
    },
  );
  assert.deepEqual(
    parseFromStreamer(
      '{"type": "ice", "candidate": "candidate:1", "sdpMLineIndex": 1}',
    ),
    {type: 'ice', candidate: 'candidate:1', sdpMLineIndex: 1},
  );
  assert.deepEqual(parseFromStreamer('{"type": "error", "message": "no"}'), {
    type: 'error',
    message: 'no',
  });
});

test('anything else is not a message', () => {
  for (const text of [
    'not json',
    '7',
    '{"type": "dance"}',
    '{"type": "cameras", "cameras": [-1]}',
    '{"type": "offer", "cameras": [0]}',
  ]) {
    assert.equal(parseFromStreamer(text), null, text);
  }
});

test("tracks are the offer's cameras in order", () => {
  assert.equal(cameraForTransceiver([2, 0], 0), 2);
  assert.equal(cameraForTransceiver([2, 0], 1), 0);
  assert.equal(cameraForTransceiver([2, 0], 2), null);
});

test('an answer with every port 0 refuses every track', () => {
  // As Chrome answered an offer of H.264 High, which its WebRTC doesn't decode.
  const refused =
    'v=0\r\na=group:BUNDLE\r\nm=video 0 UDP/TLS/RTP/SAVPF 0\r\na=mid:video0\r\n' +
    'm=video 0 UDP/TLS/RTP/SAVPF 0\r\na=mid:video1\r\n';
  assert.equal(rejectsEveryTrack(refused), true);
  const accepted =
    'v=0\r\na=group:BUNDLE video0 video1\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n' +
    'm=video 0 UDP/TLS/RTP/SAVPF 96\r\na=bundle-only\r\n';
  assert.equal(rejectsEveryTrack(accepted), false);
  assert.equal(rejectsEveryTrack('v=0\r\n'), false);
});
