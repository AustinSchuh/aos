import {
  Component,
  ElementRef,
  Input,
  OnChanges,
  ViewChild,
} from '@angular/core';

import {stoppedText} from '../lib/format.js';

/**
 * One camera: its header line, and its video, or "No signal" until frames come.  Its last frame
 * is dimmed, and says for how long, once frames stop.
 */
@Component({
  selector: 'camera-tile',
  templateUrl: './camera_tile.ng.html',
})
export class CameraTile implements OnChanges {
  @Input({required: true}) camera!: number;
  @Input() stream: MediaStream | null = null;
  @Input() line = '';
  /** How long since the stream's last new frame, in ms. */
  @Input() stoppedMs = 0;

  @ViewChild('video', {static: true}) video!: ElementRef<HTMLVideoElement>;

  /** Whether the stream has drawn a frame. */
  playing = false;

  get stopped(): string | null {
    return this.stream && this.playing ? stoppedText(this.stoppedMs) : null;
  }

  ngOnChanges(): void {
    const video = this.video.nativeElement;
    if (video.srcObject !== this.stream) {
      video.srcObject = this.stream;
      this.playing = false;
    }
  }
}
