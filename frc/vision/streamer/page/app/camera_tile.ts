import {
  Component,
  ElementRef,
  Input,
  OnChanges,
  ViewChild,
} from '@angular/core';

/** One camera: its header line, and its video, or "No signal" until frames come. */
@Component({
  selector: 'camera-tile',
  templateUrl: './camera_tile.ng.html',
})
export class CameraTile implements OnChanges {
  @Input({required: true}) camera!: number;
  @Input() stream: MediaStream | null = null;
  @Input() line = '';

  @ViewChild('video', {static: true}) video!: ElementRef<HTMLVideoElement>;

  /** Whether the stream has drawn a frame. */
  playing = false;

  ngOnChanges(): void {
    const video = this.video.nativeElement;
    if (video.srcObject !== this.stream) {
      video.srcObject = this.stream;
      this.playing = false;
    }
  }
}
