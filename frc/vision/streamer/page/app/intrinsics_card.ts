import {Component, Input} from '@angular/core';

import {imageSize} from '../lib/distortion.js';
import {Calibration, REPROJECTION_WARNING_PX} from '../lib/intrinsics.js';

/** One camera's calibration, as the Intrinsics tab shows it. */
@Component({
  selector: 'intrinsics-card',
  templateUrl: './intrinsics_card.ng.html',
})
export class IntrinsicsCard {
  @Input({required: true}) calibration!: Calibration;
  /** The image size the calibration is for; from the principal point, if not known. */
  @Input() imageWidth: number | null = null;
  @Input() imageHeight: number | null = null;
  /** The distortion map's scale, shared by every camera's. */
  @Input() distortionScale: number | null = null;

  get width(): number {
    return imageSize(this.calibration, this.imageWidth, this.imageHeight)[0];
  }

  get height(): number {
    return imageSize(this.calibration, this.imageWidth, this.imageHeight)[1];
  }

  readonly warningPx = REPROJECTION_WARNING_PX;

  /** The camera matrix's rows. */
  get rows(): number[][] {
    const k = this.calibration.k;
    return [k.slice(0, 3), k.slice(3, 6), k.slice(6, 9)];
  }

  get warn(): boolean {
    return (this.calibration.reprojectionError ?? 0) > REPROJECTION_WARNING_PX;
  }
}
