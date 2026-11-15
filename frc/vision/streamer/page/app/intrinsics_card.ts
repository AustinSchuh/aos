import {Component, Input} from '@angular/core';

import {Calibration, REPROJECTION_WARNING_PX} from '../lib/intrinsics.js';

/** One camera's calibration, as the Intrinsics tab shows it. */
@Component({
  selector: 'intrinsics-card',
  templateUrl: './intrinsics_card.ng.html',
})
export class IntrinsicsCard {
  @Input({required: true}) calibration!: Calibration;

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
