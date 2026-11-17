import {
  AfterViewInit,
  Component,
  ElementRef,
  Input,
  OnChanges,
  ViewChild,
} from '@angular/core';

import {CheckResult} from '../lib/calibration.js';
import {rampColor} from './distortion_map';

/**
 * Where in the image a check measured how much: each cell of its grid over the image in the
 * distortion maps' blue ramp, from dark (none) to light (`scale`), and gray where nothing was
 * measured.  Calibration errors live at places in the image, so this shows where the model is
 * off, not just by how much.
 */
@Component({
  selector: 'validation-grid',
  template: `
    <canvas #canvas></canvas>
  `,
})
export class ValidationGridView implements AfterViewInit, OnChanges {
  @Input({required: true}) result!: CheckResult;
  /** The value the ramp's light end stands for, shared by the calibrations compared. */
  @Input({required: true}) scale = 1;
  @Input() aspect = 1600 / 1304;

  @ViewChild('canvas') canvas?: ElementRef<HTMLCanvasElement>;

  ngAfterViewInit(): void {
    this.draw();
  }

  ngOnChanges(): void {
    this.draw();
  }

  private draw(): void {
    const canvas = this.canvas?.nativeElement;
    const {columns, rows, means} = this.result;
    if (!canvas || columns <= 0 || rows <= 0) {
      return;
    }
    const ratio = window.devicePixelRatio || 1;
    const cssWidth = canvas.clientWidth || 200;
    canvas.width = Math.round(cssWidth * ratio);
    canvas.height = Math.round((cssWidth * ratio) / this.aspect);
    const context = canvas.getContext('2d')!;
    const cellWidth = canvas.width / columns;
    const cellHeight = canvas.height / rows;
    for (let row = 0; row < rows; ++row) {
      for (let column = 0; column < columns; ++column) {
        const mean = means[row * columns + column] ?? -1;
        const [r, g, b] =
          mean < 0
            ? [60, 60, 60]
            : rampColor(this.scale > 0 ? mean / this.scale : 0);
        context.fillStyle = `rgb(${r}, ${g}, ${b})`;
        context.fillRect(
          column * cellWidth,
          row * cellHeight,
          Math.ceil(cellWidth),
          Math.ceil(cellHeight),
        );
      }
    }
  }
}
