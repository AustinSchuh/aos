import {
  AfterViewInit,
  Component,
  ElementRef,
  Input,
  OnChanges,
  ViewChild,
} from '@angular/core';

import {
  displacement,
  displacementField,
  Lens,
  MAP_COLUMNS,
  mapRows,
} from '../lib/distortion.js';

/** The blue ramp, from near the panel (no move) to light (the largest move). */
const RAMP = [
  '#0d366b',
  '#184f95',
  '#256abf',
  '#3987e5',
  '#6da7ec',
  '#9ec5f4',
  '#cde2fb',
].map((hex) => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16)));

/** `t` in [0, 1] along RAMP. */
function rampColor(t: number): number[] {
  const position = Math.min(Math.max(t, 0), 1) * (RAMP.length - 1);
  const i = Math.min(Math.floor(position), RAMP.length - 2);
  const f = position - i;
  return RAMP[i]!.map((a, k) => a + (RAMP[i + 1]![k]! - a) * f);
}

/** The arrows across. */
const ARROW_COLUMNS = 12;

/**
 * How far undistorting moves each of a camera's pixels: a heatmap of the distance, with arrows
 * from where the lens put a point to where an ideal pinhole camera would have, scaled up to see.
 */
@Component({
  selector: 'distortion-map',
  template: `
    <canvas
      #canvas
      (mousemove)="hover($event)"
      (mouseleave)="readout = null"
    ></canvas>
    <div class="distortion-legend">
      <span>0</span>
      <span class="distortion-ramp" [style.background]="rampCss"></span>
      <span>{{ scale | number: '1.1-1' }} px</span>
      <span *ngIf="scaleMax !== null">
        (this camera's most: {{ max | number: '1.1-1' }} px)
      </span>
      <span class="distortion-scale" *ngIf="arrowScale !== null">
        arrows ×{{ arrowScale | number: '1.0-1' }}
      </span>
    </div>
    <p class="distortion-readout">
      {{ readout ?? 'Hover for the move at a pixel.' }}
    </p>
  `,
})
export class DistortionMap implements AfterViewInit, OnChanges {
  @Input({required: true}) lens!: Lens;
  @Input({required: true}) width!: number;
  @Input({required: true}) height!: number;
  /** The move the ramp's light end and the longest arrow stand for, shared with other maps; this map's own largest if null. */
  @Input() scaleMax: number | null = null;

  @ViewChild('canvas') canvas?: ElementRef<HTMLCanvasElement>;

  /** This map's largest move. */
  max = 0;
  /** The move the ramp's light end stands for. */
  scale = 0;
  arrowScale: number | null = null;
  readout: string | null = null;
  readonly rampCss = `linear-gradient(to right, ${RAMP.map(
    (c) => `rgb(${c.join(',')})`,
  ).join(', ')})`;

  ngAfterViewInit(): void {
    this.draw();
  }

  ngOnChanges(): void {
    this.draw();
  }

  hover(event: MouseEvent): void {
    const canvas = event.target as HTMLCanvasElement;
    const box = canvas.getBoundingClientRect();
    const u = ((event.clientX - box.left) / box.width) * this.width;
    const v = ((event.clientY - box.top) / box.height) * this.height;
    const d = displacement(this.lens, u, v);
    const at = `(${u.toFixed(0)}, ${v.toFixed(0)})`;
    this.readout =
      d === null
        ? `${at}: the model doesn't invert here`
        : `${at} moves ${Math.hypot(d[0], d[1]).toFixed(1)} px ` +
          `(${d[0] >= 0 ? '+' : ''}${d[0].toFixed(1)}, ` +
          `${d[1] >= 0 ? '+' : ''}${d[1].toFixed(1)})`;
  }

  private draw(): void {
    const canvas = this.canvas?.nativeElement;
    if (!canvas || !(this.width > 0) || !(this.height > 0)) {
      return;
    }
    const columns = MAP_COLUMNS;
    const rows = mapRows(columns, this.width, this.height);
    const field = displacementField(
      this.lens,
      this.width,
      this.height,
      columns,
      rows,
    );
    this.max = field.max;
    this.scale = Math.max(this.scaleMax ?? 0, field.max);

    // The heatmap, one canvas pixel a cell, scaled up smoothly by CSS.
    const heat = document.createElement('canvas');
    heat.width = columns;
    heat.height = rows;
    const heatContext = heat.getContext('2d')!;
    const image = heatContext.createImageData(columns, rows);
    for (let i = 0; i < columns * rows; ++i) {
      const magnitude = Math.hypot(field.dx[i]!, field.dy[i]!);
      const color = Number.isNaN(magnitude)
        ? [60, 60, 60]
        : rampColor(this.scale > 0 ? magnitude / this.scale : 0);
      image.data.set([...color, 255], i * 4);
    }
    heatContext.putImageData(image, 0, 0);

    // Draw at the canvas's on-screen size, so the arrows stay sharp.
    const ratio = window.devicePixelRatio || 1;
    const cssWidth = canvas.clientWidth || 320;
    canvas.width = Math.round(cssWidth * ratio);
    canvas.height = Math.round((cssWidth * ratio * this.height) / this.width);
    const context = canvas.getContext('2d')!;
    context.imageSmoothingEnabled = true;
    context.drawImage(heat, 0, 0, canvas.width, canvas.height);

    // The arrows: each image pixel to where undistorting takes it, a move of `scale` about one
    // arrow's spacing long.
    const arrowRows = Math.max(
      1,
      Math.round((ARROW_COLUMNS * this.height) / this.width),
    );
    const arrows = displacementField(
      this.lens,
      this.width,
      this.height,
      ARROW_COLUMNS,
      arrowRows,
    );
    const spacing = this.width / ARROW_COLUMNS;
    this.arrowScale = this.scale > 0 ? (0.9 * spacing) / this.scale : null;
    const toCanvas = canvas.width / this.width;
    context.strokeStyle = '#e6e9ee';
    context.fillStyle = '#e6e9ee';
    context.lineWidth = 1.5 * ratio;
    for (let row = 0; row < arrowRows; ++row) {
      for (let column = 0; column < ARROW_COLUMNS; ++column) {
        const i = row * ARROW_COLUMNS + column;
        const [dx, dy] = [arrows.dx[i]!, arrows.dy[i]!];
        const x0 = ((column + 0.5) * this.width) / ARROW_COLUMNS;
        const y0 = ((row + 0.5) * this.height) / arrowRows;
        const [sx, sy] = [x0 * toCanvas, y0 * toCanvas];
        context.beginPath();
        context.arc(sx, sy, 1.5 * ratio, 0, 2 * Math.PI);
        context.fill();
        if (this.arrowScale === null || Number.isNaN(dx)) {
          continue;
        }
        const ex = sx + dx * this.arrowScale * toCanvas;
        const ey = sy + dy * this.arrowScale * toCanvas;
        const length = Math.hypot(ex - sx, ey - sy);
        if (length < 2 * ratio) {
          continue;
        }
        context.beginPath();
        context.moveTo(sx, sy);
        context.lineTo(ex, ey);
        // The head.
        const head = Math.min(6 * ratio, length / 2);
        const angle = Math.atan2(ey - sy, ex - sx);
        for (const side of [-1, 1]) {
          context.moveTo(ex, ey);
          context.lineTo(
            ex - head * Math.cos(angle + side * 0.45),
            ey - head * Math.sin(angle + side * 0.45),
          );
        }
        context.stroke();
      }
    }
  }
}
