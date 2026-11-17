import {
  AfterViewInit,
  Component,
  ElementRef,
  EventEmitter,
  Input,
  OnChanges,
  Output,
  ViewChild,
} from '@angular/core';

import {Capture, coverage} from '../lib/calibration.js';

/** The coverage grid's cells across; rows follow the image's shape. */
const GRID_COLUMNS = 16;

/**
 * Where the captures' corners are in the image, as intrinsics_calibration's "Captured Point
 * Visualization" showed them: a grid shading the cells no capture has reached (the image's
 * corners and edges fix the distortion, and are the easiest to miss), every capture's corners,
 * the selected or newest one's outlined, and after a solve each corner's leftover error as a
 * magnified arrow.  Clicking picks the capture with the nearest corner.
 */
@Component({
  selector: 'samples-view',
  template: `
    <canvas #canvas (click)="click($event)"></canvas>
    <div class="samples-legend">
      <span>
        <span class="swatch empty"></span>
        no capture yet
      </span>
      <span>
        <span class="swatch dot"></span>
        corners
      </span>
      <span *ngIf="residuals">
        <span class="swatch arrow"></span>
        error ×{{ residualScale }}
      </span>
    </div>
  `,
})
export class SamplesView implements AfterViewInit, OnChanges {
  @Input({required: true}) captures: Capture[] = [];
  @Input({required: true}) width = 0;
  @Input({required: true}) height = 0;
  /** Each capture's corners' errors, in pixels, if solved. */
  @Input() residuals: {dx: number; dy: number}[][] | null = null;
  @Input() selected: number | null = null;
  /** Captures marked to drop. */
  @Input() dropping: ReadonlySet<number> = new Set();
  @Output() readonly pick = new EventEmitter<number>();

  @ViewChild('canvas') canvas?: ElementRef<HTMLCanvasElement>;

  readonly residualScale = 50;

  ngAfterViewInit(): void {
    this.draw();
  }

  ngOnChanges(): void {
    this.draw();
  }

  click(event: MouseEvent): void {
    const canvas = event.target as HTMLCanvasElement;
    const box = canvas.getBoundingClientRect();
    const x = ((event.clientX - box.left) / box.width) * this.width;
    const y = ((event.clientY - box.top) / box.height) * this.height;
    let best = -1;
    let bestDistance = Infinity;
    this.captures.forEach((capture, index) => {
      for (const corner of capture.corners) {
        const distance = Math.hypot(corner.x - x, corner.y - y);
        if (distance < bestDistance) {
          bestDistance = distance;
          best = index;
        }
      }
    });
    if (best >= 0 && bestDistance < this.width / 20) {
      this.pick.emit(best);
    }
  }

  private draw(): void {
    const canvas = this.canvas?.nativeElement;
    if (!canvas || !(this.width > 0) || !(this.height > 0)) {
      return;
    }
    const style = getComputedStyle(canvas);
    const color = (name: string, fallback: string) =>
      style.getPropertyValue(name).trim() || fallback;
    const ratio = window.devicePixelRatio || 1;
    const cssWidth = canvas.clientWidth || 480;
    canvas.width = Math.round(cssWidth * ratio);
    canvas.height = Math.round((cssWidth * ratio * this.height) / this.width);
    const scale = canvas.width / this.width;
    const context = canvas.getContext('2d')!;
    context.fillStyle = color('--bg', '#111418');
    context.fillRect(0, 0, canvas.width, canvas.height);

    // The grid: cells no capture has reached, tinted to draw the eye.
    const rows = Math.max(
      1,
      Math.round((GRID_COLUMNS * this.height) / this.width),
    );
    const counts = coverage(
      this.captures,
      this.width,
      this.height,
      GRID_COLUMNS,
      rows,
    );
    const cellWidth = canvas.width / GRID_COLUMNS;
    const cellHeight = canvas.height / rows;
    context.fillStyle = color('--warn', '#f0b429');
    for (let row = 0; row < rows; ++row) {
      for (let column = 0; column < GRID_COLUMNS; ++column) {
        if (counts[row * GRID_COLUMNS + column] === 0) {
          context.globalAlpha = 0.14;
          context.fillRect(
            column * cellWidth + 1,
            row * cellHeight + 1,
            cellWidth - 2,
            cellHeight - 2,
          );
        }
      }
    }
    context.globalAlpha = 1;
    context.strokeStyle = color('--line', '#2c333d');
    context.lineWidth = 1;
    for (let column = 1; column < GRID_COLUMNS; ++column) {
      context.beginPath();
      context.moveTo(column * cellWidth, 0);
      context.lineTo(column * cellWidth, canvas.height);
      context.stroke();
    }
    for (let row = 1; row < rows; ++row) {
      context.beginPath();
      context.moveTo(0, row * cellHeight);
      context.lineTo(canvas.width, row * cellHeight);
      context.stroke();
    }

    // Every capture's corners.
    const highlight =
      this.selected ??
      (this.captures.length > 0 ? this.captures.length - 1 : null);
    const radius = 1.6 * ratio;
    this.captures.forEach((capture, index) => {
      if (index === highlight) {
        return;
      }
      context.fillStyle = this.dropping.has(index)
        ? color('--bad', '#e25c5c')
        : color('--accent', '#4ea1ff');
      context.globalAlpha = this.dropping.has(index) ? 0.8 : 0.55;
      for (const corner of capture.corners) {
        context.beginPath();
        context.arc(corner.x * scale, corner.y * scale, radius, 0, 2 * Math.PI);
        context.fill();
      }
    });
    context.globalAlpha = 1;

    // Each corner's error, magnified.
    if (this.residuals) {
      context.strokeStyle = color('--text', '#e6e9ee');
      context.lineWidth = 1 * ratio;
      this.captures.forEach((capture, index) => {
        const residuals = this.residuals?.[index];
        if (!residuals) {
          return;
        }
        capture.corners.forEach((corner, i) => {
          const r = residuals[i];
          if (!r) {
            return;
          }
          const x = corner.x * scale;
          const y = corner.y * scale;
          context.beginPath();
          context.moveTo(x, y);
          context.lineTo(
            x + r.dx * this.residualScale * scale,
            y + r.dy * this.residualScale * scale,
          );
          context.stroke();
        });
      });
    }

    // The highlighted capture: its corners, larger, and their outline.
    if (highlight !== null && this.captures[highlight]) {
      const corners = this.captures[highlight]!.corners;
      context.fillStyle = color('--ok', '#3ccf6e');
      for (const corner of corners) {
        context.beginPath();
        context.arc(
          corner.x * scale,
          corner.y * scale,
          radius * 1.8,
          0,
          2 * Math.PI,
        );
        context.fill();
      }
      const hull = convexHull(corners.map((c) => [c.x * scale, c.y * scale]));
      if (hull.length > 2) {
        context.strokeStyle = color('--ok', '#3ccf6e');
        context.lineWidth = 2 * ratio;
        context.beginPath();
        hull.forEach(([x, y], i) =>
          i === 0 ? context.moveTo(x, y) : context.lineTo(x, y),
        );
        context.closePath();
        context.stroke();
      }
    }
  }
}

/** The convex hull of `points`, by Andrew's monotone chain. */
function convexHull(points: [number, number][]): [number, number][] {
  const sorted = [...points].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (sorted.length < 3) {
    return sorted;
  }
  const cross = (
    o: [number, number],
    a: [number, number],
    b: [number, number],
  ) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower: [number, number][] = [];
  for (const p of sorted) {
    while (
      lower.length >= 2 &&
      cross(lower[lower.length - 2]!, lower[lower.length - 1]!, p) <= 0
    ) {
      lower.pop();
    }
    lower.push(p);
  }
  const upper: [number, number][] = [];
  for (const p of [...sorted].reverse()) {
    while (
      upper.length >= 2 &&
      cross(upper[upper.length - 2]!, upper[upper.length - 1]!, p) <= 0
    ) {
      upper.pop();
    }
    upper.push(p);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}
