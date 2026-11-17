import {Component, Input} from '@angular/core';

import {
  Board,
  boardJson,
  boardProblem,
  CAMERA_ID_PATTERN,
  Capture,
  DICTIONARIES,
  guidance,
  pose,
  sameBoard,
  SavedBoard,
  Session,
  ValidationCheck,
  validationRows,
} from '../lib/calibration.js';
import {sharedMax} from '../lib/distortion.js';
import {CalibrationSession} from './calibration';
import {CameraStreams} from './streams';

/** A board as the editor holds it: lengths in millimeters, as people measure them. */
interface BoardForm {
  squaresX: number;
  squaresY: number;
  squareMm: number;
  markerMm: number;
  dictionary: string;
}

function toForm(board: Board): BoardForm {
  return {
    squaresX: board.squaresX,
    squaresY: board.squaresY,
    squareMm: Math.round(board.squareLength * 1e5) / 100,
    markerMm: Math.round(board.markerLength * 1e5) / 100,
    dictionary: board.dictionary,
  };
}

function fromForm(form: BoardForm): Board {
  return {
    squaresX: Number(form.squaresX),
    squaresY: Number(form.squaresY),
    squareLength: Number(form.squareMm) / 1000,
    markerLength: Number(form.markerMm) / 1000,
    dictionary: form.dictionary,
  };
}

/**
 * The Calibrate tab: start a session on a camera, collect captures of a ChArUco board while
 * watching the camera's video with the corners drawn in, solve, look the result over, and save
 * it to download and commit.
 */
@Component({
  selector: 'calibrate-panel',
  templateUrl: './calibrate.ng.html',
})
export class CalibratePanel {
  @Input({required: true}) cameras: number[] = [];

  readonly dictionaries = DICTIONARIES.map(([name]) => name);

  /** The camera to start on. */
  camera: number | null = null;
  targetCaptures = 70;
  /** The board being edited, and the session's board it was taken from. */
  form: BoardForm | null = null;
  private formFrom: Board | null = null;
  boardName = '';
  cameraId = '';
  private cameraIdFrom = '';
  /** The capture picked in the samples view, or by its error bar. */
  selected: number | null = null;
  readonly dropping = new Set<number>();
  force = false;
  /** The checks to make, and whether to check the solved calibration too. */
  readonly checks = new Set<ValidationCheck>(['STRAIGHTNESS', 'RIGIDITY']);
  validateSolved = true;
  readonly checkNames: [ValidationCheck, string, string][] = [
    [
      'STRAIGHTNESS',
      'Straightness',
      "Fit the whole board; its corners' leftover error shows distortion modelled wrong.",
    ],
    [
      'RIGIDITY',
      'Rigidity',
      'Fit each half of the board alone; how far apart the halves land is the drift between small targets.',
    ],
    [
      'TWO_TAG',
      'Two-tag rig',
      "Fit each of a rigid pair of AprilTags alone; their distance shouldn't change across the image.",
    ],
  ];

  constructor(
    readonly calibration: CalibrationSession,
    readonly streams: CameraStreams,
  ) {}

  get running(): boolean {
    return this.calibration.running;
  }

  get session(): Session | null {
    const session = this.calibration.session;
    return session && session.sessionId === this.calibration.status?.sessionId
      ? session
      : null;
  }

  get guidance(): string {
    const status = this.calibration.status;
    return status ? guidance(status) : '';
  }

  get cameraChoice(): number | null {
    return this.camera ?? this.cameras[0] ?? null;
  }

  stream(camera: number): MediaStream | null {
    return this.streams.videoFor(camera).stream;
  }

  /** The editor follows the session's board, unless it's being edited. */
  get board(): BoardForm | null {
    const board = this.session?.board ?? null;
    if (board && (!this.formFrom || !sameBoard(board, this.formFrom))) {
      this.form = toForm(board);
      this.formFrom = board;
    }
    return this.form;
  }

  get cameraIdField(): string {
    const id = this.session?.cameraId ?? '';
    if (id !== this.cameraIdFrom) {
      this.cameraId = id;
      this.cameraIdFrom = id;
    }
    return this.cameraId;
  }

  get boardChanged(): boolean {
    return (
      this.form !== null &&
      this.formFrom !== null &&
      !sameBoard(fromForm(this.form), this.formFrom)
    );
  }

  get boardProblem(): string | null {
    return this.form ? boardProblem(fromForm(this.form)) : null;
  }

  get boardSize(): string {
    const board = this.session?.board;
    if (!board) {
      return '';
    }
    const w = board.squaresX * board.squareLength * 1000;
    const h = board.squaresY * board.squareLength * 1000;
    return `${Math.round(w)} × ${Math.round(h)} mm`;
  }

  get canChangeBoard(): boolean {
    return (this.session?.captures.length ?? 0) === 0;
  }

  get cameraIdValid(): boolean {
    return CAMERA_ID_PATTERN.test(this.cameraId);
  }

  get captures(): Capture[] {
    return this.session?.captures ?? [];
  }

  /** The solve, if it's of the captures as they are now. */
  get solveCurrent(): boolean {
    const solve = this.session?.solve;
    return !!solve?.calibration && solve.captures === this.captures.length;
  }

  get residuals(): {dx: number; dy: number}[][] | null {
    const solve = this.session?.solve;
    return this.solveCurrent && solve && solve.residuals.length > 0
      ? solve.residuals
      : null;
  }

  /** The tallest error bar's error. */
  get maxCaptureError(): number {
    return Math.max(0.01, ...(this.session?.solve?.captureErrors ?? []));
  }

  get poses(): {distance: number; tiltDeg: number}[] {
    return this.captures.map(pose);
  }

  /** Where the pose plot puts a capture, in its 0–100 viewBox. */
  plotX(tiltDeg: number): number {
    return 8 + (Math.min(tiltDeg, 60) / 60) * 88;
  }

  plotY(distance: number): number {
    return 92 - (Math.min(distance, 4) / 4) * 84;
  }

  /** One scale for the current and new calibrations' distortion maps. */
  distortionScale(): number {
    const session = this.session;
    const lenses = [session?.current, session?.solve?.calibration].filter(
      (lens): lens is NonNullable<typeof lens> => !!lens,
    );
    return sharedMax(
      lenses,
      session?.imageWidth || null,
      session?.imageHeight || null,
    );
  }

  toggleCheck(check: ValidationCheck): void {
    if (this.checks.has(check)) {
      this.checks.delete(check);
    } else {
      this.checks.add(check);
    }
  }

  validate(): void {
    void this.calibration.send({
      action: 'VALIDATE',
      checks: [...this.checks],
      validate_solved: this.validateSolved && this.solveCurrent,
    });
  }

  stopValidating(): void {
    void this.calibration.send({action: 'STOP_VALIDATING'});
  }

  get validationRows() {
    return validationRows(this.session?.validation ?? []);
  }

  /** One scale for a measure's grids: its largest cell. */
  gridScale(byLens: ({means: number[]} | null)[]): number {
    let max = 0;
    for (const result of byLens) {
      for (const mean of result?.means ?? []) {
        max = Math.max(max, mean);
      }
    }
    return max;
  }

  async start(): Promise<void> {
    const camera = this.cameraChoice;
    if (camera === null) {
      return;
    }
    this.selected = null;
    this.dropping.clear();
    this.formFrom = null;
    await this.calibration.start(camera, this.targetCaptures);
  }

  capture(): void {
    void this.calibration.send({action: 'CAPTURE'});
  }

  undo(): void {
    this.selected = null;
    void this.calibration.send({action: 'UNDO'});
  }

  solve(): void {
    void this.calibration.send({action: 'SOLVE'});
  }

  pick(index: number): void {
    this.selected = this.selected === index ? null : index;
  }

  toggleDrop(index: number): void {
    if (this.dropping.has(index)) {
      this.dropping.delete(index);
    } else {
      this.dropping.add(index);
    }
  }

  async drop(): Promise<void> {
    const drop = [...this.dropping];
    if (await this.calibration.send({action: 'DROP', drop})) {
      this.dropping.clear();
      this.selected = null;
    }
  }

  setTarget(): void {
    void this.calibration.send({
      action: 'SET_TARGET',
      target_captures: Number(this.targetCaptures),
    });
  }

  applyBoard(): void {
    if (this.form && !this.boardProblem) {
      void this.calibration.send({
        action: 'SET_BOARD',
        board: boardJson(fromForm(this.form)),
      });
    }
  }

  choose(saved: SavedBoard): void {
    this.form = toForm(saved.board);
    this.applyBoard();
  }

  saveBoard(): void {
    if (this.form && this.boardName.trim() && !this.boardProblem) {
      void this.calibration.send({
        action: 'SAVE_BOARD',
        board_name: this.boardName.trim(),
        board: boardJson(fromForm(this.form)),
      });
    }
  }

  deleteBoard(name: string): void {
    void this.calibration.send({action: 'DELETE_BOARD', board_name: name});
  }

  setCameraId(): void {
    void this.calibration.send({
      action: 'SET_CAMERA_ID',
      camera_id: this.cameraId,
    });
  }

  save(): void {
    void this.calibration.send({action: 'SAVE', force: this.force});
  }

  finish(): void {
    void this.calibration.send({action: 'FINISH'});
  }

  cancel(): void {
    void this.calibration.send({action: 'CANCEL'});
  }

  printBoard(): void {
    const session = this.session;
    const board = session?.board;
    if (!session || !board) {
      return;
    }
    // At its size: the page's width in millimeters.
    const width = board.squaresX * board.squareLength * 1000;
    const page = window.open('', '_blank');
    page?.document.write(
      `<title>ChArUco board</title><style>@page{margin:0}body{margin:0}` +
        `img{width:${width}mm;display:block}</style>` +
        `<img src="${session.boardImagePath}" onload="print()">`,
    );
    page?.document.close();
  }
}
