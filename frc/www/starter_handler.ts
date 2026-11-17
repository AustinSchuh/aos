import {ByteBuffer} from 'flatbuffers';
import {Connection} from '../../aos/network/www/proxy';
import {
  Status,
  State,
  LastStopReason,
  FileState,
} from '../../aos/starter/starter_ts_fbs/aos/starter';

const NODES = ['', '/orin1', '/imu', '/roborio'];

/** One application's row, as shown. */
interface Row {
  name: string;
  node: string;
  state: State;
  lastExitCode: number;
  pid: number;
  lastStartSeconds: number;
  lastStopReason: LastStopReason;
  fileState: FileState;
}

/** A state's look: running is good, starting or waiting in between, else bad. */
function stateClass(state: State): string {
  switch (state) {
    case State.RUNNING:
      return 'ok';
    case State.STARTING:
    case State.WAITING:
      return 'warn';
    default:
      return 'bad';
  }
}

function fileStateClass(state: FileState): string {
  switch (state) {
    case FileState.NOT_RUNNING:
    case FileState.NO_CHANGE:
      return 'muted';
    default:
      return 'bad';
  }
}

function cell(text: string, className = ''): HTMLTableCellElement {
  const td = document.createElement('td');
  td.textContent = text;
  td.title = text;
  if (className) {
    td.className = className;
  }
  return td;
}

function pill(text: string, className: string): HTMLTableCellElement {
  const td = document.createElement('td');
  const span = document.createElement('span');
  span.className = `pill ${className}`;
  span.textContent = text;
  td.appendChild(span);
  return td;
}

/**
 * Starter's status for each node, as a table redrawn when a status comes; its columns have
 * fixed widths, so nothing moves when an app's state changes.
 */
export class StarterHandler {
  private readonly rows = new Map<string, Row[]>();
  private readonly statusList = document.getElementById(
    'status_list',
  ) as HTMLElement;
  private readonly empty = document.getElementById('empty') as HTMLElement;
  private drawPending = false;

  constructor(private readonly connection: Connection) {
    for (const node of NODES) {
      this.connection.addConfigHandler(() => {
        this.connection.addHandler(
          node + '/aos',
          'aos.starter.Status',
          (data) => this.handleStatus(data, node),
        );
      });
    }
  }

  private handleStatus(data: Uint8Array, node: string): void {
    const status = Status.getRootAsStatus(new ByteBuffer(data));
    const rows: Row[] = [];
    for (let i = 0; i < status.statusesLength(); i++) {
      const app = status.statuses(i)!;
      rows.push({
        name: app.name() ?? '',
        node,
        state: app.state(),
        lastExitCode: app.lastExitCode(),
        pid: app.pid(),
        lastStartSeconds: Number(app.lastStartTime() / 1000000000n),
        lastStopReason: app.lastStopReason(),
        fileState: app.fileState(),
      });
    }
    this.rows.set(node, rows);
    if (!this.drawPending) {
      this.drawPending = true;
      window.requestAnimationFrame(() => this.draw());
    }
  }

  private draw(): void {
    this.drawPending = false;
    const body = document.createDocumentFragment();
    for (const node of NODES) {
      for (const row of this.rows.get(node) ?? []) {
        const tr = document.createElement('tr');
        tr.append(
          cell(row.name),
          cell(row.node || '–', 'muted'),
          pill(State[row.state], stateClass(row.state)),
          cell(String(row.lastExitCode), 'number'),
          cell(row.pid ? String(row.pid) : '–', 'number'),
          cell(`${row.lastStartSeconds} s`, 'number'),
          cell(LastStopReason[row.lastStopReason], 'muted'),
          cell(FileState[row.fileState], fileStateClass(row.fileState)),
        );
        body.appendChild(tr);
      }
    }
    this.statusList.replaceChildren(body);
    this.empty.hidden = this.statusList.childElementCount > 0;
  }
}
