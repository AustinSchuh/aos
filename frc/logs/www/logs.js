// The robot's logs: lists them, downloads the ones picked (on a laptop), converts them to
// MCAP, and opens them in the Foxglove desktop app.  Talks only to the server it came from,
// through /logs (aos/network/robot_server/logs.rs).
'use strict';

const POLL_MS = 2000;
// Foxglove opens this long before the match starts, so the lead-up is on screen.
const LEAD_SECONDS = 5;

const state = {
  mode: null,
  info: null,
  logs: [],
  robot: null,
  presets: [],
  selected: new Set(),
  // Which log's card is open, so a refresh keeps it.
  pickingFor: null,
};

const $ = (id) => document.getElementById(id);

/** An element with `props` set and `children` appended; strings become text. */
function el(tag, props = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(props)) {
    if (key === 'class') node.className = value;
    else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
    else if (value === true) node.setAttribute(key, '');
    else if (value !== false && value !== null && value !== undefined)
      node.setAttribute(key, value);
  }
  for (const child of children.flat()) {
    if (child === null || child === undefined || child === false) continue;
    node.append(
      typeof child === 'string' ? document.createTextNode(child) : child,
    );
  }
  return node;
}

async function api(path, options = {}) {
  const response = await fetch(path, {cache: 'no-store', ...options});
  const text = await response.text();
  let body = null;
  try {
    body = text ? JSON.parse(text) : null;
  } catch {
    body = {error: text};
  }
  if (!response.ok)
    throw new Error((body && body.error) || `${response.status} for ${path}`);
  return body;
}

function showError(message) {
  $('error').hidden = !message;
  $('error').textContent = message || '';
}

function bytes(n) {
  if (n === null || n === undefined) return '';
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  let value = n;
  let unit = 0;
  while (value >= 1000 && unit < units.length - 1) {
    value /= 1000;
    unit++;
  }
  return `${value.toFixed(value < 10 && unit > 0 ? 1 : 0)} ${units[unit]}`;
}

/** "Qual 12 · Red" for a match from a log's summary. */
function matchName(match) {
  const kinds = {
    practice: 'Practice',
    qualification: 'Qual',
    elimination: 'Elim',
  };
  const parts = [`${kinds[match.match_type] || 'Match'} ${match.match_number}`];
  if (match.alliance && match.alliance !== 'unknown') {
    parts.push(match.alliance[0].toUpperCase() + match.alliance.slice(1));
  }
  if (match.event_name) parts.push(match.event_name);
  return parts.join(' · ');
}

/** When a log started, from its summary, in local time. */
function startedAt(summary) {
  if (!summary || !summary.start || !summary.start.realtime) return '';
  const date = new Date(summary.start.realtime);
  if (Number.isNaN(date.getTime()) || date.getFullYear() < 2000) return '';
  return date.toLocaleString([], {
    month: 'short',
    day: 'numeric',
    hour: 'numeric',
    minute: '2-digit',
  });
}

/**
 * The Foxglove desktop app's link to open `mcap`, at `time` if given.  log_to_mcap stamps
 * messages with the robot's monotonic clock, which the summary's foxglove_time is in.
 */
function foxgloveLink(mcap, time) {
  const url = `${location.origin}/logs/mcap/${encodeURIComponent(mcap)}`;
  let link = `foxglove://open?ds=remote-file&ds.url=${encodeURIComponent(url)}`;
  if (time) link += `&time=${encodeURIComponent(time)}`;
  return link;
}

/** Just before `match` starts: auto if it ran, else when the field said which match. */
function matchTime(match) {
  const start = match.auto_start || match.start;
  if (!start || typeof start.monotonic_sec !== 'number') return null;
  return new Date(
    Math.max(0, start.monotonic_sec - LEAD_SECONDS) * 1000,
  ).toISOString();
}

/** The live Foxglove link: the robot's Foxglove server, through robot_server. */
function liveLink() {
  let robot =
    state.mode === 'robot' ? location.origin : state.info && state.info.robot;
  if (!robot) return null;
  const ws = robot.replace(/^https:/, 'wss:').replace(/^http:/, 'ws:');
  return `foxglove://open?ds=foxglove-websocket&ds.url=${encodeURIComponent(`${ws}/foxglove`)}`;
}

function jobLine(job) {
  const what = job.kind === 'pull' ? 'Downloading' : 'Converting';
  const phase = job.phase ? ` (${job.phase})` : '';
  if (job.state === 'queued')
    return el('div', {class: 'job'}, `${what}: waiting`);
  const children = [`${what}${phase}`];
  if (job.total_bytes > 0) {
    children.push(
      ' ',
      el('progress', {max: job.total_bytes, value: job.done_bytes}),
      ` ${bytes(job.done_bytes)} of ${bytes(job.total_bytes)}`,
    );
  }
  children.push(
    ' ',
    el('button', {type: 'button', onclick: () => cancelJob(job.id)}, 'Cancel'),
  );
  return el('div', {class: 'job'}, children);
}

// ---------------------------------------------------------------------------------------
// The robot's logs, on a laptop.

function renderRobot() {
  const section = $('robot-section');
  section.hidden = state.mode !== 'laptop';
  if (section.hidden) return;
  const robot = state.robot;
  const status = $('status');
  if (!robot || !robot.connected) {
    status.className = 'status';
    $('status-text').textContent =
      `Robot offline (${(state.info && state.info.robot) || ''})`;
  } else {
    status.className = 'status ok';
    $('status-text').textContent = `Robot connected: ${robot.robot}`;
  }
  const body = $('robot-logs');
  body.replaceChildren();
  const logs = (robot && robot.logs) || [];
  for (const name of [...state.selected]) {
    if (!logs.some((log) => log.name === name && !log.on_laptop))
      state.selected.delete(name);
  }
  if (logs.length === 0) {
    body.append(
      el(
        'tr',
        {},
        el(
          'td',
          {colspan: 5, class: 'muted'},
          robot && robot.connected
            ? 'The robot has no logs.'
            : (robot && robot.error) || 'Not connected.',
        ),
      ),
    );
  }
  for (const log of logs) {
    const matches = ((log.summary && log.summary.matches) || [])
      .map(matchName)
      .join(', ');
    const pulling = log.pulls && log.pulls.length > 0;
    const box = el('input', {
      type: 'checkbox',
      checked: state.selected.has(log.name),
      disabled: log.on_laptop || pulling,
      onchange: (event) => {
        if (event.target.checked) state.selected.add(log.name);
        else state.selected.delete(log.name);
        $('pull-selected').disabled = state.selected.size === 0;
      },
    });
    let where = log.on_laptop
      ? el('span', {class: 'badge'}, 'on laptop')
      : null;
    if (pulling) where = jobLine(log.pulls[0]);
    body.append(
      el(
        'tr',
        {},
        el('td', {}, box),
        el(
          'td',
          {},
          el('span', {class: 'log-name'}, log.name),
          ' ',
          log.current
            ? el('span', {class: 'badge live'}, 'being written')
            : null,
        ),
        el('td', {class: 'muted'}, matches || startedAt(log.summary)),
        el('td', {}, bytes(log.size)),
        el('td', {}, where),
      ),
    );
  }
  $('pull-selected').disabled = state.selected.size === 0;
}

async function pullSelected() {
  const names = [...state.selected];
  state.selected.clear();
  try {
    for (const name of names) {
      await api(`/logs/robot/${encodeURIComponent(name)}/pull`, {
        method: 'POST',
      });
    }
    showError('');
  } catch (error) {
    showError(`Couldn't start the download: ${error.message}`);
  }
  refresh();
}

// ---------------------------------------------------------------------------------------
// The logs here.

function renderLogs() {
  $('local-heading').textContent =
    state.mode === 'laptop' ? 'On this laptop' : 'On the robot';
  const container = $('logs');
  container.replaceChildren();
  $('no-logs').hidden = state.logs.length > 0;
  if (state.mode === 'laptop') {
    $('no-logs').textContent =
      'No logs on this laptop yet. Pick some from the robot above.';
  }
  for (const log of state.logs) container.append(renderLog(log));
}

function renderLog(log) {
  const summary = log.summary;
  const main = (log.mcaps || []).find(
    (mcap) => mcap.name === `${log.name}.mcap`,
  );
  const others = (log.mcaps || []).filter((mcap) => mcap !== main);
  const converting = (log.jobs || []).filter((job) => job.kind === 'convert');

  const facts = [];
  if (summary) {
    if (summary.min_battery_voltage) {
      const volts = summary.min_battery_voltage.volts;
      facts.push(
        el(
          'span',
          {class: volts < 7 ? 'bad' : ''},
          `min ${volts.toFixed(1)} V`,
        ),
      );
    }
    facts.push(
      el(
        'span',
        {class: summary.error_count ? 'bad' : ''},
        `${summary.error_count} error message${summary.error_count === 1 ? '' : 's'}`,
      ),
    );
    facts.push(
      el(
        'span',
        {class: summary.fault_count ? 'bad' : ''},
        `${summary.fault_count} fault${summary.fault_count === 1 ? '' : 's'}`,
      ),
    );
  } else if (converting.length === 0) {
    facts.push(el('span', {}, 'Not converted yet'));
  }

  const matches =
    summary && summary.matches && summary.matches.length > 0
      ? el(
          'div',
          {class: 'matches'},
          summary.matches.map((match) =>
            el(
              'div',
              {class: 'match'},
              el('span', {class: 'match-label'}, matchName(match)),
              main
                ? el(
                    'a',
                    {
                      class: 'button primary',
                      href: foxgloveLink(main.name, matchTime(match)),
                    },
                    'Open in Foxglove',
                  )
                : null,
            ),
          ),
        )
      : null;

  const presetPicker = el(
    'select',
    {'aria-label': 'Channels to convert'},
    state.presets.map((preset) =>
      el(
        'option',
        {value: preset.name, title: preset.description || ''},
        preset.name === 'all' ? 'All channels' : preset.name,
      ),
    ),
  );
  const canConvert = !log.current || state.mode === 'robot';
  const robotEnabled =
    state.mode === 'robot' && state.info && state.info.enabled;

  const actions = el(
    'div',
    {class: 'actions'},
    main && !matches
      ? el(
          'a',
          {class: 'button primary', href: foxgloveLink(main.name)},
          'Open in Foxglove',
        )
      : null,
    main
      ? el(
          'a',
          {
            class: 'button',
            href: `/logs/mcap/${encodeURIComponent(main.name)}`,
            download: main.name,
          },
          `Download MCAP (${bytes(main.size)})`,
        )
      : null,
    others.map((mcap) =>
      el(
        'a',
        {class: 'button', href: foxgloveLink(mcap.name), title: mcap.name},
        `Open ${mcap.name.slice(log.name.length + 1, -'.mcap'.length)}`,
      ),
    ),
    el(
      'a',
      {class: 'button', href: `/logs/${encodeURIComponent(log.name)}/download`},
      `Download raw (${bytes(log.size)})`,
    ),
    canConvert ? presetPicker : null,
    canConvert
      ? el(
          'button',
          {
            type: 'button',
            disabled: robotEnabled,
            title: robotEnabled ? 'The robot is enabled' : '',
            onclick: () => convert(log.name, {preset: presetPicker.value}),
          },
          state.mode === 'robot' ? 'Convert on robot' : 'Convert',
        )
      : null,
    canConvert
      ? el(
          'button',
          {
            type: 'button',
            disabled: robotEnabled || !summary,
            title: summary ? '' : 'Convert it once first, to find its channels',
            onclick: () => pickChannels(log.name),
          },
          'Pick channels…',
        )
      : null,
    el(
      'button',
      {
        type: 'button',
        class: 'danger',
        disabled: log.current,
        onclick: () => deleteLog(log.name),
      },
      'Delete',
    ),
  );

  return el(
    'div',
    {class: 'log'},
    el(
      'div',
      {class: 'log-head'},
      el('span', {class: 'log-name'}, log.name),
      log.current ? el('span', {class: 'badge live'}, 'being written') : null,
      el('span', {class: 'muted'}, startedAt(summary)),
      el('span', {class: 'muted'}, bytes(log.size)),
    ),
    el('div', {class: 'facts'}, facts),
    matches,
    actions,
    (log.jobs || []).map(jobLine),
  );
}

async function convert(name, request) {
  try {
    await api(`/logs/${encodeURIComponent(name)}/convert`, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(request),
    });
    showError('');
  } catch (error) {
    showError(`Couldn't convert ${name}: ${error.message}`);
  }
  refresh();
}

async function deleteLog(name) {
  const where = state.mode === 'laptop' ? 'this laptop' : 'the robot';
  if (!confirm(`Delete ${name} from ${where}? This can't be undone.`)) return;
  try {
    await api(`/logs/${encodeURIComponent(name)}`, {method: 'DELETE'});
    showError('');
  } catch (error) {
    showError(`Couldn't delete ${name}: ${error.message}`);
  }
  refresh();
}

async function cancelJob(id) {
  try {
    await api(`/logs/jobs/${id}`, {method: 'DELETE'});
  } catch (error) {
    showError(`Couldn't cancel: ${error.message}`);
  }
  refresh();
}

async function pickChannels(name) {
  let summary;
  try {
    summary = await api(`/logs/${encodeURIComponent(name)}/summary`);
  } catch (error) {
    showError(error.message);
    return;
  }
  const dialog = $('channels-dialog');
  const list = $('channel-list');
  const filter = $('channel-filter');
  const channels = (summary.channels || []).map((channel) => ({
    key: `${channel.name} ${channel.type}`,
    count: channel.count,
  }));
  const render = () => {
    const wanted = filter.value.toLowerCase();
    for (const label of list.children) {
      label.hidden = !label.dataset.key.toLowerCase().includes(wanted);
    }
  };
  list.replaceChildren(
    ...channels.map((channel) => {
      const label = el(
        'label',
        {},
        el('input', {type: 'checkbox', value: channel.key}),
        ' ',
        channel.key,
        ' ',
        el('span', {class: 'muted'}, `(${channel.count})`),
      );
      label.dataset.key = channel.key;
      return label;
    }),
  );
  filter.value = '';
  filter.oninput = render;
  state.pickingFor = name;
  dialog.onclose = () => {
    if (dialog.returnValue !== 'convert') return;
    const picked = [...list.querySelectorAll('input:checked')].map(
      (box) => box.value,
    );
    if (picked.length > 0) convert(name, {channels: picked});
  };
  dialog.showModal();
}

// ---------------------------------------------------------------------------------------

async function refresh() {
  try {
    const [info, list] = await Promise.all([api('/logs/info'), api('/logs')]);
    state.info = info;
    state.mode = info.mode;
    state.logs = list.logs || [];
    if (state.mode === 'laptop') {
      state.robot = await api('/logs/robot').catch((error) => ({
        connected: false,
        error: error.message,
      }));
    } else {
      const status = $('status');
      status.className = info.enabled ? 'status warn' : 'status ok';
      $('status-text').textContent = info.enabled
        ? 'Robot enabled: converting waits'
        : 'Robot disabled';
    }
    $('free').textContent = info.free_bytes
      ? `${bytes(info.free_bytes)} free`
      : '';
    const live = liveLink();
    $('live').hidden = !live;
    if (live) $('live').href = live;
    // Don't redraw under an open dialog or a dropdown being used.
    if (
      !$('channels-dialog').open &&
      !(document.activeElement && document.activeElement.tagName === 'SELECT')
    ) {
      renderRobot();
      renderLogs();
    }
    if ($('error').textContent.startsWith('Lost the server')) showError('');
  } catch (error) {
    $('status').className = 'status';
    $('status-text').textContent = 'Not connected';
    showError(`Lost the server: ${error.message}`);
  }
}

async function start() {
  $('pull-selected').addEventListener('click', pullSelected);
  try {
    const presets = await api('/logs/presets');
    state.presets = presets.presets || [];
  } catch {
    state.presets = [{name: 'all'}];
  }
  await refresh();
  setInterval(() => {
    if (!document.hidden) refresh();
  }, POLL_MS);
}

start();
