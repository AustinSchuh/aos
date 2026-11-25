# Robot logs

Getting from "the match just ended" to "debugging in Foxglove": list a robot's logs, bring the
ones you pick to a laptop, and open each match in the Foxglove desktop app, with no internet at
the venue. Logs are converted whole, never trimmed, so what the robot did before and after the
match is a scrub away.

The pieces:

| Piece | Where | What it does |
| --- | --- | --- |
| `/logs` | `aos/network/robot_server/logs.rs` | The routes the page uses, below. robot_server serves them on the robot; `logs_server` serves the same ones on a laptop. |
| `logs_server` | `aos/network/robot_server/logs_server_main.rs` | On a laptop: the page and `/logs` on `localhost`, over the logs it has downloaded. Downloads only the logs picked, and converts each one as soon as it's here. |
| `log_manager` | `frc/logs/log_manager*` | C++, run as a child process for anything that reads inside a log: finds its matches (from `/frc frc.JoystickState`), summarizes what went wrong, and converts it to MCAP with `aos/util:log_to_mcap_lib`. |
| The page | `frc/logs/www/` | One page for both. On the robot it's `/logs.html`, and the vision page's **Logs** tab. On a laptop it's `http://localhost:8080/`. |
| A Foxglove layout | `frc/logs/layouts/robot_logs.json` | Battery and brownouts, the match, and the robot's text errors. A robot's own layout adds its motors' currents against their limits. |

## Using it

On a laptop, once per event:

1. Trust the robot (`trust_orin.py`), as for the robot's other pages.
2. Install the Foxglove desktop app with its Standalone License, for offline use, and put the
   layout in its `standalone` folder so every log opens with it.

After a match:

```text
bazel run //aos/network/robot_server:logs_server -- \
    --robot=https://orin-1868-1.local --robot-ca=<the team CA's ca.crt>
```

Open <http://localhost:8080/>. Tick the robot's logs to bring over and press **Download
selected**. Each is downloaded (resuming where a dropped download stopped), moved into
`~/frc_logs` once complete, then converted. Each match gets an **Open in Foxglove** button,
which opens the MCAP in the desktop app just before the match starts. Everything stays on the
laptop with the robot off.

On the robot, the same page is at `https://<robot>/logs.html` and on the vision page's **Logs**
tab: download a log as stored, or convert one there while the robot is disabled.

## The `/logs` routes

```text
GET    /logs/info                  {"mode": "robot" | "laptop", "free_bytes", "robot", "enabled"}
GET    /logs/presets               the channel presets conversion offers
GET    /logs                       {"logs": [...]}: files, sizes, matches, MCAPs, jobs
GET    /logs/{log}/summary         log_manager's summary: matches, faults, errors, channels
GET    /logs/{log}/files/{*path}   one of the log's files, as stored, with Range
GET    /logs/{log}/download        the whole log, as stored, as one tar file
POST   /logs/{log}/convert         {"preset": "all"} or {"channels": ["<name> <type>"]}
DELETE /logs/{log}                 deletes the log, its summary and its MCAPs
GET    /logs/jobs, /logs/jobs/{id} conversions and downloads, and how far along they are
DELETE /logs/jobs/{id}             cancels one
GET    /logs/mcap/{file}           an MCAP, with Range and CORS, for Foxglove
GET    /logs/robot                 laptop: the robot's logs, marked with which are here
POST   /logs/robot/{log}/pull      laptop: downloads one of the robot's logs
```

The page only talks to the server it came from: on a laptop, `logs_server` talks to the robot.

## On the robot

robot_server serves `/logs` when given `--logs-dir`, the folder the logger writes into (`logs`,
beside the binaries, on the Orin). `log_manager` is deployed beside robot_server.

- **Converting only runs while the robot is disabled.** robot_server reads `/frc
  frc.JoystickState`; a request while enabled is refused, and a conversion stops if the robot is
  enabled. No JoystickState, or an old one, means robot code isn't running, so it's disabled.
- **It never gets in robot code's way.** `log_manager` runs at the lowest CPU priority, and idle
  I/O priority on Linux. Downloads of raw logs are limited to `--logs-max-download-rate` bytes a
  second (20 MB/s by default) so they can't starve the logger.
- **The log being written can't be deleted.**

MCAPs and summaries go in `logs_server/` in the log folder (`--logs-output-dir` to move them).

## Channels

Converting with the `all` preset writes every channel. Other presets, and picking channels by
hand on the page, write fewer, always with `/aos aos.logging.LogMessageFbs`, `/frc
frc.RobotState` and `/frc frc.JoystickState`, which the layout needs. The generic presets are
`all`, `power` (RobotState and every Position) and `errors` (starter status and every Status).
A robot replaces them with its own through `--logs-presets` (robot_server) or `--presets`
(`logs_server`):

```json
{"presets": [{"name": "superstructure", "description": "Everything on /superstructure.",
              "regexes": ["/superstructure .*"]}]}
```

## What the summary finds

`log_manager analyze` replays a log once and records:

- **Matches**, from JoystickState's match type, number, event and alliance, with when auto and
  teleop started and ended, on both clocks. `foxglove_time` is the time for Foxglove's `time=`:
  log_to_mcap stamps messages with the monotonic clock, so it's monotonic time as an epoch.
- **Enabled periods**, with or without a field.
- **Faults**: every `errors` vector of enums (CAN device and zeroing errors), every `estopped`
  that's true, in any Position or Status, with its CAN ID when the table has one; and brownouts.
- **Text errors**: `/aos` log messages at ERROR and FATAL.
- **The lowest battery voltage**, and when.
