//! `/logs`: the robot's logs, for people debugging a match.  See `frc/logs/README.md`.
//!
//! The same routes run in two places:
//!
//! * **On the robot**, in robot_server, over its log folder.  Converting a log to MCAP only
//!   runs while the robot is disabled, at low priority, and stops if the robot is enabled.
//! * **On a laptop**, in `logs_server`, over the logs it has downloaded from a robot, so they
//!   can be looked at with the robot off.  It adds two routes, to list the robot's logs and to
//!   download the ones the user picks.  A log is converted to MCAP as soon as it's downloaded.
//!
//! ```text
//! GET    /logs/info                  {"mode": "robot" | "laptop", "free_bytes", "robot"}
//! GET    /logs/presets               the channel presets conversion offers
//! GET    /logs                       {"logs": [...]}: files, sizes, matches, MCAPs, jobs
//! GET    /logs/{log}/summary         log_manager's summary: matches, faults, errors
//! GET    /logs/{log}/files/{*path}   one of the log's files, as stored, with Range
//! GET    /logs/{log}/download        the whole log, as stored, as one tar file
//! POST   /logs/{log}/convert         {"preset": "all"} or {"channels": ["<name> <type>"]}
//! DELETE /logs/{log}                 deletes the log, its summary and its MCAPs
//! GET    /logs/jobs, /logs/jobs/{id} conversions and downloads, and how far along they are
//! DELETE /logs/jobs/{id}             cancels one
//! GET    /logs/mcap/{file}           an MCAP, with Range and CORS, for Foxglove
//! GET    /logs/robot                 laptop: the robot's logs, marked with which are here
//! POST   /logs/robot/{log}/pull      laptop: downloads one of the robot's logs
//! ```
//!
//! Reading inside a log, to find its matches and faults or to write an MCAP, is C++
//! (`frc/logs:log_manager`), run as a child process so that a corrupt log can't take the
//! server down, and so a job can be stopped.

use std::{
    collections::VecDeque,
    ffi::CString,
    fs,
    io::{self, Read},
    path::{Component, Path, PathBuf},
    process::{Command, Stdio},
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

use axum::{
    body::{Body, Bytes},
    extract::{Path as UrlPath, State},
    http::{header, HeaderMap, HeaderValue, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Router,
};
use rustls::{pki_types::ServerName, ClientConfig, RootCertStore};
use rustls_pki_types::{pem::PemObject, CertificateDer};
use serde_json::{json, Value};
use tokio::{
    io::{AsyncBufReadExt, AsyncRead, AsyncReadExt, AsyncSeekExt, AsyncWriteExt, BufReader},
    net::TcpStream,
    sync::{mpsc, oneshot, Notify},
};
use tokio_rustls::TlsConnector;

use crate::api::{ApiRequest, LatestError};

/// The symlink the logger keeps pointing at the log it's writing.
const CURRENT_LINK: &str = "fbs_log-current";
/// The channel that says whether the robot is enabled.
const JOYSTICK_STATE: (&str, &str) = ("/frc", "frc.JoystickState");
/// A JoystickState older than this means robot code isn't running, so the robot is disabled.
const JOYSTICK_STATE_MAX_AGE: Duration = Duration::from_secs(2);
/// How often a conversion on the robot checks that the robot is still disabled.
const ENABLED_POLL: Duration = Duration::from_millis(500);
/// How many finished jobs are remembered for the page.
const KEPT_JOBS: usize = 100;
/// How much of a file is read or sent at once.
const CHUNK: usize = 64 * 1024;
/// How long the laptop waits on the robot before giving up.
const ROBOT_TIMEOUT: Duration = Duration::from_secs(10);

/// Where `/logs` runs.
#[derive(Clone, Debug)]
pub enum Mode {
    /// On the robot.  `api` reads JoystickState, so conversion only runs while disabled; with
    /// none, the robot is taken to be disabled.
    Robot {
        api: Option<mpsc::UnboundedSender<ApiRequest>>,
    },
    /// On a laptop, downloading from the robot at `robot`.
    Laptop { robot: RobotClient },
}

/// Where `/logs` finds the logs, and what it runs.
#[derive(Clone, Debug)]
pub struct LogsOptions {
    /// The folder the logger writes `fbs_log-*` folders into, or a laptop keeps them in.
    pub log_dir: PathBuf,
    /// Where MCAPs (`mcap/`), summaries (`summaries/`) and downloads in progress
    /// (`incoming/`) go.
    pub output_dir: PathBuf,
    /// `frc/logs:log_manager`.
    pub log_manager: PathBuf,
    /// A robot's own channel presets for `log_manager --presets`, if it has any.
    pub presets: Option<PathBuf>,
    /// The most bytes a second one download of a raw log may take; 0 for no limit.  On the
    /// robot, this keeps downloads from starving the logger.
    pub max_download_rate: u64,
    pub mode: Mode,
}

impl LogsOptions {
    /// Options with everything in `output_dir` under `log_dir`.
    pub fn new(log_dir: PathBuf, log_manager: PathBuf, mode: Mode) -> Self {
        Self {
            output_dir: log_dir.join("logs_server"),
            log_dir,
            log_manager,
            presets: None,
            max_download_rate: 0,
            mode,
        }
    }

    fn mcap_dir(&self) -> PathBuf {
        self.output_dir.join("mcap")
    }

    fn summary_path(&self, log: &str) -> PathBuf {
        self.output_dir
            .join("summaries")
            .join(format!("{log}.json"))
    }

    fn incoming_dir(&self) -> PathBuf {
        self.output_dir.join("incoming")
    }
}

/// True if `name` is a log folder's name, as the logger makes them: `fbs_log-NNN_<date>`.
/// Everything a request names is checked with this before it's used as a path.
pub fn is_log_name(name: &str) -> bool {
    let Some(rest) = name.strip_prefix("fbs_log-") else {
        return false;
    };
    let digits = rest.bytes().take_while(u8::is_ascii_digit).count();
    let Some(tail) = rest[digits..].strip_prefix('_') else {
        return false;
    };
    digits > 0
        && !tail.is_empty()
        && tail
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || b"_.:-".contains(&c))
}

/// The index in a log's name, which orders logs: `fbs_log-012_...` is 12.
fn log_index(name: &str) -> u64 {
    name.trim_start_matches("fbs_log-")
        .split('_')
        .next()
        .and_then(|digits| digits.parse().ok())
        .unwrap_or(0)
}

// ------------------------------------------------------------------------------------------
// Jobs: conversions, and downloads from the robot.

/// What a job does.
#[derive(Clone, Debug, PartialEq)]
pub enum JobKind {
    /// Summarizes the log, then writes an MCAP of `preset`'s channels, or of exactly
    /// `channels` (`"<name> <type>"`) if there are any.
    Convert {
        preset: String,
        channels: Vec<String>,
    },
    /// Downloads the log from the robot, then converts it with the "all" preset.
    Pull,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JobState {
    Queued,
    Running,
    Done,
    Failed,
    Cancelled,
}

impl JobState {
    fn name(self) -> &'static str {
        match self {
            JobState::Queued => "queued",
            JobState::Running => "running",
            JobState::Done => "done",
            JobState::Failed => "failed",
            JobState::Cancelled => "cancelled",
        }
    }

    fn finished(self) -> bool {
        matches!(
            self,
            JobState::Done | JobState::Failed | JobState::Cancelled
        )
    }
}

#[derive(Clone, Debug)]
struct Job {
    id: u64,
    log: String,
    kind: JobKind,
    state: JobState,
    /// What it's doing now, for the page: "analyzing", "converting", "downloading".
    phase: String,
    error: Option<String>,
    /// The MCAP it wrote, as `/logs/mcap/` names it.
    mcap: Option<String>,
    done_bytes: Arc<AtomicU64>,
    total_bytes: u64,
    cancel: Arc<AtomicBool>,
}

impl Job {
    fn to_json(&self) -> Value {
        let (kind, preset, channels) = match &self.kind {
            JobKind::Convert { preset, channels } => ("convert", json!(preset), json!(channels)),
            JobKind::Pull => ("pull", Value::Null, Value::Null),
        };
        json!({
            "id": self.id,
            "log": self.log,
            "kind": kind,
            "preset": preset,
            "channels": channels,
            "state": self.state.name(),
            "phase": self.phase,
            "error": self.error,
            "mcap": self.mcap,
            "done_bytes": self.done_bytes.load(Ordering::Relaxed),
            "total_bytes": self.total_bytes,
        })
    }
}

#[derive(Default)]
struct Jobs {
    next_id: u64,
    jobs: VecDeque<Job>,
}

impl Jobs {
    fn get_mut(&mut self, id: u64) -> Option<&mut Job> {
        self.jobs.iter_mut().find(|job| job.id == id)
    }

    /// Adds a job, unless the same one is already waiting or running, whose ID it returns.
    fn add(&mut self, log: &str, kind: JobKind) -> u64 {
        if let Some(job) = self
            .jobs
            .iter()
            .find(|job| job.log == log && job.kind == kind && !job.state.finished())
        {
            return job.id;
        }
        self.next_id += 1;
        let id = self.next_id;
        self.jobs.push_back(Job {
            id,
            log: log.to_string(),
            kind,
            state: JobState::Queued,
            phase: String::new(),
            error: None,
            mcap: None,
            done_bytes: Arc::new(AtomicU64::new(0)),
            total_bytes: 0,
            cancel: Arc::new(AtomicBool::new(false)),
        });
        // Forget the oldest finished jobs.
        while self.jobs.len() > KEPT_JOBS {
            match self.jobs.iter().position(|job| job.state.finished()) {
                Some(index) => {
                    self.jobs.remove(index);
                }
                None => break,
            }
        }
        id
    }

    /// Starts the oldest queued job, returning it.
    fn start_next(&mut self) -> Option<Job> {
        let job = self
            .jobs
            .iter_mut()
            .find(|job| job.state == JobState::Queued)?;
        job.state = JobState::Running;
        Some(job.clone())
    }
}

/// `/logs`'s state, shared by its handlers and its job runner.
pub struct Logs {
    options: LogsOptions,
    jobs: Mutex<Jobs>,
    wake: Notify,
}

impl Logs {
    pub fn new(options: LogsOptions) -> Arc<Self> {
        Arc::new(Self {
            options,
            jobs: Mutex::new(Jobs::default()),
            wake: Notify::new(),
        })
    }

    pub fn options(&self) -> &LogsOptions {
        &self.options
    }

    fn add_job(&self, log: &str, kind: JobKind) -> u64 {
        let id = self.jobs.lock().unwrap().add(log, kind);
        self.wake.notify_one();
        id
    }

    fn update(&self, id: u64, f: impl FnOnce(&mut Job)) {
        if let Some(job) = self.jobs.lock().unwrap().get_mut(id) {
            f(job);
        }
    }

    fn job_json(&self, id: u64) -> Option<Value> {
        self.jobs
            .lock()
            .unwrap()
            .jobs
            .iter()
            .find(|job| job.id == id)
            .map(Job::to_json)
    }

    fn jobs_json(&self) -> Value {
        Value::from(
            self.jobs
                .lock()
                .unwrap()
                .jobs
                .iter()
                .map(Job::to_json)
                .collect::<Vec<_>>(),
        )
    }

    /// The unfinished jobs on `log`, for the list of logs.
    fn active_jobs(&self, log: &str) -> Vec<Value> {
        self.jobs
            .lock()
            .unwrap()
            .jobs
            .iter()
            .filter(|job| job.log == log && !job.state.finished())
            .map(Job::to_json)
            .collect()
    }

    /// Whether the robot is enabled now: see [`robot_enabled`].
    pub async fn robot_enabled(&self) -> bool {
        robot_enabled(&self.options.mode).await
    }
}

/// Whether the robot is enabled now.  On a laptop, never.  On a robot whose code isn't
/// running (no JoystickState, or an old one), it isn't.
pub async fn robot_enabled(mode: &Mode) -> bool {
    {
        let Mode::Robot { api: Some(api) } = mode else {
            return false;
        };
        let (reply, answer) = oneshot::channel();
        let request = ApiRequest::Latest {
            name: JOYSTICK_STATE.0.to_string(),
            type_: JOYSTICK_STATE.1.to_string(),
            max_vector_size: 0,
            reply,
        };
        if api.send(request).is_err() {
            return false;
        }
        match answer.await {
            Ok(Ok(latest)) => {
                latest.age_ns <= JOYSTICK_STATE_MAX_AGE.as_nanos() as i64
                    && serde_json::from_str::<Value>(&latest.json)
                        .map(|state| state["enabled"] == json!(true))
                        .unwrap_or(false)
            }
            Ok(Err(LatestError::NoChannel | LatestError::NoMessage)) => false,
            // Being unable to tell is treated as enabled, to be safe.
            Ok(Err(_)) => true,
            Err(_) => false,
        }
    }
}

/// Runs `logs`'s jobs, one at a time, oldest first, until the process ends.
pub async fn run_jobs(logs: Arc<Logs>) {
    loop {
        // Made before looking, so a job added in between still wakes it.
        let woken = logs.wake.notified();
        let next = logs.jobs.lock().unwrap().start_next();
        let Some(job) = next else {
            woken.await;
            continue;
        };
        let result = match &job.kind {
            JobKind::Convert { preset, channels } => convert(&logs, &job, preset, channels).await,
            JobKind::Pull => pull(&logs, &job).await,
        };
        let cancelled = job.cancel.load(Ordering::Relaxed);
        if let (Err(error), false) = (&result, cancelled) {
            log::warn!("/logs job {} on {} failed: {error}", job.id, job.log);
        }
        logs.update(job.id, |job| {
            job.phase.clear();
            match result {
                Ok(()) => job.state = JobState::Done,
                Err(_) if cancelled => job.state = JobState::Cancelled,
                Err(error) => {
                    job.state = JobState::Failed;
                    job.error = Some(error);
                }
            }
        });
    }
}

/// The MCAP file name for converting `log` with `preset` or `channels`.
fn mcap_name(log: &str, preset: &str, channels: &[String]) -> String {
    if channels.is_empty() {
        if preset == "all" {
            format!("{log}.mcap")
        } else {
            format!("{log}.{preset}.mcap")
        }
    } else {
        // FNV-1a, so the same pick gets the same file in every run.
        let mut hash: u64 = 0xcbf29ce484222325;
        for byte in channels.join("\n").bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
        format!("{log}.custom-{hash:016x}.mcap")
    }
}

/// Summarizes `job`'s log if it hasn't been, then writes its MCAP.
async fn convert(logs: &Logs, job: &Job, preset: &str, channels: &[String]) -> Result<(), String> {
    let options = &logs.options;
    let log_path = options.log_dir.join(&job.log);
    if !log_path.is_dir() {
        return Err(format!("There's no log {}", job.log));
    }
    let robot = matches!(options.mode, Mode::Robot { .. });
    if robot && logs.robot_enabled().await {
        return Err("The robot is enabled; convert once it's disabled".to_string());
    }
    for dir in [options.mcap_dir(), options.output_dir.join("summaries")] {
        fs::create_dir_all(&dir)
            .map_err(|error| format!("Can't make {}: {error}", dir.display()))?;
    }

    // Stop if the robot is enabled meanwhile.
    let (finished, mut finished_rx) = oneshot::channel::<()>();
    if robot {
        let cancel = job.cancel.clone();
        let mode = options.mode.clone();
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = &mut finished_rx => return,
                    _ = tokio::time::sleep(ENABLED_POLL) => {}
                }
                if robot_enabled(&mode).await {
                    log::info!("The robot was enabled; stopping a conversion");
                    cancel.store(true, Ordering::Relaxed);
                    return;
                }
            }
        });
    }

    let mut base = vec![];
    if let Some(presets) = &options.presets {
        base.push(format!("--presets={}", presets.display()));
    }
    let summary = options.summary_path(&job.log);
    let result = async {
        if !summary.exists() {
            logs.update(job.id, |job| job.phase = "analyzing".to_string());
            let mut args = base.clone();
            args.extend([
                "analyze".to_string(),
                format!("--output={}", summary.display()),
                log_path.display().to_string(),
            ]);
            run_log_manager(&options.log_manager, args, job.cancel.clone(), robot).await?;
        }
        let name = mcap_name(&job.log, preset, channels);
        let output = options.mcap_dir().join(&name);
        if !output.exists() {
            logs.update(job.id, |job| job.phase = "converting".to_string());
            let mut args = base.clone();
            args.push("convert".to_string());
            if channels.is_empty() {
                args.push(format!("--preset={preset}"));
            } else {
                args.push(format!("--channels={}", channels.join(";")));
            }
            args.extend([log_path.display().to_string(), output.display().to_string()]);
            run_log_manager(&options.log_manager, args, job.cancel.clone(), robot).await?;
        }
        logs.update(job.id, |job| job.mcap = Some(name));
        Ok(())
    }
    .await;
    let _ = finished.send(());
    result
}

/// Runs log_manager with `args` on a blocking thread, killing it if `cancel` is set.  Returns
/// what it printed, or why it failed.
async fn run_log_manager(
    program: &Path,
    args: Vec<String>,
    cancel: Arc<AtomicBool>,
    low_priority: bool,
) -> Result<String, String> {
    let program = program.to_path_buf();
    tokio::task::spawn_blocking(move || run_child(&program, &args, &cancel, low_priority))
        .await
        .map_err(|error| format!("log_manager's thread failed: {error}"))?
}

fn run_child(
    program: &Path,
    args: &[String],
    cancel: &AtomicBool,
    low_priority: bool,
) -> Result<String, String> {
    let mut command = Command::new(program);
    command
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    if low_priority {
        use std::os::unix::process::CommandExt;
        // SAFETY: only async-signal-safe syscalls between fork and exec.
        unsafe {
            command.pre_exec(|| {
                lower_priority();
                Ok(())
            });
        }
    }
    let mut child = command
        .spawn()
        .map_err(|error| format!("Can't run {}: {error}", program.display()))?;
    // Read both pipes while it runs, so a full one can't stall it.
    let drain = |pipe: Option<Box<dyn Read + Send>>| {
        std::thread::spawn(move || {
            let mut text = String::new();
            if let Some(mut pipe) = pipe {
                let _ = pipe.read_to_string(&mut text);
            }
            text
        })
    };
    let stdout = drain(child.stdout.take().map(|p| Box::new(p) as _));
    let stderr = drain(child.stderr.take().map(|p| Box::new(p) as _));
    let status = loop {
        if cancel.load(Ordering::Relaxed) {
            let _ = child.kill();
            let _ = child.wait();
            return Err("Cancelled".to_string());
        }
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) => std::thread::sleep(Duration::from_millis(100)),
            Err(error) => return Err(format!("Lost log_manager: {error}")),
        }
    };
    let stdout = stdout.join().unwrap_or_default();
    let stderr = stderr.join().unwrap_or_default();
    if status.success() {
        Ok(stdout)
    } else {
        // The end of what it said is where it says why.
        let tail: Vec<&str> = stderr.lines().rev().take(5).collect();
        let tail: Vec<&str> = tail.into_iter().rev().collect();
        Err(format!(
            "log_manager failed ({status}): {}",
            tail.join("\n")
        ))
    }
}

/// Lowest CPU priority, and idle I/O priority on Linux, so log work can't slow robot code or
/// the logger down.
fn lower_priority() {
    // SAFETY: plain syscalls on this process.
    unsafe {
        libc::setpriority(libc::PRIO_PROCESS, 0, 19);
        #[cfg(target_os = "linux")]
        {
            const IOPRIO_WHO_PROCESS: libc::c_long = 1;
            const IOPRIO_CLASS_IDLE: libc::c_long = 3;
            libc::syscall(
                libc::SYS_ioprio_set,
                IOPRIO_WHO_PROCESS,
                0 as libc::c_long,
                IOPRIO_CLASS_IDLE << 13,
            );
        }
    }
}

// ------------------------------------------------------------------------------------------
// Listing logs.

/// A file in a log, relative to the log's folder.
#[derive(Clone, Debug, PartialEq)]
pub struct LogFile {
    pub path: String,
    pub size: u64,
}

/// The files in `dir`, recursively, without following symlinks, sorted.
pub fn log_files(dir: &Path) -> io::Result<Vec<LogFile>> {
    let mut files = Vec::new();
    let mut pending = vec![PathBuf::new()];
    while let Some(relative) = pending.pop() {
        for entry in fs::read_dir(dir.join(&relative))? {
            let entry = entry?;
            let kind = entry.file_type()?;
            let path = relative.join(entry.file_name());
            if kind.is_dir() {
                pending.push(path);
            } else if kind.is_file() {
                let Some(path) = path.to_str() else { continue };
                files.push(LogFile {
                    path: path.replace('\\', "/"),
                    size: entry.metadata()?.len(),
                });
            }
        }
    }
    files.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(files)
}

/// The log the logger is writing, if any.
fn current_log(log_dir: &Path) -> Option<String> {
    let target = fs::read_link(log_dir.join(CURRENT_LINK)).ok()?;
    let name = target
        .components()
        .filter_map(|component| match component {
            Component::Normal(name) => name.to_str(),
            _ => None,
        })
        .next_back()?;
    Some(name.to_string())
}

/// What the list says about a log's summary: its matches, and how much went wrong.
fn summary_brief(summary: &Value) -> Value {
    let matches: Vec<Value> = summary["matches"]
        .as_array()
        .map(|matches| {
            matches
                .iter()
                .map(|m| {
                    json!({
                        "label": m["label"],
                        "match_type": m["match_type"],
                        "match_number": m["match_number"],
                        "alliance": m["alliance"],
                        "event_name": m["event_name"],
                        "start": m["start"],
                        "auto_start": m["auto_start"],
                        "teleop_start": m["teleop_start"],
                        "end": m["end"],
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    let count = |key: &str| summary[key].as_array().map_or(0, Vec::len);
    json!({
        "start": summary["start"],
        "end": summary["end"],
        "matches": matches,
        "fault_count": count("faults"),
        "error_count": count("log_messages"),
        "min_battery_voltage": summary["min_battery_voltage"],
    })
}

/// Every log in `options.log_dir`, newest first, with its files, summary and MCAPs.
fn list_logs(options: &LogsOptions) -> io::Result<Vec<Value>> {
    let current = current_log(&options.log_dir);
    let mut names = Vec::new();
    for entry in fs::read_dir(&options.log_dir)? {
        let entry = entry?;
        let Ok(name) = entry.file_name().into_string() else {
            continue;
        };
        if is_log_name(&name) && entry.file_type()?.is_dir() {
            names.push(name);
        }
    }
    names.sort_by(|a, b| log_index(b).cmp(&log_index(a)).then(b.cmp(a)));

    let mcaps: Vec<(String, u64)> = fs::read_dir(options.mcap_dir())
        .map(|entries| {
            entries
                .filter_map(Result::ok)
                .filter_map(|entry| {
                    let name = entry.file_name().into_string().ok()?;
                    let size = entry.metadata().ok()?.len();
                    name.ends_with(".mcap").then_some((name, size))
                })
                .collect()
        })
        .unwrap_or_default();

    let mut logs = Vec::new();
    for name in names {
        let files = log_files(&options.log_dir.join(&name))?;
        let size: u64 = files.iter().map(|file| file.size).sum();
        let summary = fs::read(options.summary_path(&name))
            .ok()
            .and_then(|bytes| serde_json::from_slice::<Value>(&bytes).ok())
            .map(|summary| summary_brief(&summary));
        let prefix = format!("{name}.");
        let log_mcaps: Vec<Value> = mcaps
            .iter()
            .filter(|(mcap, _)| mcap.starts_with(&prefix))
            .map(|(mcap, size)| json!({"name": mcap, "size": size}))
            .collect();
        logs.push(json!({
            "name": name,
            "size": size,
            "current": current.as_deref() == Some(name.as_str()),
            "files": files
                .iter()
                .map(|file| json!({"path": file.path, "size": file.size}))
                .collect::<Vec<_>>(),
            "summary": summary,
            "mcaps": log_mcaps,
        }));
    }
    Ok(logs)
}

/// Bytes free where the logs are.
#[allow(clippy::unnecessary_cast)]
fn free_bytes(path: &Path) -> Option<u64> {
    use std::os::unix::ffi::OsStrExt;
    let path = CString::new(path.as_os_str().as_bytes()).ok()?;
    // SAFETY: statvfs writes only into `stats`, and `path` is a C string.
    unsafe {
        let mut stats: libc::statvfs = std::mem::zeroed();
        if libc::statvfs(path.as_ptr(), &mut stats) != 0 {
            return None;
        }
        Some(stats.f_bavail as u64 * stats.f_frsize as u64)
    }
}

// ------------------------------------------------------------------------------------------
// Serving files.

/// A byte range of a file `len` long, from a `Range` header.  `Ok(None)` is the whole file;
/// `Err` is a range the file doesn't have.  Only single ranges are supported.
pub fn parse_range(range: Option<&str>, len: u64) -> Result<Option<(u64, u64)>, ()> {
    let Some(range) = range else {
        return Ok(None);
    };
    let Some(spec) = range.trim().strip_prefix("bytes=") else {
        return Ok(None);
    };
    if spec.contains(',') {
        // Several ranges: send the whole file, which is allowed.
        return Ok(None);
    }
    let Some((start, end)) = spec.split_once('-') else {
        return Err(());
    };
    let (start, end) = (start.trim(), end.trim());
    let (start, end) = if start.is_empty() {
        // The last `end` bytes.
        let suffix: u64 = end.parse().map_err(|_| ())?;
        if suffix == 0 {
            return Err(());
        }
        (len.saturating_sub(suffix), len.saturating_sub(1))
    } else {
        let start: u64 = start.parse().map_err(|_| ())?;
        let end = if end.is_empty() {
            len.saturating_sub(1)
        } else {
            end.parse::<u64>()
                .map_err(|_| ())?
                .min(len.saturating_sub(1))
        };
        (start, end)
    };
    if len == 0 || start > end || start >= len {
        return Err(());
    }
    Ok(Some((start, end)))
}

/// What Foxglove, served from elsewhere, needs to read MCAPs from here.
fn add_cors(headers: &mut HeaderMap) {
    for (name, value) in [
        ("access-control-allow-origin", "*"),
        ("access-control-allow-methods", "GET, HEAD, OPTIONS"),
        ("access-control-allow-headers", "Range"),
        (
            "access-control-expose-headers",
            "Content-Range, Content-Length, Accept-Ranges",
        ),
    ] {
        headers.insert(name, HeaderValue::from_static(value));
    }
}

/// A stream of `len` bytes of `file`, at no more than `rate` bytes a second (0 for no limit).
fn file_stream(
    file: tokio::fs::File,
    len: u64,
    rate: u64,
) -> impl futures::Stream<Item = io::Result<Bytes>> + Send {
    futures::stream::unfold((file, len), move |(mut file, left)| async move {
        if left == 0 {
            return None;
        }
        let mut buffer = vec![0u8; CHUNK.min(left as usize)];
        match file.read(&mut buffer).await {
            Ok(0) => Some((
                Err(io::Error::new(io::ErrorKind::UnexpectedEof, "File shrank")),
                (file, 0),
            )),
            Ok(read) => {
                buffer.truncate(read);
                if rate > 0 {
                    tokio::time::sleep(Duration::from_secs_f64(read as f64 / rate as f64)).await;
                }
                Some((Ok(Bytes::from(buffer)), (file, left - read as u64)))
            }
            Err(error) => Some((Err(error), (file, 0))),
        }
    })
}

/// Serves `path` with Range support, as `content_type`.
async fn serve_file(
    path: &Path,
    request: &HeaderMap,
    content_type: &'static str,
    rate: u64,
    cors: bool,
    download_name: Option<String>,
) -> Response {
    let mut file = match tokio::fs::File::open(path).await {
        Ok(file) => file,
        Err(_) => return error(StatusCode::NOT_FOUND, "No such file"),
    };
    let len = match file.metadata().await {
        Ok(metadata) if metadata.is_file() => metadata.len(),
        _ => return error(StatusCode::NOT_FOUND, "No such file"),
    };
    let range = request
        .get(header::RANGE)
        .and_then(|value| value.to_str().ok());
    let (status, start, count) = match parse_range(range, len) {
        Ok(None) => (StatusCode::OK, 0, len),
        Ok(Some((start, end))) => (StatusCode::PARTIAL_CONTENT, start, end - start + 1),
        Err(()) => {
            let mut response = StatusCode::RANGE_NOT_SATISFIABLE.into_response();
            response.headers_mut().insert(
                header::CONTENT_RANGE,
                HeaderValue::from_str(&format!("bytes */{len}")).unwrap(),
            );
            if cors {
                add_cors(response.headers_mut());
            }
            return response;
        }
    };
    if start > 0 && file.seek(io::SeekFrom::Start(start)).await.is_err() {
        return error(StatusCode::INTERNAL_SERVER_ERROR, "Can't read the file");
    }
    let mut response = Response::new(Body::from_stream(file_stream(file, count, rate)));
    *response.status_mut() = status;
    let headers = response.headers_mut();
    headers.insert(header::CONTENT_TYPE, HeaderValue::from_static(content_type));
    headers.insert(header::CONTENT_LENGTH, HeaderValue::from(count));
    headers.insert(header::ACCEPT_RANGES, HeaderValue::from_static("bytes"));
    if status == StatusCode::PARTIAL_CONTENT {
        headers.insert(
            header::CONTENT_RANGE,
            HeaderValue::from_str(&format!("bytes {start}-{}/{len}", start + count - 1)).unwrap(),
        );
    }
    if let Some(name) = download_name {
        if let Ok(value) = HeaderValue::from_str(&format!("attachment; filename=\"{name}\"")) {
            headers.insert(header::CONTENT_DISPOSITION, value);
        }
    }
    if cors {
        add_cors(headers);
    }
    response
}

// ------------------------------------------------------------------------------------------
// Tar, for downloading a whole log as one file.

const TAR_BLOCK: u64 = 512;

/// How many bytes `tar_header` and a file's padded contents take.
fn tar_entry_len(size: u64) -> u64 {
    TAR_BLOCK + size.div_ceil(TAR_BLOCK) * TAR_BLOCK
}

/// The length of a tar of `files`, with its end-of-archive blocks.
pub fn tar_len(files: &[LogFile]) -> u64 {
    files
        .iter()
        .map(|file| tar_entry_len(file.size))
        .sum::<u64>()
        + 2 * TAR_BLOCK
}

/// A ustar header for a regular file `name` of `size` bytes.  None if the name is too long.
pub fn tar_header(name: &str, size: u64, mtime: u64) -> Option<[u8; 512]> {
    let mut header = [0u8; 512];
    let bytes = name.as_bytes();
    // Names over 100 bytes are split at a '/' into a prefix of up to 155.
    let (prefix, name) = if bytes.len() <= 100 {
        (&[][..], bytes)
    } else {
        let split = (0..bytes.len())
            .rev()
            .find(|&i| bytes[i] == b'/' && i <= 155 && bytes.len() - i - 1 <= 100)?;
        (&bytes[..split], &bytes[split + 1..])
    };
    let octal = |field: &mut [u8], value: u64| {
        let text = format!("{:0width$o}", value, width = field.len() - 1);
        if text.len() >= field.len() {
            return false;
        }
        field[..text.len()].copy_from_slice(text.as_bytes());
        true
    };
    header[..name.len()].copy_from_slice(name);
    octal(&mut header[100..108], 0o644);
    octal(&mut header[108..116], 0);
    octal(&mut header[116..124], 0);
    if !octal(&mut header[124..136], size) {
        return None;
    }
    octal(&mut header[136..148], mtime);
    header[156] = b'0';
    header[257..263].copy_from_slice(b"ustar\0");
    header[263..265].copy_from_slice(b"00");
    header[345..345 + prefix.len()].copy_from_slice(prefix);
    // The checksum is of the header with its own field as spaces.
    header[148..156].copy_from_slice(b"        ");
    let sum: u64 = header.iter().map(|&b| u64::from(b)).sum();
    let text = format!("{sum:06o}\0 ");
    header[148..156].copy_from_slice(text.as_bytes());
    Some(header)
}

/// Streams `dir`'s `files` as a tar, each under `top/`, at no more than `rate` bytes a second.
/// Each file is cut to, or padded out to, the size it was listed at, so the length matches
/// [`tar_len`] even for a log still being written.
fn tar_stream(
    dir: PathBuf,
    top: String,
    files: Vec<LogFile>,
    rate: u64,
) -> impl futures::Stream<Item = io::Result<Bytes>> + Send {
    let (tx, rx) = mpsc::channel::<io::Result<Bytes>>(4);
    tokio::spawn(async move {
        let send = |chunk: io::Result<Bytes>| {
            let tx = tx.clone();
            async move { tx.send(chunk).await.is_ok() }
        };
        for file in &files {
            let mtime = fs::metadata(dir.join(&file.path))
                .and_then(|metadata| metadata.modified())
                .ok()
                .and_then(|time| time.duration_since(std::time::UNIX_EPOCH).ok())
                .map_or(0, |time| time.as_secs());
            let Some(header) = tar_header(&format!("{top}/{}", file.path), file.size, mtime) else {
                let _ = send(Err(io::Error::other(format!("Can't tar {}", file.path)))).await;
                return;
            };
            if !send(Ok(Bytes::copy_from_slice(&header))).await {
                return;
            }
            let mut left = file.size;
            if let Ok(mut opened) = tokio::fs::File::open(dir.join(&file.path)).await {
                let mut buffer = vec![0u8; CHUNK];
                while left > 0 {
                    let want = CHUNK.min(left as usize);
                    match opened.read(&mut buffer[..want]).await {
                        Ok(0) | Err(_) => break,
                        Ok(read) => {
                            left -= read as u64;
                            if rate > 0 {
                                tokio::time::sleep(Duration::from_secs_f64(
                                    read as f64 / rate as f64,
                                ))
                                .await;
                            }
                            if !send(Ok(Bytes::copy_from_slice(&buffer[..read]))).await {
                                return;
                            }
                        }
                    }
                }
            }
            // Pad a file that shrank or vanished, then to the block.
            let padding = left + (file.size.div_ceil(TAR_BLOCK) * TAR_BLOCK - file.size);
            if padding > 0 && !send(Ok(Bytes::from(vec![0u8; padding as usize]))).await {
                return;
            }
        }
        let _ = send(Ok(Bytes::from(vec![0u8; 2 * TAR_BLOCK as usize]))).await;
    });
    futures::stream::unfold(rx, |mut rx| async move {
        rx.recv().await.map(|item| (item, rx))
    })
}

// ------------------------------------------------------------------------------------------
// Talking to the robot, from a laptop.

/// The robot's `/logs`, as a laptop reaches it: `http://host:port` or `https://host[:port]`.
#[derive(Clone)]
pub struct RobotClient {
    pub base: String,
    host: String,
    port: u16,
    tls: Option<TlsConnector>,
}

impl std::fmt::Debug for RobotClient {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RobotClient")
            .field("base", &self.base)
            .finish()
    }
}

impl RobotClient {
    /// A client for the robot at `base`.  An https robot is trusted through `ca`, the team
    /// certificate authority (or a robot's own), as a PEM file.
    pub fn new(base: &str, ca: Option<&Path>) -> Result<Self, String> {
        let base = base.trim_end_matches('/').to_string();
        let (https, rest) = if let Some(rest) = base.strip_prefix("https://") {
            (true, rest)
        } else if let Some(rest) = base.strip_prefix("http://") {
            (false, rest)
        } else {
            return Err(format!("{base} isn't an http:// or https:// URL"));
        };
        if rest.contains('/') {
            return Err(format!("{base} has a path; give just the robot"));
        }
        let default_port = if https { 443 } else { 80 };
        let (host, port) = if let Some(bracketed) = rest.strip_prefix('[') {
            // An IPv6 address, maybe with a port.
            let (host, after) = bracketed
                .split_once(']')
                .ok_or_else(|| format!("{base} has a bad address"))?;
            let port = match after.strip_prefix(':') {
                Some(port) => port.parse().map_err(|_| format!("{base} has a bad port"))?,
                None => default_port,
            };
            (host.to_string(), port)
        } else {
            match rest.split_once(':') {
                Some((host, port)) => (
                    host.to_string(),
                    port.parse().map_err(|_| format!("{base} has a bad port"))?,
                ),
                None => (rest.to_string(), default_port),
            }
        };
        let tls = if https {
            let Some(ca) = ca else {
                return Err(format!(
                    "{base} is https, so give the team certificate authority with --robot-ca"
                ));
            };
            let mut roots = RootCertStore::empty();
            for cert in CertificateDer::pem_file_iter(ca)
                .map_err(|error| format!("Can't read {}: {error}", ca.display()))?
            {
                let cert =
                    cert.map_err(|error| format!("Bad certificate in {}: {error}", ca.display()))?;
                roots
                    .add(cert)
                    .map_err(|error| format!("Bad certificate in {}: {error}", ca.display()))?;
            }
            let config = ClientConfig::builder_with_provider(Arc::new(
                rustls::crypto::ring::default_provider(),
            ))
            .with_safe_default_protocol_versions()
            .map_err(|error| error.to_string())?
            .with_root_certificates(roots)
            .with_no_client_auth();
            Some(TlsConnector::from(Arc::new(config)))
        } else {
            None
        };
        Ok(Self {
            base,
            host,
            port,
            tls,
        })
    }

    /// GETs `path` from the robot, from byte `from` on.  Returns the status, the body's length
    /// if it said, and the body.  HTTP/1.0, so the body is plain bytes up to the close.
    async fn get(
        &self,
        path: &str,
        from: u64,
    ) -> Result<(u16, Option<u64>, Box<dyn AsyncRead + Unpin + Send>), String> {
        let tcp = tokio::time::timeout(
            ROBOT_TIMEOUT,
            TcpStream::connect((self.host.as_str(), self.port)),
        )
        .await
        .map_err(|_| format!("Timed out reaching {}", self.base))?
        .map_err(|error| format!("Can't reach {}: {error}", self.base))?;
        let stream: Box<dyn AsyncReadWrite> = match &self.tls {
            Some(tls) => {
                let name = ServerName::try_from(self.host.clone())
                    .map_err(|error| format!("Bad robot name {}: {error}", self.host))?;
                Box::new(
                    tokio::time::timeout(ROBOT_TIMEOUT, tls.connect(name, tcp))
                        .await
                        .map_err(|_| format!("Timed out reaching {}", self.base))?
                        .map_err(|error| {
                            format!("Can't connect securely to {}: {error}", self.base)
                        })?,
                )
            }
            None => Box::new(tcp),
        };
        let mut stream = BufReader::new(stream);
        let range = if from > 0 {
            format!("Range: bytes={from}-\r\n")
        } else {
            String::new()
        };
        let request = format!(
            "GET {path} HTTP/1.0\r\nHost: {}\r\nUser-Agent: logs_server\r\n{range}\r\n",
            self.host
        );
        stream
            .get_mut()
            .write_all(request.as_bytes())
            .await
            .map_err(|error| format!("Can't ask {}: {error}", self.base))?;
        let mut line = String::new();
        read_line(&mut stream, &mut line).await?;
        let status: u16 = line
            .split_whitespace()
            .nth(1)
            .and_then(|status| status.parse().ok())
            .ok_or_else(|| format!("The robot's answer isn't HTTP: {line:?}"))?;
        let mut length = None;
        loop {
            if read_line(&mut stream, &mut line).await? == 0 || line.trim().is_empty() {
                break;
            }
            if let Some((name, value)) = line.split_once(':') {
                if name.trim().eq_ignore_ascii_case("content-length") {
                    length = value.trim().parse().ok();
                }
            }
        }
        Ok((status, length, Box::new(stream)))
    }

    /// GETs `path` as JSON.
    async fn get_json(&self, path: &str) -> Result<Value, String> {
        let (status, _, mut body) = self.get(path, 0).await?;
        let mut text = Vec::new();
        tokio::time::timeout(ROBOT_TIMEOUT, body.read_to_end(&mut text))
            .await
            .map_err(|_| "Timed out reading the robot's answer".to_string())?
            .map_err(|error| format!("Lost the robot: {error}"))?;
        if status != 200 {
            return Err(format!(
                "The robot said {status}: {}",
                String::from_utf8_lossy(&text)
            ));
        }
        serde_json::from_slice(&text)
            .map_err(|error| format!("The robot's answer isn't JSON: {error}"))
    }
}

/// Reads one line of the robot's answer's head into `line`, returning its length.
async fn read_line(
    stream: &mut BufReader<Box<dyn AsyncReadWrite>>,
    line: &mut String,
) -> Result<usize, String> {
    line.clear();
    tokio::time::timeout(ROBOT_TIMEOUT, stream.read_line(line))
        .await
        .map_err(|_| "Timed out waiting for the robot".to_string())?
        .map_err(|error| format!("Lost the robot: {error}"))
}

trait AsyncReadWrite: AsyncRead + tokio::io::AsyncWrite + Unpin + Send {}
impl<T: AsyncRead + tokio::io::AsyncWrite + Unpin + Send> AsyncReadWrite for T {}

/// Percent-encodes a path segment.
fn encode(segment: &str) -> String {
    let mut out = String::new();
    for byte in segment.bytes() {
        if byte.is_ascii_alphanumeric() || b"-_.~".contains(&byte) {
            out.push(byte as char);
        } else {
            out.push_str(&format!("%{byte:02X}"));
        }
    }
    out
}

/// Downloads `job`'s log from the robot into `incoming/`, resuming files already partly
/// there, then moves it into the log folder and queues its conversion.
async fn pull(logs: &Logs, job: &Job) -> Result<(), String> {
    let Mode::Laptop { robot } = &logs.options.mode else {
        return Err("Only a laptop downloads from the robot".to_string());
    };
    let destination = logs.options.log_dir.join(&job.log);
    if destination.exists() {
        // Already here: just make sure it's converted.
        logs.add_job(
            &job.log,
            JobKind::Convert {
                preset: "all".to_string(),
                channels: vec![],
            },
        );
        return Ok(());
    }
    logs.update(job.id, |job| job.phase = "listing".to_string());
    let list = robot.get_json("/logs").await?;
    let entry = list["logs"]
        .as_array()
        .and_then(|list| list.iter().find(|log| log["name"] == json!(job.log)))
        .ok_or_else(|| format!("The robot has no log {}", job.log))?;
    let files: Vec<LogFile> = entry["files"]
        .as_array()
        .map(|files| {
            files
                .iter()
                .filter_map(|file| {
                    Some(LogFile {
                        path: file["path"].as_str()?.to_string(),
                        size: file["size"].as_u64()?,
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    for file in &files {
        if !safe_relative(&file.path) {
            return Err(format!("The robot named a bad file: {}", file.path));
        }
    }
    let total: u64 = files.iter().map(|file| file.size).sum();
    logs.update(job.id, |job| {
        job.phase = "downloading".to_string();
        job.total_bytes = total;
    });

    let incoming = logs.options.incoming_dir().join(&job.log);
    for file in &files {
        if job.cancel.load(Ordering::Relaxed) {
            return Err("Cancelled".to_string());
        }
        let local = incoming.join(&file.path);
        if let Some(parent) = local.parent() {
            fs::create_dir_all(parent)
                .map_err(|error| format!("Can't make {}: {error}", parent.display()))?;
        }
        let have = fs::metadata(&local).map_or(0, |metadata| metadata.len());
        if have >= file.size {
            job.done_bytes.fetch_add(file.size, Ordering::Relaxed);
            continue;
        }
        job.done_bytes.fetch_add(have, Ordering::Relaxed);
        let path = format!(
            "/logs/{}/files/{}",
            encode(&job.log),
            file.path
                .split('/')
                .map(encode)
                .collect::<Vec<_>>()
                .join("/")
        );
        let (status, _, mut body) = robot.get(&path, have).await?;
        let mut out = match status {
            // Resumed where it left off.
            206 => {
                tokio::fs::OpenOptions::new()
                    .append(true)
                    .open(&local)
                    .await
            }
            // The whole file, again.
            200 => {
                job.done_bytes.fetch_sub(have, Ordering::Relaxed);
                tokio::fs::File::create(&local).await
            }
            _ => return Err(format!("The robot said {status} for {}", file.path)),
        }
        .map_err(|error| format!("Can't write {}: {error}", local.display()))?;
        let mut buffer = vec![0u8; CHUNK];
        loop {
            if job.cancel.load(Ordering::Relaxed) {
                return Err("Cancelled".to_string());
            }
            let read = tokio::time::timeout(ROBOT_TIMEOUT, body.read(&mut buffer))
                .await
                .map_err(|_| "The robot stopped sending".to_string())?
                .map_err(|error| format!("Lost the robot: {error}"))?;
            if read == 0 {
                break;
            }
            out.write_all(&buffer[..read])
                .await
                .map_err(|error| format!("Can't write {}: {error}", local.display()))?;
            job.done_bytes.fetch_add(read as u64, Ordering::Relaxed);
        }
        out.flush()
            .await
            .map_err(|error| format!("Can't write {}: {error}", local.display()))?;
        let got = fs::metadata(&local).map_or(0, |metadata| metadata.len());
        // A file the logger was still writing can have grown; less is a broken download.
        if got < file.size {
            return Err(format!(
                "{} stopped at {got} of {} bytes; download again to resume",
                file.path, file.size
            ));
        }
    }
    fs::create_dir_all(&incoming)
        .map_err(|error| format!("Can't make {}: {error}", incoming.display()))?;
    fs::create_dir_all(&logs.options.log_dir)
        .map_err(|error| format!("Can't make {}: {error}", logs.options.log_dir.display()))?;
    fs::rename(&incoming, &destination)
        .map_err(|error| format!("Can't move the download into place: {error}"))?;
    logs.add_job(
        &job.log,
        JobKind::Convert {
            preset: "all".to_string(),
            channels: vec![],
        },
    );
    Ok(())
}

/// Whether `path` stays inside the folder it's relative to.
fn safe_relative(path: &str) -> bool {
    !path.is_empty()
        && Path::new(path)
            .components()
            .all(|component| matches!(component, Component::Normal(_)))
}

// ------------------------------------------------------------------------------------------
// The routes.

fn json_response(status: StatusCode, body: Value) -> Response {
    (
        status,
        [(header::CONTENT_TYPE, "application/json")],
        body.to_string(),
    )
        .into_response()
}

fn error(status: StatusCode, message: &str) -> Response {
    json_response(status, json!({"error": message}))
}

fn bad_log(log: &str) -> Response {
    error(StatusCode::NOT_FOUND, &format!("There's no log {log}"))
}

/// The `/logs` routes, over `logs`.  Run [`run_jobs`] alongside, or nothing converts.
pub fn router(logs: Arc<Logs>) -> Router {
    let mut router = Router::new()
        .route("/logs", get(list))
        .route("/logs/info", get(info))
        .route("/logs/presets", get(presets))
        .route("/logs/jobs", get(jobs))
        .route("/logs/jobs/{id}", get(job).delete(cancel_job))
        .route("/logs/mcap/{file}", get(mcap).options(mcap_options))
        .route("/logs/{log}", axum::routing::delete(delete_log))
        .route("/logs/{log}/summary", get(summary))
        .route("/logs/{log}/files/{*path}", get(file))
        .route("/logs/{log}/download", get(download))
        .route("/logs/{log}/convert", post(convert_log));
    if matches!(logs.options.mode, Mode::Laptop { .. }) {
        router = router
            .route("/logs/robot", get(robot_logs))
            .route("/logs/robot/{log}/pull", post(pull_log));
    }
    router.with_state(logs)
}

type Shared = State<Arc<Logs>>;

async fn info(State(logs): Shared) -> Response {
    let (mode, robot) = match &logs.options.mode {
        Mode::Robot { .. } => ("robot", Value::Null),
        Mode::Laptop { robot } => ("laptop", json!(robot.base)),
    };
    json_response(
        StatusCode::OK,
        json!({
            "mode": mode,
            "robot": robot,
            "free_bytes": free_bytes(&logs.options.log_dir),
            "enabled": logs.robot_enabled().await,
        }),
    )
}

async fn presets(State(logs): Shared) -> Response {
    let mut args = vec![];
    if let Some(presets) = &logs.options.presets {
        args.push(format!("--presets={}", presets.display()));
    }
    args.push("presets".to_string());
    match run_log_manager(
        &logs.options.log_manager,
        args,
        Arc::new(AtomicBool::new(false)),
        false,
    )
    .await
    {
        Ok(text) => match serde_json::from_str::<Value>(&text) {
            Ok(presets) => json_response(StatusCode::OK, presets),
            Err(_) => error(
                StatusCode::INTERNAL_SERVER_ERROR,
                "log_manager's presets aren't JSON",
            ),
        },
        Err(why) => error(StatusCode::INTERNAL_SERVER_ERROR, &why),
    }
}

async fn list(State(logs): Shared) -> Response {
    let options = logs.options.clone();
    let listed = tokio::task::spawn_blocking(move || list_logs(&options)).await;
    match listed {
        Ok(Ok(mut list)) => {
            for log in &mut list {
                let name = log["name"].as_str().unwrap_or_default().to_string();
                log["jobs"] = Value::from(logs.active_jobs(&name));
            }
            json_response(StatusCode::OK, json!({"logs": list}))
        }
        Ok(Err(error_)) if error_.kind() == io::ErrorKind::NotFound => {
            json_response(StatusCode::OK, json!({"logs": []}))
        }
        Ok(Err(error_)) => error(
            StatusCode::INTERNAL_SERVER_ERROR,
            &format!("Can't list {}: {error_}", logs.options.log_dir.display()),
        ),
        Err(_) => error(StatusCode::INTERNAL_SERVER_ERROR, "Listing failed"),
    }
}

async fn summary(State(logs): Shared, UrlPath(log): UrlPath<String>) -> Response {
    if !is_log_name(&log) {
        return bad_log(&log);
    }
    match tokio::fs::read(logs.options.summary_path(&log)).await {
        Ok(bytes) => (
            StatusCode::OK,
            [(header::CONTENT_TYPE, "application/json")],
            bytes,
        )
            .into_response(),
        Err(_) => error(
            StatusCode::NOT_FOUND,
            &format!("{log} hasn't been summarized; convert it first"),
        ),
    }
}

async fn file(
    State(logs): Shared,
    UrlPath((log, path)): UrlPath<(String, String)>,
    headers: HeaderMap,
) -> Response {
    if !is_log_name(&log) || !safe_relative(&path) {
        return bad_log(&log);
    }
    serve_file(
        &logs.options.log_dir.join(&log).join(&path),
        &headers,
        "application/octet-stream",
        logs.options.max_download_rate,
        false,
        None,
    )
    .await
}

async fn download(State(logs): Shared, UrlPath(log): UrlPath<String>) -> Response {
    if !is_log_name(&log) {
        return bad_log(&log);
    }
    let dir = logs.options.log_dir.join(&log);
    let files = match log_files(&dir) {
        Ok(files) => files,
        Err(_) => return bad_log(&log),
    };
    let len = tar_len(&files);
    let mut response = Response::new(Body::from_stream(tar_stream(
        dir,
        log.clone(),
        files,
        logs.options.max_download_rate,
    )));
    let headers = response.headers_mut();
    headers.insert(
        header::CONTENT_TYPE,
        HeaderValue::from_static("application/x-tar"),
    );
    headers.insert(header::CONTENT_LENGTH, HeaderValue::from(len));
    if let Ok(value) = HeaderValue::from_str(&format!("attachment; filename=\"{log}.tar\"")) {
        headers.insert(header::CONTENT_DISPOSITION, value);
    }
    response
}

async fn convert_log(State(logs): Shared, UrlPath(log): UrlPath<String>, body: String) -> Response {
    if !is_log_name(&log) || !logs.options.log_dir.join(&log).is_dir() {
        return bad_log(&log);
    }
    let request: Value = if body.trim().is_empty() {
        json!({})
    } else {
        match serde_json::from_str(&body) {
            Ok(request) => request,
            Err(_) => return error(StatusCode::BAD_REQUEST, "The body isn't JSON"),
        }
    };
    let preset = request["preset"].as_str().unwrap_or("all").to_string();
    if preset.is_empty()
        || !preset
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || c == b'_' || c == b'-')
    {
        return error(
            StatusCode::BAD_REQUEST,
            "Presets are named with letters, digits, _ and -",
        );
    }
    let channels: Vec<String> = request["channels"]
        .as_array()
        .map(|channels| {
            channels
                .iter()
                .filter_map(|channel| channel.as_str().map(str::to_string))
                .collect()
        })
        .unwrap_or_default();
    if channels
        .iter()
        .any(|channel| channel.contains([';', ',']) || !channel.contains(' '))
    {
        return error(
            StatusCode::BAD_REQUEST,
            "Channels are \"<name> <type>\", with no ';' or ','",
        );
    }
    if matches!(logs.options.mode, Mode::Robot { .. }) && logs.robot_enabled().await {
        return error(
            StatusCode::CONFLICT,
            "The robot is enabled; convert once it's disabled",
        );
    }
    let id = logs.add_job(&log, JobKind::Convert { preset, channels });
    json_response(
        StatusCode::ACCEPTED,
        logs.job_json(id).unwrap_or(Value::Null),
    )
}

async fn delete_log(State(logs): Shared, UrlPath(log): UrlPath<String>) -> Response {
    if !is_log_name(&log) || !logs.options.log_dir.join(&log).is_dir() {
        return bad_log(&log);
    }
    if current_log(&logs.options.log_dir).as_deref() == Some(log.as_str()) {
        return error(StatusCode::CONFLICT, "The logger is still writing that log");
    }
    if !logs.active_jobs(&log).is_empty() {
        return error(
            StatusCode::CONFLICT,
            "That log is being converted or downloaded",
        );
    }
    if let Err(error_) = fs::remove_dir_all(logs.options.log_dir.join(&log)) {
        return error(
            StatusCode::INTERNAL_SERVER_ERROR,
            &format!("Can't delete {log}: {error_}"),
        );
    }
    let _ = fs::remove_file(logs.options.summary_path(&log));
    if let Ok(entries) = fs::read_dir(logs.options.mcap_dir()) {
        let prefix = format!("{log}.");
        for entry in entries.filter_map(Result::ok) {
            if entry
                .file_name()
                .to_str()
                .is_some_and(|name| name.starts_with(&prefix))
            {
                let _ = fs::remove_file(entry.path());
            }
        }
    }
    json_response(StatusCode::OK, json!({"deleted": log}))
}

async fn jobs(State(logs): Shared) -> Response {
    json_response(StatusCode::OK, json!({"jobs": logs.jobs_json()}))
}

async fn job(State(logs): Shared, UrlPath(id): UrlPath<u64>) -> Response {
    match logs.job_json(id) {
        Some(job) => json_response(StatusCode::OK, job),
        None => error(StatusCode::NOT_FOUND, &format!("There's no job {id}")),
    }
}

async fn cancel_job(State(logs): Shared, UrlPath(id): UrlPath<u64>) -> Response {
    let mut jobs = logs.jobs.lock().unwrap();
    let Some(job) = jobs.get_mut(id) else {
        return error(StatusCode::NOT_FOUND, &format!("There's no job {id}"));
    };
    match job.state {
        JobState::Queued => job.state = JobState::Cancelled,
        JobState::Running => job.cancel.store(true, Ordering::Relaxed),
        _ => {}
    }
    json_response(StatusCode::OK, job.to_json())
}

async fn mcap(State(logs): Shared, UrlPath(name): UrlPath<String>, headers: HeaderMap) -> Response {
    if !name.ends_with(".mcap") || !safe_relative(&name) || name.contains('/') {
        return error(StatusCode::NOT_FOUND, "No such MCAP");
    }
    serve_file(
        &logs.options.mcap_dir().join(&name),
        &headers,
        "application/octet-stream",
        0,
        true,
        Some(name.clone()),
    )
    .await
}

async fn mcap_options() -> Response {
    let mut response = StatusCode::NO_CONTENT.into_response();
    add_cors(response.headers_mut());
    response
}

async fn robot_logs(State(logs): Shared) -> Response {
    let Mode::Laptop { robot } = &logs.options.mode else {
        return error(StatusCode::NOT_FOUND, "Only a laptop lists a robot's logs");
    };
    match robot.get_json("/logs").await {
        Ok(mut list) => {
            if let Some(entries) = list["logs"].as_array_mut() {
                for log in entries {
                    let name = log["name"].as_str().unwrap_or_default().to_string();
                    log["on_laptop"] =
                        json!(is_log_name(&name) && logs.options.log_dir.join(&name).is_dir());
                    log["pulls"] = Value::from(
                        logs.active_jobs(&name)
                            .into_iter()
                            .filter(|job| job["kind"] == "pull")
                            .collect::<Vec<_>>(),
                    );
                    // The laptop's page doesn't need every file.
                    if let Some(object) = log.as_object_mut() {
                        object.remove("files");
                    }
                }
            }
            list["connected"] = json!(true);
            list["robot"] = json!(robot.base);
            json_response(StatusCode::OK, list)
        }
        Err(why) => json_response(
            StatusCode::OK,
            json!({"connected": false, "robot": robot.base, "error": why, "logs": []}),
        ),
    }
}

async fn pull_log(State(logs): Shared, UrlPath(log): UrlPath<String>) -> Response {
    if !is_log_name(&log) {
        return bad_log(&log);
    }
    let id = logs.add_job(&log, JobKind::Pull);
    json_response(
        StatusCode::ACCEPTED,
        logs.job_json(id).unwrap_or(Value::Null),
    )
}

/// The laptop's `logs_server`: the log page from `data_dir`, and `/logs` over `options`.
pub fn laptop_router(data_dir: PathBuf, logs: Arc<Logs>) -> Router {
    let index = data_dir.join("logs.html");
    router(logs)
        .route("/healthz", get(|| async { "ok" }))
        .route_service("/", tower_http::services::ServeFile::new(index))
        .fallback_service(tower_http::services::ServeDir::new(data_dir))
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::http::Request;
    use tower::ServiceExt;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("logs_test_{name}_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A log folder `name` in `dir` holding `files`.
    fn make_log(dir: &Path, name: &str, files: &[(&str, &[u8])]) {
        for (path, contents) in files {
            let path = dir.join(name).join(path);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, contents).unwrap();
        }
    }

    /// A stand-in for log_manager: writes a summary for analyze and an MCAP for convert, and
    /// records its arguments.
    fn fake_log_manager(dir: &Path) -> PathBuf {
        use std::os::unix::fs::PermissionsExt;
        let script = dir.join("fake_log_manager.sh");
        fs::write(
            &script,
            r#"#!/bin/sh
echo "$@" >> "$(dirname "$0")/calls"
case "$1" in
  presets) echo '{"presets": [{"name": "all", "regexes": [".*"]}]}' ;;
  analyze)
    out="${2#--output=}"
    echo '{"matches": [{"label": "test_qm7_blue", "match_number": 7}], "faults": [1, 2], "log_messages": [], "min_battery_voltage": null}' > "$out" ;;
  convert) for last; do :; done; printf 'MCAP' > "$last" ;;
  *) exit 2 ;;
esac
"#,
        )
        .unwrap();
        fs::set_permissions(&script, fs::Permissions::from_mode(0o755)).unwrap();
        script
    }

    async fn send(router: &Router, request: Request<Body>) -> (StatusCode, HeaderMap, Vec<u8>) {
        let response = router.clone().oneshot(request).await.unwrap();
        let status = response.status();
        let headers = response.headers().clone();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap()
            .to_vec();
        (status, headers, body)
    }

    async fn get_json(router: &Router, uri: &str) -> Value {
        let (status, _, body) = send(router, Request::get(uri).body(Body::empty()).unwrap()).await;
        assert_eq!(
            status,
            StatusCode::OK,
            "{uri}: {}",
            String::from_utf8_lossy(&body)
        );
        serde_json::from_slice(&body).unwrap()
    }

    /// Waits for job `id` to finish, returning it.
    async fn wait_for_job(router: &Router, id: u64) -> Value {
        for _ in 0..200 {
            let job = get_json(router, &format!("/logs/jobs/{id}")).await;
            if ["done", "failed", "cancelled"].contains(&job["state"].as_str().unwrap()) {
                return job;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        panic!("Job {id} didn't finish");
    }

    const LOG: &str = "fbs_log-012_2026-03-14_10-02-11";
    const OLDER: &str = "fbs_log-003_2026-03-14_09-00-00";

    fn robot_fixture(name: &str) -> (Arc<Logs>, PathBuf) {
        let dir = temp_dir(name);
        let log_dir = dir.join("logs");
        make_log(
            &log_dir,
            LOG,
            &[("a.bfbs", b"0123456789"), ("orin/b.bfbs", b"abc")],
        );
        make_log(&log_dir, OLDER, &[("a.bfbs", b"x")]);
        std::os::unix::fs::symlink(LOG, log_dir.join(CURRENT_LINK)).unwrap();
        let logs = Logs::new(LogsOptions::new(
            log_dir,
            fake_log_manager(&dir),
            Mode::Robot { api: None },
        ));
        (logs, dir)
    }

    #[test]
    fn checks_log_names() {
        assert!(is_log_name(LOG));
        assert!(is_log_name("fbs_log-1_x"));
        for bad in [
            "fbs_log-current",
            "fbs_log-_x",
            "fbs_log-12_",
            "fbs_log-12_../x",
            "fbs_log-12_a/b",
            "../fbs_log-12_x",
            "",
        ] {
            assert!(!is_log_name(bad), "{bad}");
        }
        assert_eq!(log_index(LOG), 12);
        assert!(safe_relative("orin/b.bfbs"));
        for bad in ["", "../x", "/etc/passwd", "a/../../x"] {
            assert!(!safe_relative(bad), "{bad}");
        }
    }

    #[test]
    fn parses_ranges() {
        assert_eq!(parse_range(None, 10), Ok(None));
        assert_eq!(parse_range(Some("bytes=0-4"), 10), Ok(Some((0, 4))));
        assert_eq!(parse_range(Some("bytes=5-"), 10), Ok(Some((5, 9))));
        assert_eq!(parse_range(Some("bytes=-3"), 10), Ok(Some((7, 9))));
        assert_eq!(parse_range(Some("bytes=8-100"), 10), Ok(Some((8, 9))));
        assert_eq!(parse_range(Some("bytes=0-1,3-4"), 10), Ok(None));
        assert_eq!(parse_range(Some("bytes=10-"), 10), Err(()));
        assert_eq!(parse_range(Some("bytes=5-2"), 10), Err(()));
        assert_eq!(parse_range(Some("bytes=x-"), 10), Err(()));
    }

    #[test]
    fn writes_tar_headers() {
        let header = tar_header("dir/file.bfbs", 1234, 0).unwrap();
        assert_eq!(&header[..13], b"dir/file.bfbs");
        assert_eq!(&header[124..135], b"00000002322");
        assert_eq!(&header[257..263], b"ustar\0");
        // The checksum is the sum of the header with the checksum field as spaces.
        let mut copy = header;
        copy[148..156].copy_from_slice(b"        ");
        let sum: u64 = copy.iter().map(|&b| u64::from(b)).sum();
        assert_eq!(&header[148..155], format!("{sum:06o}\0").as_bytes());

        let long = format!("{}/{}", "d".repeat(120), "f".repeat(50));
        let header = tar_header(&long, 1, 0).unwrap();
        assert_eq!(&header[..50], "f".repeat(50).as_bytes());
        assert_eq!(&header[345..465], "d".repeat(120).as_bytes());
        assert!(tar_header(&"x".repeat(300), 1, 0).is_none());

        let files = [
            LogFile {
                path: "a".into(),
                size: 10,
            },
            LogFile {
                path: "b".into(),
                size: 512,
            },
        ];
        assert_eq!(tar_len(&files), (512 + 512) + (512 + 512) + 1024);
    }

    #[test]
    fn names_mcaps() {
        assert_eq!(mcap_name(LOG, "all", &[]), format!("{LOG}.mcap"));
        assert_eq!(mcap_name(LOG, "power", &[]), format!("{LOG}.power.mcap"));
        let custom = mcap_name(LOG, "all", &["/a b".to_string()]);
        assert!(custom.starts_with(&format!("{LOG}.custom-")), "{custom}");
        assert_eq!(custom, mcap_name(LOG, "all", &["/a b".to_string()]));
        assert_ne!(custom, mcap_name(LOG, "all", &["/a c".to_string()]));
    }

    #[test]
    fn parses_robot_urls() {
        let robot = RobotClient::new("http://localhost:8090/", None).unwrap();
        assert_eq!((robot.host.as_str(), robot.port), ("localhost", 8090));
        assert_eq!(robot.base, "http://localhost:8090");
        let robot = RobotClient::new("http://10.18.68.101", None).unwrap();
        assert_eq!(robot.port, 80);
        assert!(RobotClient::new("https://orin-1868-1.local", None)
            .unwrap_err()
            .contains("--robot-ca"));
        assert!(RobotClient::new("orin-1868-1.local", None).is_err());
        assert!(RobotClient::new("http://robot/logs", None).is_err());
    }

    #[tokio::test]
    async fn lists_logs_newest_first() {
        let (logs, _dir) = robot_fixture("list");
        let router = router(logs);
        let list = get_json(&router, "/logs").await;
        let names: Vec<&str> = list["logs"]
            .as_array()
            .unwrap()
            .iter()
            .map(|log| log["name"].as_str().unwrap())
            .collect();
        assert_eq!(names, [LOG, OLDER]);
        let newest = &list["logs"][0];
        assert_eq!(newest["current"], true);
        assert_eq!(newest["size"], 13);
        assert_eq!(
            newest["files"],
            json!([{"path": "a.bfbs", "size": 10}, {"path": "orin/b.bfbs", "size": 3}])
        );
        assert_eq!(list["logs"][1]["current"], false);

        let info = get_json(&router, "/logs/info").await;
        assert_eq!(info["mode"], "robot");
        assert_eq!(info["enabled"], false);
        // Laptop-only routes aren't on the robot.
        let (status, _, _) = send(
            &router,
            Request::get("/logs/robot").body(Body::empty()).unwrap(),
        )
        .await;
        assert!(
            [StatusCode::NOT_FOUND, StatusCode::METHOD_NOT_ALLOWED].contains(&status),
            "{status}"
        );
    }

    #[tokio::test]
    async fn serves_files_with_ranges() {
        let (logs, _dir) = robot_fixture("files");
        let router = router(logs);
        let uri = format!("/logs/{LOG}/files/a.bfbs");
        let (status, headers, body) =
            send(&router, Request::get(&uri).body(Body::empty()).unwrap()).await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(body, b"0123456789");
        assert_eq!(headers[header::ACCEPT_RANGES], "bytes");

        let (status, headers, body) = send(
            &router,
            Request::get(&uri)
                .header(header::RANGE, "bytes=4-")
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::PARTIAL_CONTENT);
        assert_eq!(body, b"456789");
        assert_eq!(headers[header::CONTENT_RANGE], "bytes 4-9/10");

        let (status, _, _) = send(
            &router,
            Request::get(&uri)
                .header(header::RANGE, "bytes=20-")
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::RANGE_NOT_SATISFIABLE);

        let (_, _, body) = send(
            &router,
            Request::get(format!("/logs/{LOG}/files/orin/b.bfbs"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(body, b"abc");

        for bad in [
            format!("/logs/{LOG}/files/..%2F..%2Fsecret"),
            "/logs/fbs_log-current/files/a.bfbs".to_string(),
            format!("/logs/{LOG}/files/missing"),
        ] {
            let (status, _, _) =
                send(&router, Request::get(&bad).body(Body::empty()).unwrap()).await;
            assert_eq!(status, StatusCode::NOT_FOUND, "{bad}");
        }
    }

    #[tokio::test]
    async fn downloads_a_log_as_tar() {
        let (logs, _dir) = robot_fixture("tar");
        let router = router(logs);
        let (status, headers, body) = send(
            &router,
            Request::get(format!("/logs/{LOG}/download"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(headers[header::CONTENT_LENGTH], body.len().to_string());
        assert_eq!(body.len(), 512 + 512 + 512 + 512 + 1024);
        assert_eq!(&body[..LOG.len() + 7], format!("{LOG}/a.bfbs").as_bytes());
        assert_eq!(&body[512..522], b"0123456789");
        assert_eq!(
            &body[1024..1024 + LOG.len() + 12],
            format!("{LOG}/orin/b.bfbs").as_bytes()
        );
        assert!(headers[header::CONTENT_DISPOSITION]
            .to_str()
            .unwrap()
            .contains(".tar"));
    }

    #[tokio::test]
    async fn converts_and_serves_mcaps_to_foxglove() {
        let (logs, dir) = robot_fixture("convert");
        tokio::spawn(run_jobs(logs.clone()));
        let router = router(logs);

        let (status, _, body) = send(
            &router,
            Request::post(format!("/logs/{OLDER}/convert"))
                .body(Body::from(r#"{"preset": "all"}"#))
                .unwrap(),
        )
        .await;
        assert_eq!(
            status,
            StatusCode::ACCEPTED,
            "{}",
            String::from_utf8_lossy(&body)
        );
        let id = serde_json::from_slice::<Value>(&body).unwrap()["id"]
            .as_u64()
            .unwrap();
        let job = wait_for_job(&router, id).await;
        assert_eq!(job["state"], "done", "{job}");
        assert_eq!(job["mcap"], format!("{OLDER}.mcap"));

        let calls = fs::read_to_string(dir.join("calls")).unwrap();
        assert!(calls.contains("analyze --output="), "{calls}");
        assert!(calls.contains("convert --preset=all"), "{calls}");

        // The list now has the match and the MCAP.
        let list = get_json(&router, "/logs").await;
        let older = &list["logs"][1];
        assert_eq!(older["summary"]["matches"][0]["label"], "test_qm7_blue");
        assert_eq!(older["summary"]["fault_count"], 2);
        assert_eq!(
            older["mcaps"],
            json!([{"name": format!("{OLDER}.mcap"), "size": 4}])
        );
        let summary = get_json(&router, &format!("/logs/{OLDER}/summary")).await;
        assert_eq!(summary["matches"][0]["match_number"], 7);

        // Foxglove reads it from another origin, in pieces.
        let (status, headers, body) = send(
            &router,
            Request::get(format!("/logs/mcap/{OLDER}.mcap"))
                .header(header::RANGE, "bytes=0-1")
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::PARTIAL_CONTENT);
        assert_eq!(body, b"MC");
        assert_eq!(headers["access-control-allow-origin"], "*");
        assert!(headers["access-control-expose-headers"]
            .to_str()
            .unwrap()
            .contains("Content-Range"));
        let (status, headers, _) = send(
            &router,
            Request::options(format!("/logs/mcap/{OLDER}.mcap"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::NO_CONTENT);
        assert_eq!(headers["access-control-allow-headers"], "Range");

        // A second request for the same thing doesn't convert again.
        let (_, _, body) = send(
            &router,
            Request::post(format!("/logs/{OLDER}/convert"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        let id = serde_json::from_slice::<Value>(&body).unwrap()["id"]
            .as_u64()
            .unwrap();
        assert_eq!(wait_for_job(&router, id).await["state"], "done");
        let calls = fs::read_to_string(dir.join("calls")).unwrap();
        assert_eq!(
            calls
                .lines()
                .filter(|line| line.starts_with("convert "))
                .count(),
            1,
            "{calls}"
        );

        // Bad picks are refused.
        let (status, _, _) = send(
            &router,
            Request::post(format!("/logs/{OLDER}/convert"))
                .body(Body::from(r#"{"channels": ["/a b;rm"]}"#))
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn deletes_logs_but_not_the_current_one() {
        let (logs, _dir) = robot_fixture("delete");
        let log_dir = logs.options.log_dir.clone();
        let router = router(logs);
        let delete = |log: &str| {
            Request::delete(format!("/logs/{log}"))
                .body(Body::empty())
                .unwrap()
        };
        let (status, _, _) = send(&router, delete(LOG)).await;
        assert_eq!(status, StatusCode::CONFLICT);
        assert!(log_dir.join(LOG).is_dir());
        let (status, _, _) = send(&router, delete(OLDER)).await;
        assert_eq!(status, StatusCode::OK);
        assert!(!log_dir.join(OLDER).exists());
    }

    /// The robot is enabled while a fake JoystickState says so.
    fn fake_robot(enabled: Arc<AtomicBool>) -> mpsc::UnboundedSender<ApiRequest> {
        let (requests, mut incoming) = mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Some(request) = incoming.recv().await {
                if let ApiRequest::Latest { reply, .. } = request {
                    let _ = reply.send(Ok(crate::api::Latest {
                        monotonic_event_time_ns: 0,
                        realtime_event_time_ns: 0,
                        age_ns: 0,
                        json: format!("{{\"enabled\": {}}}", enabled.load(Ordering::Relaxed)),
                    }));
                }
            }
        });
        requests
    }

    #[tokio::test]
    async fn refuses_to_convert_while_enabled() {
        let (logs, dir) = robot_fixture("enabled");
        let enabled = Arc::new(AtomicBool::new(true));
        let mut options = logs.options.clone();
        options.mode = Mode::Robot {
            api: Some(fake_robot(enabled.clone())),
        };
        let logs = Logs::new(options);
        tokio::spawn(run_jobs(logs.clone()));
        let router = router(logs);
        assert_eq!(get_json(&router, "/logs/info").await["enabled"], true);
        let (status, _, _) = send(
            &router,
            Request::post(format!("/logs/{OLDER}/convert"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::CONFLICT);
        assert!(!dir.join("calls").exists());

        enabled.store(false, Ordering::Relaxed);
        let (status, _, body) = send(
            &router,
            Request::post(format!("/logs/{OLDER}/convert"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::ACCEPTED);
        let id = serde_json::from_slice::<Value>(&body).unwrap()["id"]
            .as_u64()
            .unwrap();
        assert_eq!(wait_for_job(&router, id).await["state"], "done");
    }

    /// A laptop lists a robot's logs, downloads the one picked, and converts it.
    #[tokio::test]
    async fn laptop_pulls_a_picked_log_from_the_robot() {
        let (robot, _robot_dir) = robot_fixture("pull_robot");
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        tokio::spawn(crate::serve_http(listener, router(robot)));

        let laptop_dir = temp_dir("pull_laptop");
        let log_dir = laptop_dir.join("frc_logs");
        let laptop = Logs::new(LogsOptions::new(
            log_dir.clone(),
            fake_log_manager(&laptop_dir),
            Mode::Laptop {
                robot: RobotClient::new(&format!("http://{address}"), None).unwrap(),
            },
        ));
        tokio::spawn(run_jobs(laptop.clone()));
        let router = router(laptop);

        let listed = get_json(&router, "/logs/robot").await;
        assert_eq!(listed["connected"], true);
        assert_eq!(listed["logs"].as_array().unwrap().len(), 2);
        assert_eq!(listed["logs"][0]["on_laptop"], false);

        // Nothing comes over until it's picked.
        assert_eq!(get_json(&router, "/logs").await["logs"], json!([]));

        // A partial earlier download is resumed.
        let incoming = laptop_dir.join("frc_logs/logs_server/incoming").join(LOG);
        fs::create_dir_all(&incoming).unwrap();
        fs::write(incoming.join("a.bfbs"), b"0123").unwrap();

        let (status, _, body) = send(
            &router,
            Request::post(format!("/logs/robot/{LOG}/pull"))
                .body(Body::empty())
                .unwrap(),
        )
        .await;
        assert_eq!(status, StatusCode::ACCEPTED);
        let id = serde_json::from_slice::<Value>(&body).unwrap()["id"]
            .as_u64()
            .unwrap();
        let job = wait_for_job(&router, id).await;
        assert_eq!(job["state"], "done", "{job}");
        assert_eq!(job["done_bytes"], 13);
        assert_eq!(
            fs::read(log_dir.join(LOG).join("a.bfbs")).unwrap(),
            b"0123456789"
        );
        assert_eq!(
            fs::read(log_dir.join(LOG).join("orin/b.bfbs")).unwrap(),
            b"abc"
        );
        assert!(!incoming.exists());

        // Then it's converted by itself.
        for _ in 0..200 {
            if log_dir
                .join("logs_server/mcap")
                .join(format!("{LOG}.mcap"))
                .exists()
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        let list = get_json(&router, "/logs").await;
        assert_eq!(list["logs"][0]["name"], LOG);
        assert_eq!(list["logs"][0]["mcaps"][0]["name"], format!("{LOG}.mcap"));
        assert_eq!(
            get_json(&router, "/logs/robot").await["logs"][0]["on_laptop"],
            true
        );
        // The other log stayed on the robot.
        assert!(!log_dir.join(OLDER).exists());
        assert_eq!(get_json(&router, "/logs/info").await["mode"], "laptop");
    }

    #[tokio::test]
    async fn laptop_says_when_the_robot_is_away() {
        let laptop_dir = temp_dir("away");
        let laptop = Logs::new(LogsOptions::new(
            laptop_dir.join("frc_logs"),
            fake_log_manager(&laptop_dir),
            Mode::Laptop {
                robot: RobotClient::new("http://127.0.0.1:1", None).unwrap(),
            },
        ));
        let listed = get_json(&router(laptop), "/logs/robot").await;
        assert_eq!(listed["connected"], false);
        assert!(listed["error"].as_str().unwrap().contains("127.0.0.1"));
    }
}
