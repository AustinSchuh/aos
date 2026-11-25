//! `logs_server`: the robot's logs, on a laptop.  See `frc/logs/README.md`.
//!
//! Serves the log page and robot_server's `/logs` routes on `localhost`, over the logs it has
//! downloaded from a robot into `--log-dir`.  The page lists the robot's logs; the ones the
//! user picks are downloaded, then converted to MCAP for Foxglove, and stay here, so they can
//! be looked at with the robot off.
//!
//! ```text
//! bazel run //aos/network/robot_server:logs_server -- \
//!     --robot=https://orin-1868-1.local --robot-ca=$HOME/robot-ca/ca.crt
//! ```
//!
//! then open <http://localhost:8080/>.

use std::path::{Path, PathBuf};

use clap::Parser;
use robot_server::logs;

#[derive(Parser, Debug)]
#[command(name = "logs_server")]
struct Flags {
    /// The robot, as `https://<name>` (robot_server), or `http://<host>:<port>` (a replay).
    #[arg(long)]
    robot: String,
    /// The team certificate authority (or a robot's own) that an https robot's certificate
    /// comes from, as a PEM file.  `trust_orin.py` prints where it is.
    #[arg(long)]
    robot_ca: Option<PathBuf>,
    /// Where downloaded logs are kept.  `~` is the home folder.
    #[arg(long, default_value = "~/frc_logs")]
    log_dir: String,
    /// Port for the page, on `--listen`.
    #[arg(long, default_value_t = 8080)]
    port: u16,
    /// The address to listen on: this computer only, unless told otherwise.
    #[arg(long, default_value = "127.0.0.1")]
    listen: String,
    /// `frc/logs:log_manager`.
    #[arg(long, default_value = "frc/logs/log_manager")]
    log_manager: PathBuf,
    /// The log page.
    #[arg(long, default_value = "frc/logs/www")]
    data_dir: PathBuf,
    /// The robot's channel presets, for converting logs to MCAP.  The generic ones without it.
    #[arg(long)]
    presets: Option<PathBuf>,
}

/// `path` with a leading `~` as the home folder, and made absolute against where the user
/// started it (`bazel run` starts it elsewhere).
fn expand(path: &str) -> PathBuf {
    let path = match path.strip_prefix('~') {
        Some(rest) => {
            let home = std::env::var_os("HOME")
                .map(PathBuf::from)
                .unwrap_or_default();
            home.join(rest.trim_start_matches('/'))
        }
        None => PathBuf::from(path),
    };
    if path.is_absolute() {
        return path;
    }
    let start = std::env::var_os("BUILD_WORKING_DIRECTORY")
        .map(PathBuf::from)
        .or_else(|| std::env::current_dir().ok())
        .unwrap_or_default();
    start.join(path)
}

/// `path` as given if it exists here (in Bazel's runfiles, for the defaults), or else from
/// where the user started it.
fn locate(path: &Path) -> PathBuf {
    if path.as_os_str().is_empty() || path.is_absolute() || path.exists() {
        return path.to_path_buf();
    }
    expand(&path.to_string_lossy())
}

fn main() {
    let flags = Flags::parse();
    if std::env::var_os("RUST_LOG").is_none() {
        // SAFETY: nothing else runs yet.
        unsafe { std::env::set_var("RUST_LOG", "info") };
    }
    let _ = env_logger_init();

    let ca = flags.robot_ca.as_deref().map(locate);
    let robot = logs::RobotClient::new(&flags.robot, ca.as_deref())
        .unwrap_or_else(|error| panic!("{error}"));
    let log_dir = expand(&flags.log_dir);
    std::fs::create_dir_all(&log_dir)
        .unwrap_or_else(|error| panic!("Can't make {}: {error}", log_dir.display()));
    let log_manager = std::fs::canonicalize(locate(&flags.log_manager))
        .unwrap_or_else(|error| panic!("Can't find {}: {error}", flags.log_manager.display()));
    let data_dir = std::fs::canonicalize(locate(&flags.data_dir))
        .unwrap_or_else(|error| panic!("Can't find {}: {error}", flags.data_dir.display()));

    let mut options =
        logs::LogsOptions::new(log_dir.clone(), log_manager, logs::Mode::Laptop { robot });
    options.presets = flags.presets.as_deref().map(locate);
    let logs = logs::Logs::new(options);

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("Failed to start tokio");
    runtime.block_on(async move {
        let listener = tokio::net::TcpListener::bind((flags.listen.as_str(), flags.port))
            .await
            .unwrap_or_else(|error| {
                panic!("Can't listen on {}:{}: {error}", flags.listen, flags.port)
            });
        println!(
            "Serving the logs in {} on http://localhost:{}/, from the robot at {}",
            log_dir.display(),
            flags.port,
            flags.robot
        );
        tokio::spawn(logs::run_jobs(logs.clone()));
        robot_server::serve_http(listener, logs::laptop_router(data_dir, logs)).await;
    });
}

/// Logs to stderr, without pulling in a logging crate: prints what `log` is given at INFO and
/// above.
fn env_logger_init() -> Result<(), log::SetLoggerError> {
    struct Stderr;
    impl log::Log for Stderr {
        fn enabled(&self, metadata: &log::Metadata) -> bool {
            metadata.level() <= log::Level::Info
        }
        fn log(&self, record: &log::Record) {
            if self.enabled(record.metadata()) {
                eprintln!("{} {}", record.level(), record.args());
            }
        }
        fn flush(&self) {}
    }
    log::set_logger(&Stderr).map(|()| log::set_max_level(log::LevelFilter::Info))
}
