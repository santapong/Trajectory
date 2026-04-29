//! Inject the current git commit SHA as a compile-time env var so emitted
//! RAPID programs can be traced back to an exact commit, not just the
//! crate's semver. Falls back gracefully when git is unavailable (e.g.
//! when published as a crate or built outside a git checkout).

use std::process::Command;

fn main() {
    let sha = Command::new("git")
        .args(["rev-parse", "--short=12", "HEAD"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                String::from_utf8(o.stdout).ok()
            } else {
                None
            }
        })
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string());

    println!("cargo:rustc-env=TRAJECTORY_GIT_SHA={sha}");

    // Re-run if HEAD moves.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs/heads");
}
