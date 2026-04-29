//! Provenance metadata embedded in emitted CNC programs.
//!
//! UAT operators need to trace a RAPID program back to the exact inputs that
//! produced it: the source PNG, the options, the tool/frame configs, and the
//! commit/version of this code. [`JobMetadata`] captures that record. The
//! post-processor writes it as a RAPID comment block at the top of the
//! emitted MODULE.

use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use sha2::{Digest, Sha256};

#[derive(Debug, Clone)]
pub struct JobMetadata {
    pub crate_version: &'static str,
    pub git_sha: &'static str,
    /// Unix-epoch seconds at the moment the job ran.
    pub timestamp_unix: u64,
    /// SHA-256 of the source heightmap (or other input artifact), hex.
    pub source_sha256: Option<String>,
    /// Path of the source heightmap, for human reference.
    pub source_path: Option<String>,
    /// Serialized HeightMapOpts JSON (or whatever opts the job ran with).
    pub opts_json: Option<String>,
    /// Path of the tool config.
    pub tool_path: Option<String>,
    /// Path of the workpiece frame config.
    pub frame_path: Option<String>,
    /// Free-form note (e.g. "UAT run #42, operator AB").
    pub note: Option<String>,
}

impl Default for JobMetadata {
    fn default() -> Self {
        Self {
            crate_version: env!("CARGO_PKG_VERSION"),
            git_sha: option_env!("TRAJECTORY_GIT_SHA").unwrap_or("unknown"),
            timestamp_unix: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0),
            source_sha256: None,
            source_path: None,
            opts_json: None,
            tool_path: None,
            frame_path: None,
            note: None,
        }
    }
}

impl JobMetadata {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_source_file<P: AsRef<Path>>(mut self, path: P) -> std::io::Result<Self> {
        let bytes = std::fs::read(&path)?;
        self.source_sha256 = Some(sha256_hex(&bytes));
        self.source_path = Some(path.as_ref().display().to_string());
        Ok(self)
    }

    pub fn with_source_bytes(mut self, label: impl Into<String>, bytes: &[u8]) -> Self {
        self.source_sha256 = Some(sha256_hex(bytes));
        self.source_path = Some(label.into());
        self
    }

    pub fn with_opts_json(mut self, json: impl Into<String>) -> Self {
        self.opts_json = Some(json.into());
        self
    }

    pub fn with_tool_path(mut self, p: impl Into<String>) -> Self {
        self.tool_path = Some(p.into());
        self
    }

    pub fn with_frame_path(mut self, p: impl Into<String>) -> Self {
        self.frame_path = Some(p.into());
        self
    }

    pub fn with_note(mut self, note: impl Into<String>) -> Self {
        self.note = Some(note.into());
        self
    }

    /// Render as RAPID `!` comment lines, indented two spaces. Each line ends
    /// with `\n` so the caller can prepend it to a MODULE body verbatim.
    pub fn to_rapid_comment(&self) -> String {
        let mut out = String::new();
        out.push_str("  ! ----- trajectory provenance -----\n");
        out.push_str(&format!("  ! version       : {}\n", self.crate_version));
        out.push_str(&format!("  ! git_sha       : {}\n", self.git_sha));
        out.push_str(&format!("  ! timestamp_utc : {} (unix)\n", self.timestamp_unix));
        if let Some(s) = &self.source_path {
            out.push_str(&format!("  ! source_path   : {s}\n"));
        }
        if let Some(s) = &self.source_sha256 {
            out.push_str(&format!("  ! source_sha256 : {s}\n"));
        }
        if let Some(s) = &self.tool_path {
            out.push_str(&format!("  ! tool_path     : {s}\n"));
        }
        if let Some(s) = &self.frame_path {
            out.push_str(&format!("  ! frame_path    : {s}\n"));
        }
        if let Some(s) = &self.opts_json {
            for line in s.lines() {
                out.push_str(&format!("  ! opts          : {line}\n"));
            }
        }
        if let Some(s) = &self.note {
            for line in s.lines() {
                out.push_str(&format!("  ! note          : {line}\n"));
            }
        }
        out.push_str("  ! ----------------------------------\n");
        out
    }
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    let mut s = String::with_capacity(64);
    for b in digest {
        s.push_str(&format!("{:02x}", b));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_of_empty_string_is_known_constant() {
        assert_eq!(
            sha256_hex(b""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
    }

    #[test]
    fn comment_block_includes_required_fields() {
        let m = JobMetadata::new()
            .with_source_bytes("relief.png", b"hello")
            .with_opts_json(r#"{"max_depth_mm":4.0}"#);
        let c = m.to_rapid_comment();
        assert!(c.contains("source_path   : relief.png"));
        assert!(c.contains("source_sha256 : 2cf24d"));
        assert!(c.contains("opts          : {\"max_depth_mm\":4.0}"));
        assert!(c.contains("version       :"));
    }
}
