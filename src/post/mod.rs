//! Post-processor framework. A [`PostProcessor`] consumes a [`Toolpath`] plus
//! a [`PostContext`] and emits a machine-specific program string.
//!
//! [`Toolpath`]: crate::toolpath::Toolpath

use crate::frame::WorkpieceFrame;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

#[cfg(feature = "abb")]
pub mod abb_rapid;

#[cfg(feature = "abb")]
pub use abb_rapid::AbbRapidPost;

#[derive(Debug, thiserror::Error)]
pub enum PostError {
    #[error("template error: {0}")]
    Template(String),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
}

#[derive(Debug, Clone)]
pub struct PostContext<'a> {
    pub program_name: &'a str,
    pub tool: &'a Tool,
    pub frame: &'a WorkpieceFrame,
    pub feedrate_mm_min: f64,
    pub rapid_speed: f64,
}

pub trait PostProcessor {
    fn emit(&self, path: &Toolpath, ctx: &PostContext<'_>) -> Result<String, PostError>;
}
