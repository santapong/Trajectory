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

#[cfg(feature = "abb")]
pub mod provenance;

#[cfg(feature = "abb")]
pub use provenance::JobMetadata;

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
    /// Optional provenance block emitted as RAPID comments before the MODULE
    /// body. Lets a UAT operator trace any program back to its inputs.
    #[cfg(feature = "abb")]
    pub metadata: Option<&'a JobMetadata>,
}

pub trait PostProcessor {
    fn emit(&self, path: &Toolpath, ctx: &PostContext<'_>) -> Result<String, PostError>;
}
