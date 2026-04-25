//! Tool definition: type, dimensions, and TCP offset from the flange.
//!
//! Loaded from JSON at runtime. The TCP offset is what the post-processor
//! emits as `tooldata` (RAPID), `$TOOL` (KUKA), or G10 L1 (G-code) so the
//! controller applies it as RTCP/TCPM/G43.

use std::path::Path;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ToolType {
    Ball,
    Flat,
    Conical,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TcpOffset {
    pub xyz: [f64; 3],
    #[serde(default)]
    pub rpy_deg: Option<[f64; 3]>,
}

impl Default for TcpOffset {
    fn default() -> Self {
        Self {
            xyz: [0.0, 0.0, 0.0],
            rpy_deg: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tool {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: ToolType,
    pub diameter_mm: f64,
    pub length_mm: f64,
    pub tcp: TcpOffset,
    #[serde(default)]
    pub cone_half_angle_deg: Option<f64>,
}

impl Tool {
    pub fn load_json<P: AsRef<Path>>(p: P) -> std::io::Result<Self> {
        let bytes = std::fs::read(p)?;
        serde_json::from_slice(&bytes)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }

    pub fn radius_mm(&self) -> f64 {
        self.diameter_mm * 0.5
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_minimal_tool() {
        let json = r#"{
            "name": "Ball6",
            "type": "ball",
            "diameter_mm": 6.0,
            "length_mm": 80.0,
            "tcp": { "xyz": [0, 0, 80] }
        }"#;
        let t: Tool = serde_json::from_str(json).expect("parse");
        assert_eq!(t.name, "Ball6");
        assert_eq!(t.kind, ToolType::Ball);
        assert!((t.diameter_mm - 6.0).abs() < 1e-9);
        assert!((t.radius_mm() - 3.0).abs() < 1e-9);
        assert_eq!(t.tcp.xyz, [0.0, 0.0, 80.0]);
    }

    #[test]
    fn parse_conical_tool() {
        let json = r#"{
            "name": "Conical10",
            "type": "conical",
            "diameter_mm": 10.0,
            "length_mm": 60.0,
            "tcp": { "xyz": [0, 0, 60] },
            "cone_half_angle_deg": 15.0
        }"#;
        let t: Tool = serde_json::from_str(json).unwrap();
        assert_eq!(t.kind, ToolType::Conical);
        assert!((t.cone_half_angle_deg.unwrap() - 15.0).abs() < 1e-9);
    }
}
