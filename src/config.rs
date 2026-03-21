use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RrtConfig {
    /// Maximum extension distance per step.
    pub step_size: f64,
    /// Maximum number of iterations before giving up.
    pub max_iterations: usize,
    /// Probability (0.0-1.0) of sampling the goal directly.
    pub goal_bias: f64,
    /// Distance threshold to consider the goal reached.
    pub goal_tolerance: f64,
    /// Neighborhood radius for RRT* rewiring.
    pub rewire_radius: f64,
    /// Number of path smoothing passes.
    pub smoothing_iterations: usize,
}

impl Default for RrtConfig {
    fn default() -> Self {
        Self {
            step_size: 0.5,
            max_iterations: 5000,
            goal_bias: 0.05,
            goal_tolerance: 0.5,
            rewire_radius: 2.0,
            smoothing_iterations: 100,
        }
    }
}
