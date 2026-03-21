use rand::Rng;

use crate::collision::{point_collides, segment_collides};
use crate::config::RrtConfig;
use crate::geometry::{Point2D, Workspace};

#[derive(Debug, Clone)]
pub struct Node {
    pub id: usize,
    pub position: Point2D,
    pub parent: Option<usize>,
    pub cost: f64,
}

#[derive(Debug)]
pub struct RrtPlanner {
    pub config: RrtConfig,
    pub workspace: Workspace,
    nodes: Vec<Node>,
}

impl RrtPlanner {
    pub fn new(config: RrtConfig, workspace: Workspace) -> Self {
        Self {
            config,
            workspace,
            nodes: Vec::new(),
        }
    }

    /// Run basic RRT algorithm.
    pub fn plan(&mut self, start: Point2D, goal: Point2D) -> Option<Vec<Point2D>> {
        self.nodes.clear();
        self.nodes.push(Node {
            id: 0,
            position: start,
            parent: None,
            cost: 0.0,
        });

        let mut rng = rand::thread_rng();

        for _ in 0..self.config.max_iterations {
            let sample = self.sample_point(&mut rng, &goal);
            let nearest_id = self.find_nearest(&sample);
            let nearest_pos = self.nodes[nearest_id].position;
            let new_pos = nearest_pos.steer_toward(&sample, self.config.step_size);

            if point_collides(&new_pos, &self.workspace) {
                continue;
            }
            if segment_collides(&nearest_pos, &new_pos, &self.workspace) {
                continue;
            }

            let new_cost = self.nodes[nearest_id].cost + nearest_pos.distance_to(&new_pos);
            let new_id = self.nodes.len();
            self.nodes.push(Node {
                id: new_id,
                position: new_pos,
                parent: Some(nearest_id),
                cost: new_cost,
            });

            if new_pos.distance_to(&goal) <= self.config.goal_tolerance {
                return Some(self.extract_path(new_id));
            }
        }

        None
    }

    /// Run RRT* algorithm with rewiring for optimal paths.
    pub fn plan_star(&mut self, start: Point2D, goal: Point2D) -> Option<Vec<Point2D>> {
        self.nodes.clear();
        self.nodes.push(Node {
            id: 0,
            position: start,
            parent: None,
            cost: 0.0,
        });

        let mut rng = rand::thread_rng();
        let mut best_goal_node: Option<usize> = None;
        let mut best_goal_cost = f64::INFINITY;

        for _ in 0..self.config.max_iterations {
            let sample = self.sample_point(&mut rng, &goal);
            let nearest_id = self.find_nearest(&sample);
            let nearest_pos = self.nodes[nearest_id].position;
            let new_pos = nearest_pos.steer_toward(&sample, self.config.step_size);

            if point_collides(&new_pos, &self.workspace) {
                continue;
            }
            if segment_collides(&nearest_pos, &new_pos, &self.workspace) {
                continue;
            }

            // Find neighbors within rewire radius
            let neighbor_ids = self.find_neighbors(&new_pos, self.config.rewire_radius);

            // Choose best parent among neighbors
            let mut best_parent = nearest_id;
            let mut best_cost =
                self.nodes[nearest_id].cost + nearest_pos.distance_to(&new_pos);

            for &nid in &neighbor_ids {
                let n_pos = self.nodes[nid].position;
                let candidate_cost = self.nodes[nid].cost + n_pos.distance_to(&new_pos);
                if candidate_cost < best_cost
                    && !segment_collides(&n_pos, &new_pos, &self.workspace)
                {
                    best_parent = nid;
                    best_cost = candidate_cost;
                }
            }

            let new_id = self.nodes.len();
            self.nodes.push(Node {
                id: new_id,
                position: new_pos,
                parent: Some(best_parent),
                cost: best_cost,
            });

            // Rewire: check if routing through new node improves neighbors
            for &nid in &neighbor_ids {
                let n_pos = self.nodes[nid].position;
                let candidate_cost = best_cost + new_pos.distance_to(&n_pos);
                if candidate_cost < self.nodes[nid].cost
                    && !segment_collides(&new_pos, &n_pos, &self.workspace)
                {
                    self.nodes[nid].parent = Some(new_id);
                    self.nodes[nid].cost = candidate_cost;
                }
            }

            if new_pos.distance_to(&goal) <= self.config.goal_tolerance
                && best_cost < best_goal_cost
            {
                best_goal_node = Some(new_id);
                best_goal_cost = best_cost;
            }
        }

        best_goal_node.map(|id| self.extract_path(id))
    }

    fn sample_point(&self, rng: &mut impl Rng, goal: &Point2D) -> Point2D {
        if rng.gen::<f64>() < self.config.goal_bias {
            *goal
        } else {
            Point2D::new(
                rng.gen_range(self.workspace.bounds_min.x..self.workspace.bounds_max.x),
                rng.gen_range(self.workspace.bounds_min.y..self.workspace.bounds_max.y),
            )
        }
    }

    fn find_nearest(&self, point: &Point2D) -> usize {
        self.nodes
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.position
                    .distance_to(point)
                    .partial_cmp(&b.position.distance_to(point))
                    .unwrap()
            })
            .map(|(i, _)| i)
            .unwrap()
    }

    fn find_neighbors(&self, point: &Point2D, radius: f64) -> Vec<usize> {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| node.position.distance_to(point) <= radius)
            .map(|(i, _)| i)
            .collect()
    }

    fn extract_path(&self, goal_node_id: usize) -> Vec<Point2D> {
        let mut path = Vec::new();
        let mut current = Some(goal_node_id);
        while let Some(id) = current {
            path.push(self.nodes[id].position);
            current = self.nodes[id].parent;
        }
        path.reverse();
        path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_workspace() -> Workspace {
        Workspace::new(Point2D::new(0.0, 0.0), Point2D::new(20.0, 20.0))
    }

    #[test]
    fn test_rrt_open_space() {
        let config = RrtConfig {
            step_size: 1.0,
            max_iterations: 5000,
            goal_bias: 0.1,
            goal_tolerance: 1.0,
            ..RrtConfig::default()
        };
        let mut planner = RrtPlanner::new(config, open_workspace());
        let path = planner.plan(Point2D::new(1.0, 1.0), Point2D::new(19.0, 19.0));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(path.len() >= 2);
    }

    #[test]
    fn test_rrt_star_open_space() {
        let config = RrtConfig {
            step_size: 1.0,
            max_iterations: 5000,
            goal_bias: 0.1,
            goal_tolerance: 1.0,
            ..RrtConfig::default()
        };
        let mut planner = RrtPlanner::new(config, open_workspace());
        let path = planner.plan_star(Point2D::new(1.0, 1.0), Point2D::new(19.0, 19.0));
        assert!(path.is_some());
    }

    #[test]
    fn test_rrt_avoids_obstacle() {
        let mut ws = open_workspace();
        ws.add_obstacle(crate::geometry::Obstacle::Circle {
            center: Point2D::new(10.0, 10.0),
            radius: 3.0,
        });
        let config = RrtConfig {
            step_size: 1.0,
            max_iterations: 10000,
            goal_bias: 0.1,
            goal_tolerance: 1.0,
            ..RrtConfig::default()
        };
        let mut planner = RrtPlanner::new(config, ws);
        let path = planner.plan(Point2D::new(1.0, 10.0), Point2D::new(19.0, 10.0));
        assert!(path.is_some());
    }
}
