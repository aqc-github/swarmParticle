/* 
    This file contains the definitions for the swarm elements.
    The swarm is a collection of Drones.
    Each Drone is composed of a real Worker and a number M of virtual Workers (particles).
*/

use piston_window::{Context, Graphics, Polygon};
use piston_window::ellipse::Ellipse;
use piston_window::line::Line;
use piston_window::types::Color;
use rand::Rng;

// Define map dimensions as a constant
const DIMENSIONS: (f64, f64) = (700.0, 700.0);

/// Represents the environment map with obstacles.
#[derive(Clone)]
pub struct Map {
    /// 2D grid where 1 indicates an obstacle and 0 indicates free space.
    pub matrix: Vec<Vec<u8>>,
    /// Dimensions of the map (width, height).
    pub dimensions: (f64, f64),
    /// Size of each cell in the grid.
    pub cell_size: f64,
}

impl Map {
    /// Creates a new map with the specified dimensions and cell size.
    pub fn new(dimensions: (f64, f64), cell_size: f64) -> Self {
        let rows = (dimensions.1 / cell_size).ceil() as usize;
        let cols = (dimensions.0 / cell_size).ceil() as usize;
        Map {
            matrix: vec![vec![0; cols]; rows],
            dimensions,
            cell_size,
        }
    }

    /// Converts world coordinates to grid indices.
    pub fn world_to_grid(&self, x: f64, y: f64) -> (usize, usize) {
        let row = (y / self.cell_size).floor() as usize;
        let col = (x / self.cell_size).floor() as usize;
        (row, col)
    }

    /// Checks if a position contains an obstacle or is out of bounds.
    pub fn is_obstacle(&self, x: f64, y: f64) -> bool {
        let (row, col) = self.world_to_grid(x, y);
        if row < self.matrix.len() && col < self.matrix[0].len() {
            self.matrix[row][col] == 1
        } else {
            true // Out-of-bounds is treated as an obstacle
        }
    }

    /// Simulates a sensor reading in a given direction up to a maximum distance.
    pub fn get_sensor_reading(&self, x: f64, y: f64, angle: f64, max_distance: f64) -> f64 {
        let mut distance = 0.0;
        let step = self.cell_size / 4.0; // Finer step size for improved resolution
        
        while distance < max_distance {
            let check_x = x + distance * angle.cos();
            let check_y = y + distance * angle.sin();
            
            if self.is_obstacle(check_x, check_y) {
                return distance;
            }
            
            distance += step;
        }
        
        max_distance // Return max distance if no obstacle is encountered
    }
}

/// Represents a worker, either the real drone or a virtual particle in MCL.
#[derive(Clone)]
pub struct Worker {
    /// Estimated position and orientation (x, y, theta in radians).
    pub position_estimate: (f64, f64, f64),
    /// Vision matrix representing the surroundings (50x50 grid)
    pub vision_matrix: Vec<Vec<u8>>,
    /// Weight of the particle, used in MCL for virtual workers.
    pub weight: f64,
}

impl Worker {
    /// Calculates the points of a triangle for rendering the worker.
    pub fn calculate_triangle_points(&self) -> Vec<(f64, f64)> {
        let size = 5.0; // Size of the triangle for visualization
        let (x, y, theta) = self.position_estimate;
        let front_point = (x + size * theta.cos(), y + size * theta.sin());
        let back_left = (x + size * (theta - 2.4).cos(), y + size * (theta - 2.4).sin());
        let back_right = (x + size * (theta + 2.4).cos(), y + size * (theta + 2.4).sin());
        vec![front_point, back_left, back_right]
    }

    /// Updates vision matrix based on the worker's position and the map.
    pub fn update_vision_matrix(&mut self, map: &Map) {
        let (x, y, _) = self.position_estimate;
        let vision_size = 25; // Half the size of the vision matrix (50/2)
        
        // Initialize vision matrix
        self.vision_matrix = vec![vec![0; 50]; 50];
        
        // Fill vision matrix with map data
        for i in 0..50 {
            for j in 0..50 {
                // Convert vision matrix coordinates to world coordinates
                let world_x = x + (j as f64 - vision_size as f64) * map.cell_size;
                let world_y = y + (i as f64 - vision_size as f64) * map.cell_size;
                
                // Check if the position is an obstacle
                if map.is_obstacle(world_x, world_y) {
                    self.vision_matrix[i][j] = 1;
                }
            }
        }
    }

    /// Updates the worker's location with bearing and distance changes.
    /// Returns true if a collision occurs (only checked for real workers).
    pub fn update_location(&mut self, bearing_update: f64, distance_update: f64, is_real_worker: bool) -> bool {
        let (x, y, theta) = self.position_estimate;
        
        // Calculate new position based on movement
        let new_x = x + distance_update * (theta + bearing_update).cos();
        let new_y = y + distance_update * (theta + bearing_update).sin();
        let new_theta = theta + bearing_update;
        
        let mut collision = false;
        
        if is_real_worker {
            let margin = 50.0; // Boundary margin for real workers
            let bounded_x = new_x.max(margin).min(DIMENSIONS.0 - margin);
            let bounded_y = new_y.max(margin).min(DIMENSIONS.1 - margin);
            if bounded_x != new_x || bounded_y != new_y {
                // Collision with boundary: turn 180 degrees
                self.position_estimate = (bounded_x, bounded_y, theta + std::f64::consts::PI);
                collision = true;
            } else {
                self.position_estimate = (new_x, new_y, new_theta);
            }
        } else {
            // For particles, allow full map range without collision checks
            self.position_estimate = (new_x.max(0.0).min(DIMENSIONS.0), 
                                     new_y.max(0.0).min(DIMENSIONS.1), 
                                     new_theta);
        }
        collision
    }

    /// Updates the worker's weight with the provided value.
    pub fn update_weight(&mut self, weight_update: f64) {
        self.weight = weight_update;
    }
}

/// Represents a drone with a real worker and a set of particles for localization.
#[derive(Clone)]
pub struct Drone {
    /// Unique identifier for the drone.
    pub id: usize,
    /// RGB color for rendering the drone and its particles.
    pub color: [f32; 3],
    /// The real worker representing the drone's actual position.
    pub worker: Worker,
    /// Vector of particles representing possible positions in MCL.
    pub particles: Vec<Worker>,
    /// Relative poses to other drones (id, distance, bearing).
    pub relative_poses: Vec<(usize, f64, f64)>,
}

impl Drone {
    /// Creates a new drone with a real worker and a specified number of particles.
    pub fn new(id: usize, color: [f32; 3], num_particles: usize) -> Self {
        let mut rng = rand::thread_rng();
        let margin = 100.0; // Margin from borders for real worker spawn

        // Initialize real worker with a position away from borders
        let real_worker = Worker {
            position_estimate: (
                rng.gen_range(margin..DIMENSIONS.0 - margin),
                rng.gen_range(margin..DIMENSIONS.1 - margin),
                rng.gen_range(0.0..2.0 * std::f64::consts::PI),
            ),
            vision_matrix: Vec::new(),
            weight: 1.0,
        };

        // Initialize particles randomly across the entire map
        let mut particles = Vec::with_capacity(num_particles);
        for _ in 0..num_particles {
            particles.push(Worker {
                position_estimate: (
                    rng.gen_range(0.0..DIMENSIONS.0),
                    rng.gen_range(0.0..DIMENSIONS.1),
                    rng.gen_range(0.0..2.0 * std::f64::consts::PI),
                ),
                vision_matrix: Vec::new(),
                weight: 1.0,
            });
        }

        Drone {
            id,
            color,
            worker: real_worker,
            particles,
            relative_poses: Vec::new(),
        }
    }

    /// Renders the drone and its particles on the graphics context.
    pub fn render(&self, context: &Context, graphics: &mut impl Graphics, is_updating: bool, all_drones: &[Drone]) {
        let (x, y, _) = self.worker.position_estimate;
        
        // Render particles with lower opacity
        let particle_color: Color = [self.color[0], self.color[1], self.color[2], 0.2];
        let particle_triangle = Polygon::new(particle_color);
        for particle in &self.particles {
            let points = particle.calculate_triangle_points();
            let polygon: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
            particle_triangle.draw(&polygon, &context.draw_state, context.transform, graphics);
        }
        
        // Render real worker with full opacity
        let worker_color: Color = [self.color[0], self.color[1], self.color[2], 1.0];
        let worker_triangle = Polygon::new(worker_color);
        let points = self.worker.calculate_triangle_points();
        let polygon: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
        worker_triangle.draw(&polygon, &context.draw_state, context.transform, graphics);
        
        // Draw orange circumference around real worker
        let circle_color: Color = [1.0, 0.5, 0.0, 1.0]; // Orange, full opacity
        let radius = 8.0;
        let num_segments = 32;
        let angle_step = 2.0 * std::f64::consts::PI / num_segments as f64;
        
        for i in 0..num_segments {
            let angle1 = i as f64 * angle_step;
            let angle2 = (i + 1) as f64 * angle_step;
            let x1 = x + radius * angle1.cos();
            let y1 = y + radius * angle1.sin();
            let x2 = x + radius * angle2.cos();
            let y2 = y + radius * angle2.sin();
            Line::new(circle_color, 1.0)
                .draw([x1, y1, x2, y2], &context.draw_state, context.transform, graphics);
        }
        
        // Draw update indicator if currently updating
        if is_updating {
            let indicator_color: Color = [1.0, 1.0, 0.0, 0.3]; // Yellow, semi-transparent
            let indicator_ellipse = Ellipse::new(indicator_color);
            indicator_ellipse.draw(
                [x - 10.0, y - 10.0, 20.0, 20.0],
                &context.draw_state,
                context.transform,
                graphics,
            );
        }
        
        // Draw lines to indicate relative poses to other drones
        for (target_id, _distance, _bearing) in &self.relative_poses {
            if let Some(target) = all_drones.iter().find(|d| d.id == *target_id) {
                let (tx, ty, _) = target.worker.position_estimate;
                let line_color: Color = [0.5, 0.5, 0.5, 0.2]; // Gray, semi-transparent
                Line::new(line_color, 0.5)
                    .draw([x, y, tx, ty], &context.draw_state, context.transform, graphics);
            }
        }
    }

    /// Updates the drone's motion and sensor readings based on movement inputs.
    pub fn motion_update(&mut self, map: &Map, bearing_update: f64, distance_update: f64) {
        // Update real worker and check for collisions
        let collision = self.worker.update_location(bearing_update, distance_update, true);
        
        // Adjust bearing for particles based on real worker collision
        let effective_bearing = if collision {
            bearing_update + std::f64::consts::PI // Turn 180 degrees
        } else {
            bearing_update
        };
        
        // Update particles with controlled noise
        let mut rng = rand::thread_rng();
        for particle in &mut self.particles {
            // Add noise to bearing and distance for exploration
            let noisy_bearing = effective_bearing + rng.gen_range(-0.1..0.1); // ±5.7 degrees
            let noisy_distance = distance_update + rng.gen_range(-2.0..2.0); // ±2 units
            
            // Occasionally kidnap a particle to prevent local minima (1% chance)
            if rng.gen_bool(0.01) {
                particle.position_estimate = (
                    rng.gen_range(0.0..DIMENSIONS.0),
                    rng.gen_range(0.0..DIMENSIONS.1),
                    rng.gen_range(0.0..2.0 * std::f64::consts::PI),
                );
            } else {
                particle.update_location(noisy_bearing, noisy_distance, false);
            }
        }
        
        // Update vision matrices for real worker and particles
        self.worker.update_vision_matrix(map);
        for particle in &mut self.particles {
            particle.update_vision_matrix(map);
        }
    }

    /// Updates the belief state of the drone using MCL: weights and resampling.
    pub fn belief_update(&mut self) {
        // Calculate weights based on vision matrix similarity
        for particle in &mut self.particles {
            let error: f64 = particle.vision_matrix.iter()
                .zip(self.worker.vision_matrix.iter())
                .map(|(row1, row2)| {
                    row1.iter()
                        .zip(row2.iter())
                        .map(|(a, b)| (*a as f64 - *b as f64).powi(2))
                        .sum::<f64>()
                })
                .sum();
            
            let sigma = 10.0; // Tuned for vision matrix comparison
            let weight = (-error / (2.0 * sigma * sigma)).exp();
            particle.update_weight(weight);
        }
        
        // Normalize weights to sum to 1
        let sum_weights: f64 = self.particles.iter().map(|p| p.weight).sum();
        if sum_weights > 0.0 { // Prevent division by zero
            for particle in &mut self.particles {
                particle.weight /= sum_weights;
            }
        }
        
        // Perform systematic resampling based on weights
        let num_particles = self.particles.len();
        let mut new_particles = Vec::with_capacity(num_particles);
        let mut rng = rand::thread_rng();
        
        // Compute cumulative weights
        let mut cumulative_weights = Vec::with_capacity(num_particles);
        let mut sum = 0.0;
        for particle in &self.particles {
            sum += particle.weight;
            cumulative_weights.push(sum);
        }
        
        // Systematic resampling algorithm
        let step = 1.0 / num_particles as f64;
        let mut u = rng.gen_range(0.0..step);
        let mut j = 0;
        
        for _ in 0..num_particles {
            while j < num_particles - 1 && u > cumulative_weights[j] {
                j += 1;
            }
            new_particles.push(self.particles[j].clone());
            u += step;
        }
        
        self.particles = new_particles;
    }

    /// Updates vision matrices for the real worker and all particles.
    pub fn update_sensor_readings(&mut self, map: &Map) {
        self.worker.update_vision_matrix(map);
        for particle in &mut self.particles {
            particle.update_vision_matrix(map);
        }
    }
}

/// Represents a collection of drones forming the swarm.
#[derive(Clone)]
pub struct Swarm {
    /// Vector containing all drones in the swarm.
    pub drone_list: Vec<Drone>,
}

impl Swarm {
    /// Renders all drones in the swarm.
    pub fn render(&self, context: &Context, graphics: &mut impl Graphics, is_updating: bool) {
        for drone in &self.drone_list {
            drone.render(context, graphics, is_updating, &self.drone_list);
        }
    }

    /// Updates the relative poses of all drones in the swarm.
    pub fn update_relative_poses(&mut self) {
        for i in 0..self.drone_list.len() {
            let mut relative_poses = Vec::new();
            
            // Calculate poses relative to other drones
            for j in 0..self.drone_list.len() {
                if i != j {
                    let (x1, y1, _) = self.drone_list[i].worker.position_estimate;
                    let (x2, y2, _) = self.drone_list[j].worker.position_estimate;
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    let distance = (dx * dx + dy * dy).sqrt();
                    let bearing = dy.atan2(dx);
                    relative_poses.push((j, distance, bearing));
                }
            }
            
            self.drone_list[i].relative_poses = relative_poses;
        }
    }

    /// Updates sensor readings for all drones in the swarm.
    pub fn update_sensor_readings(&mut self, map: &Map) {
        for drone in &mut self.drone_list {
            drone.update_sensor_readings(map);
        }
    }

    /// Moves the swarm around the map and updates beliefs.
    pub fn move_swarm(&mut self, map: &Map) {
        let mut rng = rand::thread_rng();
        for drone in &mut self.drone_list {
            let bearing_update = rng.gen_range(-0.05..0.05); // Small bearing change
            let distance_update = 15.0; // Consistent movement distance
            drone.motion_update(map, bearing_update, distance_update);
            drone.belief_update();
        }
    }
}