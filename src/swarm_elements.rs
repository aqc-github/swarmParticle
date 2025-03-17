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

// Add dimensions constant
const DIMENSIONS: (f64, f64) = (700.0, 700.0);

// STRUCTURE FOR THE MAP
#[derive(Clone)]
pub struct Map {
    pub matrix: Vec<Vec<u8>>, // 1 for obstacle, 0 for free space
    pub dimensions: (f64, f64),
    pub cell_size: f64, // Size of each cell in the matrix
}

impl Map {
    pub fn new(dimensions: (f64, f64), cell_size: f64) -> Self {
        let rows = (dimensions.1 / cell_size).ceil() as usize;
        let cols = (dimensions.0 / cell_size).ceil() as usize;
        Map {
            matrix: vec![vec![0; cols]; rows],
            dimensions,
            cell_size,
        }
    }

    pub fn world_to_grid(&self, x: f64, y: f64) -> (usize, usize) {
        let row = (y / self.cell_size).floor() as usize;
        let col = (x / self.cell_size).floor() as usize;
        (row, col)
    }

    pub fn is_obstacle(&self, x: f64, y: f64) -> bool {
        let (row, col) = self.world_to_grid(x, y);
        if row < self.matrix.len() && col < self.matrix[0].len() {
            self.matrix[row][col] == 1
        } else {
            true // Consider out of bounds as obstacle
        }
    }

    pub fn get_sensor_reading(&self, x: f64, y: f64, angle: f64, max_distance: f64) -> f64 {
        let mut distance = 0.0;
        let step = self.cell_size / 2.0; // Smaller step for more accurate readings
        
        while distance < max_distance {
            let check_x = x + distance * angle.cos();
            let check_y = y + distance * angle.sin();
            
            if self.is_obstacle(check_x, check_y) {
                return distance;
            }
            
            distance += step;
        }
        
        max_distance
    }
}

// STRUCTURE FOR THE WORKER
#[derive(Clone)]
pub struct Worker {
    pub position_estimate: (f64, f64, f64), // (x, y, theta)
    pub sensor_reach: f64, // The range of the sensor, LIDAR.
    pub sensor_reading: Vec<f64>, // Vector of N sensor readings, each reading (0, sensor_reach]
    pub weight: f64, // Weight of the particle
}

impl Worker {
    pub fn calculate_triangle_points(&self) -> Vec<(f64, f64)> {
        let size = 5.0;
        let (x, y, theta) = self.position_estimate;
        let front_point = (
            x + size * theta.cos(),
            y + size * theta.sin()
        );
        let back_left = (
            x + size * (theta - 2.4).cos(),
            y + size * (theta - 2.4).sin()
        );
        let back_right = (
            x + size * (theta + 2.4).cos(),
            y + size * (theta + 2.4).sin()
        );
        vec![front_point, back_left, back_right]
    }

    pub fn update_sensor_readings(&mut self, map: &Map) {
        let (x, y, theta) = self.position_estimate;
        let num_readings = 8; // Number of sensor readings in different directions
        let angle_step = 2.0 * std::f64::consts::PI / num_readings as f64;
        
        self.sensor_reading.clear();
        for i in 0..num_readings {
            let angle = theta + i as f64 * angle_step;
            let reading = map.get_sensor_reading(x, y, angle, self.sensor_reach);
            self.sensor_reading.push(reading);
        }
    }

    pub fn update_location(&mut self) {
        let mut rng = rand::thread_rng();
        let (x, y, theta) = self.position_estimate;
        
        // Generate random movement with larger step size but smaller bearing changes
        let step_size = 15.0; // Increased from 5.0 for more movement
        let bearing = rng.gen_range(-std::f64::consts::PI/8.0..std::f64::consts::PI/8.0); // Reduced from PI/4.0 for less randomness
        let distance = rng.gen_range(0.0..step_size);
        
        // Calculate new position
        let new_x = x + distance * (theta + bearing).cos();
        let new_y = y + distance * (theta + bearing).sin();
        
        // Keep within boundaries with 50px margin
        let margin = 50.0;
        let bounded_x = new_x.max(margin).min(DIMENSIONS.0 - margin);
        let bounded_y = new_y.max(margin).min(DIMENSIONS.1 - margin);
        
        // If we hit a boundary, turn around
        let mut new_theta = theta + bearing;
        if new_x < margin || new_x > DIMENSIONS.0 - margin || new_y < margin || new_y > DIMENSIONS.1 - margin {
            new_theta += std::f64::consts::PI; // Turn 180 degrees
        }
        
        // Update position estimate
        self.position_estimate = (bounded_x, bounded_y, new_theta);
    }

    pub fn update_weight(&mut self, weight_update: f64) {
        self.weight = weight_update;
    }
}

// STRUCTURE FOR THE DRONE
#[derive(Clone)]
pub struct Drone {
    pub id: usize, // ID of the drone
    pub color: [f32; 3], // RGB color for the drone and the particles, 
    //opacity is 1 for drone and .2 for particles
    pub worker: Worker, // The real worker that the drone is representing
    pub particles: Vec<Worker>, // Vector of M particles that the drone is representing
    pub relative_poses: Vec<(usize, f64, f64)>, // Vector of relative poses to other drones (id, distance, bearing)
}

impl Drone {
    fn get_target_drone(&self, target_id: usize) -> Option<&Drone> {
        // This is a placeholder - we need to pass the swarm to access other drones
        None
    }

    pub fn render(&self, context: &Context, graphics: &mut impl Graphics, is_updating: bool, all_drones: &[Drone]) {
        let (x, y, _) = self.worker.position_estimate;
        
        // Draw particles
        let particle_color: Color = [self.color[0], self.color[1], self.color[2], 0.2];
        let particle_triangle = Polygon::new(particle_color);
        for particle in &self.particles {
            let points = particle.calculate_triangle_points();
            let polygon: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
            particle_triangle.draw(&polygon, &context.draw_state, context.transform, graphics);
        }
        
        // Draw real worker
        let worker_color: Color = [self.color[0], self.color[1], self.color[2], 1.0];
        let worker_triangle = Polygon::new(worker_color);
        let points = self.worker.calculate_triangle_points();
        let polygon: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
        worker_triangle.draw(&polygon, &context.draw_state, context.transform, graphics);
        
        // Draw orange circumference around real worker
        let circle_color: Color = [1.0, 0.5, 0.0, 1.0]; // Full opacity orange
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
        
        // Draw update indicator if updating (thin circle)
        if is_updating {
            let indicator_color: Color = [1.0, 1.0, 0.0, 0.3]; // More transparent yellow
            let indicator_ellipse = Ellipse::new(indicator_color);
            indicator_ellipse.draw(
                [x - 10.0, y - 10.0, 20.0, 20.0],
                &context.draw_state,
                context.transform,
                graphics,
            );
        }
        
        // Draw relative poses with thinner lines
        for (target_id, _distance, _bearing) in &self.relative_poses {
            if let Some(target) = all_drones.iter().find(|d| d.id == *target_id) {
                let (tx, ty, _) = target.worker.position_estimate;
                let line_color: Color = [0.5, 0.5, 0.5, 0.2]; // More transparent gray
                Line::new(line_color, 0.5) // Thinner line
                    .draw([x, y, tx, ty], &context.draw_state, context.transform, graphics);
            }
        }
    }

    pub fn motion_update(&mut self, map: &Map, _bearing_update: f64, _distance_update: f64) {
        // Update real worker's location
        self.worker.update_location();  
        
        // Update all particles' location with some noise
        let mut rng = rand::thread_rng();   
        let _noise_bearing = rng.gen_range(-0.1..0.1);
        let _noise_distance = rng.gen_range(-0.1..0.1);
        for particle in &mut self.particles {
            particle.update_location();
        }
        
        // Update real worker's sensor readings
        self.worker.update_sensor_readings(map);
        // Update all particles' sensor readings
        for particle in &mut self.particles {
            particle.update_sensor_readings(map);
        }
    }

    pub fn belief_update(&mut self) {
        // Compare the sensor readings of the real worker and the particles, 
        // if the sensor reading of the particle is closer to the real worker's sensor reading, 
        // then update the weight of the particle
        for particle in &mut self.particles {
            let error = particle.sensor_reading.iter()
                .zip(self.worker.sensor_reading.iter())
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>();
            let sigma = 5.0; // Sensor noise standard deviation (tune this)
            let weight = (-error / (2.0 * sigma * sigma)).exp();
            particle.update_weight(weight);
        }
        
        // Normalize the weights of the particles
        let sum_weights = self.particles.iter().map(|p| p.weight).sum::<f64>();
        for particle in &mut self.particles {
            particle.weight = particle.weight / sum_weights;
        }
        
        // Cumulative weighted resampling
        let num_particles = self.particles.len();
        let mut new_particles = Vec::with_capacity(num_particles);
        
        // Calculate cumulative weights
        let mut cumulative_weights = Vec::with_capacity(num_particles);
        let mut sum = 0.0;
        for particle in &self.particles {
            sum += particle.weight;
            cumulative_weights.push(sum);
        }
        
        // Systematic resampling
        let step = 1.0 / num_particles as f64;
        let mut u = rand::thread_rng().gen_range(0.0..step);
        let mut j = 0;
        
        for _ in 0..num_particles {
            while u > cumulative_weights[j] {
                j += 1;
            }
            new_particles.push(self.particles[j].clone());
            u += step;
        }
        
        self.particles = new_particles;
    }

    pub fn update_sensor_readings(&mut self, map: &Map) {
        // Update real worker's sensor readings
        self.worker.update_sensor_readings(map);
        
        // Update all particles' sensor readings
        for particle in &mut self.particles {
            particle.update_sensor_readings(map);
        }
    }
}

#[derive(Clone)]
pub struct Swarm {
    pub drone_list: Vec<Drone>, // Vector of Drones
}

impl Swarm {
    pub fn render(&self, context: &Context, graphics: &mut impl Graphics, is_updating: bool) {
        for drone in &self.drone_list {
            drone.render(context, graphics, is_updating, &self.drone_list);
        }
    }

    pub fn update_relative_poses(&mut self) {
        // ... existing update_relative_poses code ...
    }

    pub fn update_sensor_readings(&mut self, map: &Map) {
        for drone in &mut self.drone_list {
            drone.update_sensor_readings(map);
        }
    }

    pub fn move_swarm(&mut self, map: &Map) {
        // Move the swarm around the map
        let mut rng = rand::thread_rng();
        for drone in &mut self.drone_list {
            let bearing_update = rng.gen_range(-0.05..0.05); // Reduced from -0.1..0.1 for less randomness
            let distance_update = 15.0; // Increased from 10.0 for more movement
            drone.motion_update(map, bearing_update, distance_update);
            drone.belief_update();
        }
    }
}









