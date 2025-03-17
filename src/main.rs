/*
File: main.rs
Author: @aqc-github
Main entry point for the algorithm for Swarm Particle Localisation.
The goal is to prove that Particle filter (Monte Carlo) localisation 
is faster when computing from a swarm of robots.
*/

use piston_window::{clear, PistonWindow, WindowSettings, Context, Graphics};
use piston_window::Rectangle; // for creating the obstacles in the maze
use std::time::{Duration, Instant};

mod swarm_elements;
use swarm_elements::*;
use rand::Rng;

// Global dimensions for the map
const DIMENSIONS: (f64, f64) = (700.0, 700.0); // 300x300 units
const CELL_SIZE: f64 = 5.0; // Size of each cell in the map matrix
const UPDATE_INTERVAL: Duration = Duration::from_millis(1); // Update every 1 seconds

/// Obstacle structure
struct Obstacle {
    rectangle: Rectangle,
    origin: (f64, f64),
    dimension: (f64, f64),
}

impl Obstacle {
    pub fn render(&self, context: &Context, graphics: &mut impl Graphics) {
        let from = (self.origin.0, self.origin.1);
        let to = (self.origin.0 + self.dimension.0, self.origin.1 + self.dimension.1);
        self.rectangle.draw_from_to(from, to, &context.draw_state, context.transform, graphics);
    }

    pub fn fill_map_matrix(&self, map: &mut Map) {
        let (start_x, start_y) = self.origin;
        let (width, height) = self.dimension;
        
        // Convert obstacle bounds to grid coordinates
        let (start_row, start_col) = map.world_to_grid(start_x, start_y);
        let (end_row, end_col) = map.world_to_grid(start_x + width, start_y + height);
        
        // Fill the matrix cells that contain the obstacle
        for row in start_row..=end_row {
            for col in start_col..=end_col {
                if row < map.matrix.len() && col < map.matrix[0].len() {
                    map.matrix[row][col] = 1;
                }
            }
        }
    }
}

// Function to generate X random obstacles in the map
fn generate_random_map(num_obstacles: usize) -> (Vec<Obstacle>, Map) {
    let mut rng = rand::thread_rng();
    let mut obstacles = Vec::new();
    let color: [f32; 4] = [0.0, 0.0, 0.0, 1.0];  // Black color with full opacity
    let rectangle = Rectangle::new(color);

    // Create empty map matrix
    let mut map = Map::new(DIMENSIONS, CELL_SIZE);

    for _ in 0..num_obstacles {
        let width = rng.gen_range(10.0..50.0);  // Random width between 10 and 50
        let height = rng.gen_range(10.0..50.0); // Random height between 10 and 50
        let dimension = (width, height);
        let x = rng.gen_range(0.0..(DIMENSIONS.0 - width));   // Ensure it fits in x-dimension
        let y = rng.gen_range(0.0..(DIMENSIONS.1 - height));  // Ensure it fits in y-dimension
        let origin = (x, y);

        let obstacle = Obstacle {rectangle, origin, dimension};
        obstacle.fill_map_matrix(&mut map);
        obstacles.push(obstacle);
    }

    (obstacles, map)
}

fn main() {
    // Create a window (adjusted to fit the map)
    let mut window: PistonWindow = WindowSettings::new("Swarm MCL", [DIMENSIONS.0 as u32, DIMENSIONS.1 as u32])
        .exit_on_esc(true)
        .build()
        .unwrap();

    // Generate a random map with obstacles and create the map matrix
    let (_map, map_matrix) = generate_random_map(100);

    let mut last_update = Instant::now();

    // Create drones with particles
    let particle_count = 1000;
    let drone1 = Drone::new(0, [1.0, 0.0, 0.0], particle_count);
    let drone2 = Drone::new(1, [0.0, 1.0, 0.0], particle_count);
    let drone3 = Drone::new(2, [0.0, 0.0, 1.0], particle_count);

    let drone_list = vec![drone1, drone2, drone3];

    // Initialize the swarm with the drone list
    let mut swarm = Swarm { drone_list };

    // Main loop
    while let Some(e) = window.next() {
        let is_updating = last_update.elapsed() >= UPDATE_INTERVAL;
        
        // Check if it's time to update (every second)
        if is_updating {
            // Update relative poses
            swarm.update_relative_poses();
            
            // Move the swarm and update beliefs
            swarm.move_swarm(&map_matrix);
            
            last_update = Instant::now();
        }
        
        window.draw_2d(&e, |c, g, _device| {
            clear([1.0, 1.0, 1.0, 1.0], g);
            
            // Render obstacles
            for obstacle in &_map {
                obstacle.render(&c, g);
            }
            
            // Render swarm with update state
            swarm.render(&c, g, is_updating);
        });
    }
}
