/*
File: main.rs
Author: @aqc-github
Main entry point for the algorithm for Swarm Particle Localisation.
The goal is to prove that Particle filter (Monte Carlo) localisation 
is faster when computing from a swarm of robots.
*/

use piston_window::{clear, PistonWindow, WindowSettings, Context, Graphics};
use piston_window::Rectangle; // for creating the obstacles in the maze
use piston_window::{Polygon, ellipse}; // for rendering shapes
use rand::Rng;

// Global dimensions for the map
const DIMENSIONS: (f64, f64) = (700.0, 700.0); // 300x300 units



/// Robot struct (unchanged)
struct Robot {
    color: [f32; 4],
    particles: Vec<(f64, f64, f64)>, // (x, y, theta)
    estimated_pose: (f64, f64, f64), // (x, y, theta)
}

impl Robot {
    pub fn render(&self, context: &Context, graphics: &mut impl Graphics) {
        // The render method renders the drone as a triangle pointing in theta
        let triangle = Polygon::new(self.color);
        let points = self.calculate_triangle_points(self.estimated_pose.0, self.estimated_pose.1, self.estimated_pose.2);
        // Convert Vec<(f64, f64)> to Vec<[f64; 2]>
        let polygon: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
        triangle.draw(&polygon, &context.draw_state, context.transform, graphics);

        // For added visibility render an orange circle around the drone
        let orange_color_border: [f32; 4] = [1.0, 0.5, 0.0, 1.0];   // Solid border
        let circle_size = 16.0;
        // Draw circle border only
        ellipse::Ellipse::new_border(orange_color_border, 1.0).draw(
            [self.estimated_pose.0 - circle_size/2.0, self.estimated_pose.1 - circle_size/2.0, circle_size, circle_size],
            &context.draw_state,
            context.transform,
            graphics
        );

        // Render the particles as triangles, but more transparent
        let particle_color: [f32; 4] = self.color;
        let particle_color_transparent: [f32; 4] = [particle_color[0], particle_color[1], particle_color[2], 0.2];
        let particle_triangle = Polygon::new(particle_color_transparent);
        for particle in &self.particles {
            let points = self.calculate_triangle_points(particle.0, particle.1, particle.2);
            let polygon: Vec<[f64; 2]> = points.iter().map(|&(x, y)| [x, y]).collect();
            particle_triangle.draw(&polygon, &context.draw_state, context.transform, graphics);
        }
    }

    fn calculate_triangle_points(&self, x: f64, y: f64, theta: f64) -> Vec<(f64, f64)> {
        let size = 5.0;  // Size of the triangle
        let front_point = (
            x + size * theta.cos(),
            y + size * theta.sin()
        );
        let back_left = (
            x + size * (theta - 2.4).cos(),  // 2.4 radians ≈ 140 degrees
            y + size * (theta - 2.4).sin()
        );
        let back_right = (
            x + size * (theta + 2.4).cos(),
            y + size * (theta + 2.4).sin()
        );
        vec![front_point, back_left, back_right]
    }
}


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
}


// Function to generate X random obstacles in the map
fn generate_random_map(num_obstacles: usize) -> Vec<Obstacle> {
    let mut rng = rand::thread_rng();
    let mut obstacles = Vec::new();
    let color: [f32; 4] = [0.0, 0.0, 0.0, 1.0];  // Black color with full opacity
    let rectangle = Rectangle::new(color);

    for _ in 0..num_obstacles {
        let width = rng.gen_range(10.0..50.0);  // Random width between 10 and 50
        let height = rng.gen_range(10.0..50.0); // Random height between 10 and 50
        let dimension = (width, height);
        let x = rng.gen_range(0.0..(DIMENSIONS.0 - width));   // Ensure it fits in x-dimension
        let y = rng.gen_range(0.0..(DIMENSIONS.1 - height));  // Ensure it fits in y-dimension
        let origin = (x, y);

        obstacles.push(Obstacle {rectangle, origin, dimension});
    }

    obstacles
}

fn main() {
    // Create a window (adjusted to fit the map)
    let mut window: PistonWindow = WindowSettings::new("Swarm MCL", [DIMENSIONS.0 as u32, DIMENSIONS.1 as u32])
        .exit_on_esc(true)
        .build()
        .unwrap();

    // Generate a random map with 5 obstacles
    let _map = generate_random_map(100);

    // Initialize two robots with random particles
    let mut rng = rand::thread_rng();

    // Robot 1 appears in a random location and orientation
    let robot1 = Robot {
        color: [1.0, 0.0, 0.0, 1.0],  // Red color
        particles: (0..100).map(|_| (
            rng.gen_range(0.0..DIMENSIONS.0),  // x anywhere in map
            rng.gen_range(0.0..DIMENSIONS.1),  // y anywhere in map
            rng.gen_range(0.0..2.0 * std::f64::consts::PI),  // theta in [0, 2π]
        )).collect(),
        estimated_pose: (rng.gen_range(0.0..DIMENSIONS.0), rng.gen_range(0.0..DIMENSIONS.1), rng.gen_range(0.0..2.0 * std::f64::consts::PI)),
    };

    // Robot 2 appears in a random location and orientation
    let robot2 = Robot {
        color: [0.0, 0.0, 1.0, 1.0],  // Blue color
        particles: (0..100).map(|_| (
            rng.gen_range(0.0..DIMENSIONS.0),  // x anywhere in map
            rng.gen_range(0.0..DIMENSIONS.1),  // y anywhere in map
            rng.gen_range(0.0..2.0 * std::f64::consts::PI),  // theta in [0, 2π]
        )).collect(),
        estimated_pose: (rng.gen_range(0.0..DIMENSIONS.0), rng.gen_range(0.0..DIMENSIONS.1), rng.gen_range(0.0..2.0 * std::f64::consts::PI)),
    };
    
    // Robot 3 appears in a random location and orientation
    let robot3 = Robot {
        color: [0.0, 1.0, 0.0, 1.0],  // Green color
        particles: (0..100).map(|_| (
            rng.gen_range(0.0..DIMENSIONS.0),  // x anywhere in map
            rng.gen_range(0.0..DIMENSIONS.1),  // y anywhere in map
            rng.gen_range(0.0..2.0 * std::f64::consts::PI),  // theta in [0, 2π]
        )).collect(),
        estimated_pose: (rng.gen_range(0.0..DIMENSIONS.0), rng.gen_range(0.0..DIMENSIONS.1), rng.gen_range(0.0..2.0 * std::f64::consts::PI)),
    };
    let robots = vec![robot1, robot2, robot3];
    

    // Main loop
    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g, _device| {
            // Clear the screen to white
            clear([1.0, 1.0, 1.0, 1.0], g);
            // Render the obstacles
            for obstacle in &_map {
                obstacle.render(&c, g);
            }
            // Render the robots and their particles
            for robot in &robots {
                robot.render(&c, g);
            }
        });
    }
    
    
    
}
