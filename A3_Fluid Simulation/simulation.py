import os
import sys
import pygame
import numpy as np
from copy import deepcopy

# solver parameters
gravity = np.array([0.0, -9.81])   
rest_density = 997  
gas_constant = 450.0 
kernel_radius = 20.0  
kernel_radius_squared = kernel_radius * kernel_radius  
particle_mass = 4  
delta_time = 1.0 / 3000 

# smoothing kernels
poly_6 = (315.0 / 64.0) / (np.pi * kernel_radius**8) 
spiky_grad = -12.0 / (np.pi * kernel_radius**5)

# simulation parameters
boundary_epsilon = kernel_radius*0.5  
boundary_damping = -0.05

# interaction
num_particles = 300

# rendering projection parameters
window_width = 1200
window_height = 800
view_width = 1.0 * 1200.0
view_height = 1.0 * 800.0

# water, dam, house range
water_bounds = np.array([[0,0],[1,3]]) * 200
house_bounds = np.array([[5,0],[5.5,1]]) * 200
wall_bounds = np.array([[5.5,0],[6,0.5]]) * 200

class Particle:
    def __init__(self, _x, _y):
        self.position = np.array([_x, _y])
        self.velocity = np.array([0.0, 0.0])
        self.force = np.array([0.0, 0.0])
        self.density = 0.0
        self.pressure = 0.0

    def update(self):
        self.velocity += delta_time * self.force / (self.density + 1e-05)
        self.position += delta_time * self.velocity

class FluidSimulator():
    def __init__(self) -> None:
        self.particles = []

    def reset(self):
        y_values = np.arange(water_bounds[0][1], water_bounds[1][1], kernel_radius)
        x_values = np.arange(water_bounds[0][0], water_bounds[1][0], kernel_radius)

        for y in y_values:
            for x in x_values:
                random_jitter = np.random.rand()
                new_particle = Particle(x + random_jitter, y)
                self.particles.append(new_particle)
        
    def apply_forces(self):
        for part in self.particles:
            part.update()

    def collision_boundary(self):
        for part in self.particles:
            for axis in range(2):   
                if part.position[axis] < 0.0:
                    part.velocity[axis] *= boundary_damping
                    part.position[axis] = 0.0
                elif part.position[axis] > view_width:
                    part.velocity[axis] *= boundary_damping
                    part.position[axis] = view_width


    def collision_wall(self, bounds):
        for part in self.particles:
            if not (bounds[0][0] <= part.position[0] <= bounds[1][0] and bounds[0][1] <= part.position[1] <= bounds[1][1]):
                continue 
            else:
                pos_fixed = deepcopy(part.position)
                dist_to_wall = [abs(part.position[0]-bounds[0][0]), abs(part.position[0]-bounds[1][0]), abs(part.position[1]-bounds[1][1])]
                closest_axis = np.argmin(dist_to_wall)

                if closest_axis == 0:
                    pos_fixed[0] = bounds[0][0]
                    part.velocity[0] *= boundary_damping
                elif closest_axis == 1:
                    pos_fixed[0] = bounds[1][0]
                    part.velocity[0] *= boundary_damping
                elif closest_axis == 2:
                    pos_fixed[1] = bounds[1][1]
                    part.velocity[1] *= boundary_damping

                part.position = pos_fixed

    def wall_reflected_axis(self, pos, wall_range):
        pos_fixed = deepcopy(pos)
        dist_to_wall = [abs(pos[0]-wall_range[0][0]), abs(pos[0]-wall_range[1][0]), abs(pos[1]-wall_range[1][1])]
        closest_axis = np.argmin(dist_to_wall)
        if closest_axis == 0:
            pos_fixed[0] = wall_range[0][0]
        elif closest_axis == 1:
            pos_fixed[0] = wall_range[1][0]
        elif closest_axis == 2:
            pos_fixed[1] = wall_range[1][1]
        return pos_fixed

    def compute_density(self):
        for part in self.particles:
            part.density = 0.0
            for other_part in self.particles:
                diff = other_part.position - part.position
                distance = np.linalg.norm(diff)**2
                if distance < kernel_radius_squared:
                    part.density += particle_mass * poly_6 * (kernel_radius_squared - distance)**3
            part.pressure = gas_constant * (part.density - rest_density)

    def compute_forces(self):
        global gravity
        for part in self.particles:
            pressure_force = np.array([0.0, 0.0])
            for other_part in self.particles:
                if part == other_part:
                    continue

                diff = other_part.position - part.position
                distance = np.linalg.norm(diff)

                if distance < kernel_radius:
                    distance += 1e-05
                    diff_normalized = diff / distance
                    ratio = (part.pressure + other_part.pressure) / (2.0 * other_part.density + 1e-05)
                    pressure_force += -diff_normalized * particle_mass * ratio * spiky_grad * (kernel_radius - distance)**3

            gravity_force = gravity * particle_mass * (1.0 / (part.density + 1e-05)) # changed gravity -> gravity_force
            part.force = pressure_force + gravity_force

    def update(self):
        self.compute_density()
        self.compute_forces()
        self.apply_forces()
        self.collision_boundary()
        self.collision_wall(wall_bounds)
        self.collision_wall(house_bounds)


class FluidVisualizer():
    def __init__(self):
        self.water_simulator = FluidSimulator()
        self.record = False
        self.set_directory()
        self.frame_idx = 0
        pygame.init()
        self.screen = pygame.display.set_mode((window_width, window_height))
        pygame.display.set_caption('Water Simulation')

    def render_particles(self):
        for part in self.water_simulator.particles:
            pygame.draw.circle(self.screen, (51, 153, 255), (int(part.position[0]), int(part.position[1])), int(kernel_radius / 1.5))

    def render_wall(self, bounds, color):
        pygame.draw.rect(self.screen, color, pygame.Rect(bounds[0][0], bounds[0][1], bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]))

    def run(self):
        self.water_simulator.reset()
        clock = pygame.time.Clock()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.water_simulator.particles.clear()
                        self.water_simulator.reset()
            
            self.screen.fill((255, 255, 255)) # White
            self.water_simulator.update()
            self.render_particles()
            self.render_wall(wall_bounds, (0, 255, 0)) # Green
            self.render_wall(house_bounds, (255, 0, 0)) # Red

            pygame.display.flip()
            flipped_screen = pygame.transform.flip(self.screen, False, True)
            filename = f"{self.directory}/{self.frame_idx}.png"
            self.frame_idx += 1
            pygame.image.save(flipped_screen, filename)

            clock.tick(60)

    def set_directory(self):
        self.directory = "./output_width"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        else:
            file_list = os.listdir(self.directory)
            for file_name in file_list:
                if "mp4" in file_name:
                    continue
                file_path = os.path.join(self.directory, file_name)
                if os.path.isfile(file_path):
                    os.remove(file_path)


if __name__ == "__main__":
    visualizer = FluidVisualizer()
    visualizer.run()