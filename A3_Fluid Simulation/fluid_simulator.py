import numpy as np 

# Constants 
num_particles = 300                 # number of particles
gravity = np.array([0.0, -9.81])    # acceleration due to gravity 
rest_density = 997                  # rest density of fluid 
gas_constant = 450               # ideal gas law, calculation of pressure of fluid 
kernel_radius = 20              # range of influence for each particle 
particle_mass = 4                   # mass of each particle in fluid 
delta_time = 1/5000              # time step of simulation

# Visualizer & Bounds
window_width = 1200
window_height = 800
water_bounds = np.array([[0,0],[1,3]]) * 200
wall_bounds = np.array([[2,0],[2.5,1]]) * 200 # (2 - 2.5 : range of x coordinate location (2m far from left wall, 0.5m thick so 2.5m))
house_bounds = np.array([[5.5,0],[6,0.5]]) * 200 # static house location

class FluidSimulator():
    def __init__(self) -> None:
        self.particles = []

    def handle_boundary_collision(self, particle):
        def handle_axis_collision(axis):
            if particle.position[axis] < 0.0:
                particle.velocity[axis] *= -0.05
                particle.position[axis] = 0.0
            elif particle.position[axis] > window_width:
                particle.velocity[axis] *= -0.05
                particle.position[axis] = window_width

        handle_axis_collision(0)
        handle_axis_collision(1)

    def collision_boundary(self):
        for part in self.particles:
            self.handle_boundary_collision(part)


    def handle_wall_collision(self, bounds):
        for particle in self.particles:
            if bounds[0][0] <= particle.position[0] <= bounds[1][0] and bounds[0][1] <= particle.position[1] <= bounds[1][1]:
                fixed_position = np.copy(particle.position)

                # Compute distances in both axes (0 and 1, corresponding to x and y respectively)
                distances = [abs(particle.position[i]-bounds[i//2][i%2]) for i in range(2)]

                closest_axis_index = distances.index(min(distances))

                axis = closest_axis_index % 2
                wall_edge = closest_axis_index // 2

                fixed_position[axis] = bounds[wall_edge][axis]
                particle.velocity[axis] *= -0.05

                particle.position = fixed_position

    def compute_density(self):
        for particle in self.particles:
            particle.density = 0.0
            density_factor = particle_mass * (315.0 / 64.0) / (np.pi * kernel_radius**8)

            for other_particle in self.particles:
                position_difference = other_particle.position - particle.position
                squared_distance = np.dot(position_difference, position_difference)

                if squared_distance < kernel_radius**2:
                    density_increase = density_factor * (kernel_radius**2 - squared_distance)**3
                    particle.density += density_increase

            particle.pressure = gas_constant * (particle.density - rest_density)

    def compute_forces(self):
        global gravity
        for current_particle in self.particles:
            pressure_force = np.zeros(2)
            for other_particle in self.particles:
                if current_particle == other_particle:
                    continue

                diff_vector = other_particle.position - current_particle.position
                dist = np.sqrt(np.dot(diff_vector, diff_vector))

                if dist < kernel_radius:
                    dist += 0.000001
                    normalized_diff_vector = diff_vector / dist
                    pressure_ratio = (current_particle.pressure + other_particle.pressure) / (2.0 * other_particle.density + 0.000001)
                    pressure_contribution = -normalized_diff_vector * particle_mass * pressure_ratio * -12.0 / (np.pi * kernel_radius**5) * (kernel_radius - dist)**3
                    pressure_force += pressure_contribution

            gravity_contribution = gravity * particle_mass * (1.0 / (current_particle.density + 0.000001))
            current_particle.force = pressure_force + gravity_contribution

        
    def apply_forces(self):
        for part in self.particles:
            part.update()

    def reset(self):
        y_values = np.arange(water_bounds[0][1], water_bounds[1][1], kernel_radius)
        x_values = np.arange(water_bounds[0][0], water_bounds[1][0], kernel_radius)

        for y in y_values:
            for x in x_values:
                new_particle = Particle(x + np.random.rand(), y)
                self.particles.append(new_particle)

    def update(self):
        self.compute_density()
        self.compute_forces()
        self.apply_forces()
        self.collision_boundary()
        self.handle_wall_collision(wall_bounds)
        self.handle_wall_collision(house_bounds)

class Particle:
    def __init__(self, _x, _y):
        self.position = np.array([_x, _y])
        self.velocity = np.array([0.0, 0.0])
        self.force = np.array([0.0, 0.0])
        self.density = 0.0
        self.pressure = 0.0

    def update(self):
        self.velocity += delta_time * self.force / (self.density + 0.000001)
        self.position += delta_time * self.velocity