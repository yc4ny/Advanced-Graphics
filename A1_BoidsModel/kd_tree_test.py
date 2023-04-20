import pygame
import pygame_gui
import numpy as np 
import random 
import time
import timeit
from scipy.spatial import KDTree

class Boid:
    def __init__(self, position, velocity, max_speed = 1.0):
        self.position = position 
        self.velocity = velocity 
        self.max_speed = max_speed 

    def update(self, forces):
        self.velocity += np.sum(forces, axis=0)
        self.velocity = self.velocity / np.linalg.norm(self.velocity) * self.max_speed
        self.position += self.velocity

class BoidSimulation:
    def __init__(self, num_boids, bounds, use_kdtree = True,neighbor_radius = 1, separation_weight=1.0, alignment_weight=1.0, cohesion_weight=1.0, boundary_weight=1.0,
                 goal_weight=1.0, predator_weight=2.0, obstacle_weight=2.5):
        self.boids = []
        self.bounds = bounds
        self.weights = {
            'separation': separation_weight,
            'alignment': alignment_weight,
            'cohesion': cohesion_weight,
            'boundary': boundary_weight,
            'goal': goal_weight,
            'predator': predator_weight,
            'obstacle': obstacle_weight
        }
        self.obstacles = [
            Obstacle(np.array([70.0, 70.0, 70.0]), 5.0),
            Obstacle(np.array([80.0, 80.0, 80.0]), 5.0),
            Obstacle(np.array([90.0, 90.0, 90.0]), 5.0)
        ]
        self.use_kdtree = use_kdtree
        self.neighbor_radius = neighbor_radius 

        for _ in range(num_boids):
            position = np.array([random.uniform(bounds[0], bounds[1]), random.uniform(bounds[2], bounds[3]), random.uniform(bounds[4], bounds[5])])
            velocity = np.array([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)])
            self.boids.append(Boid(position, velocity, max_speed=3.0))

        # Define goal, predators, and obstacles here
        self.goal_position = np.array([75.0, 75.0, 75.0])
        self.predator_positions = [np.array([60.0, 60.0, 60.0])]
        self.obstacles = []  # Assuming a list of Obstacle instances with a `position` attribute

    def cohesion(self, boid, neighbors, cohesion_radius=5.0):
        center_of_mass = np.zeros(3)
        count = 0
        for neighbor in neighbors:
            distance = np.linalg.norm(boid.position - neighbor.position)
            if distance < cohesion_radius and distance > 0:
                center_of_mass += neighbor.position
                count += 1
        if count > 0:
            center_of_mass /= count
            return (center_of_mass - boid.position) * self.weights['cohesion']
        else:
            return np.zeros(3)

    def separation(self, boid, neighbors, separation_radius=2.0):
        separation_force = np.zeros(3)
        for neighbor in neighbors:
            if np.linalg.norm(boid.position - neighbor.position) < separation_radius:
                separation_force += (boid.position - neighbor.position)
        return separation_force * self.weights['separation']

    def alignment(self, boid, neighbors, alignment_radius=5.0):
        average_velocity = np.zeros(3)
        count = 0
        for neighbor in neighbors:
            distance = np.linalg.norm(boid.position - neighbor.position)
            if distance < alignment_radius and distance > 0:
                average_velocity += neighbor.velocity
                count += 1
        if count > 0:
            average_velocity /= count
            return (average_velocity - boid.velocity) * self.weights['alignment']
        else:
            return np.zeros(3)

    def boundary(self, boid):
        boundary_force = np.zeros(3)
        for i in range(3):
            if boid.position[i] < self.bounds[i * 2]:
                boundary_force[i] = self.bounds[i * 2] - boid.position[i]
            elif boid.position[i] > self.bounds[i * 2 + 1]:
                boundary_force[i] = self.bounds[i * 2 + 1] - boid.position[i]
        return boundary_force * self.weights['boundary']
    
    def goal_seeking(self, boid):
        vector_to_goal = self.goal_position - boid.position
        normalized_vector = vector_to_goal / np.linalg.norm(vector_to_goal)
        return self.weights['goal'] * normalized_vector

    def predator_avoidance(self, boid, radius=5.0):
        avoidance_force = np.zeros(3)
        
        for predator in self.predator_positions:
            distance_to_predator = np.linalg.norm(predator - boid.position)
            
            if distance_to_predator < radius:
                escape_vector = boid.position - predator
                normalized_vector = escape_vector / distance_to_predator
                avoidance_force += self.weights['predator'] * normalized_vector

        return avoidance_force

    def obstacle_avoidance(self, boid, radius=3.0):
        avoidance_force = np.zeros(3)
        
        for obstacle in self.obstacles:
            distance_to_obstacle = np.linalg.norm(obstacle.position - boid.position)
            
            if distance_to_obstacle < radius:
                escape_vector = boid.position - obstacle.position
                normalized_vector = escape_vector / distance_to_obstacle
                avoidance_force += self.weights['obstacle'] * normalized_vector

        return avoidance_force
    
    def update(self):

        # Step 3: Using KD-Tree for Large-scale Simulation
        boid_positions = np.array([boid.position for boid in self.boids])
        boid_kdtree = KDTree(boid_positions)

        for boid in self.boids:
            if self.use_kdtree:
                # Step 3: Using KD-Tree for Large-scale Simulation - Added KD Tree 
                neighbor_indices = boid_kdtree.query_ball_point(boid.position, self.neighbor_radius)
                neighbors = [self.boids[i] for i in neighbor_indices if self.boids[i] is not boid]
            else:
                # Step 3: Without KD-Tree 
                neighbors = [other for other in self.boids if other is not boid]

            separation_force = self.separation(boid, neighbors)
            alignment_force = self.alignment(boid, neighbors)
            cohesion_force = self.cohesion(boid, neighbors)
            boundary_force = self.boundary(boid)
            goal_force = self.goal_seeking(boid)
            # predator_force = self.predator_avoidance(boid)
            # obstacle_force = self.obstacle_avoidance(boid)

            # Step 1 
            forces = [separation_force, alignment_force, cohesion_force, boundary_force, goal_force]
            
            # # Step 2
            # forces = [separation_force, alignment_force, cohesion_force, boundary_force,
            #           goal_force, predator_force, obstacle_force]
            
            boid.update(forces)

class Obstacle:
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius

class BoidVisualizer:
    def __init__(self, simulation, screen_size):
        pygame.init()
        self.simulation = simulation
        self.screen_size = screen_size
        self.scale = min(screen_size[0]/simulation.bounds[1], screen_size[1]/simulation.bounds[3])
        self.screen = pygame.display.set_mode(screen_size)
        self.manager = pygame_gui.UIManager(screen_size)
        self.create_sliders()

    def create_sliders(self):
        self.separation_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect((20, 10), (200, 20)),
            text="Separation Weight",
            manager=self.manager
        )
        self.separation_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pygame.Rect((20, 40), (200, 20)),
            start_value=self.simulation.weights['separation'],
            value_range=(0, 100),
            manager=self.manager
        )
        self.alignment_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect((20, 70), (200, 20)),
            text="Alignment Weight",
            manager=self.manager
        )
        self.alignment_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pygame.Rect((20, 100), (200, 20)),
            start_value=self.simulation.weights['alignment'],
            value_range=(0, 100),
            manager=self.manager
        )
        self.cohesion_label = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect((20, 130), (200, 20)),
            text="Cohesion Weight",
            manager=self.manager
        )
        self.cohesion_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pygame.Rect((20, 160), (200, 20)),
            start_value=self.simulation.weights['cohesion'],
            value_range=(0, 100),
            manager=self.manager
        )

    def set_goal_position(self, x, y):
        self.simulation.goal_position[0] = x / self.scale
        self.simulation.goal_position[1] = y / self.scale

    def set_predator_position(self, x, y):
        self.simulation.predator_positions[0][0] = x / self.scale
        self.simulation.predator_positions[0][1] = y / self.scale

    def run(self):
        clock = pygame.time.Clock()
        is_running = True
        while is_running:
            time_delta = clock.tick(60) / 1000.0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    is_running = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left mouse button
                        x, y = event.pos
                        self.set_goal_position(x, y)

                if event.type == pygame.MOUSEMOTION:
                    x, y = event.pos
                    self.set_predator_position(x, y)

                self.manager.process_events(event)

                if event.type == pygame.USEREVENT:
                    if event.user_type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                        if event.ui_element == self.separation_slider:
                            self.simulation.weights['separation'] = event.value
                        elif event.ui_element == self.alignment_slider:
                            self.simulation.weights['alignment'] = event.value
                        elif event.ui_element == self.cohesion_slider:
                            self.simulation.weights['cohesion'] = event.value
            self.screen.fill((255, 255, 255))
            self.simulation.update()
            self.manager.update(time_delta)
            self.screen.fill((255, 255, 255))

            for boid in self.simulation.boids:
                position = (int(boid.position[0] * self.scale), int(boid.position[1] * self.scale))
                pygame.draw.circle(self.screen, (0, 0, 0), position, 5)

            for obstacle in self.simulation.obstacles:
                position = (int(obstacle.position[0] * self.scale), int(obstacle.position[1] * self.scale))
                pygame.draw.circle(self.screen, (0, 0, 255), position, int(obstacle.radius * self.scale))
                
            # Add this block to draw the predator
            predator_position = (int(self.simulation.predator_positions[0][0] * self.scale),
                                 int(self.simulation.predator_positions[0][1] * self.scale))
            pygame.draw.circle(self.screen, (255, 0, 0), predator_position, 7)

            self.manager.update(clock.tick(60) / 1000.0)
            self.manager.draw_ui(self.screen)

            pygame.display.flip()

            self.simulation.update()

        pygame.quit()

def measure_time(simulation, iterations):
    start_time = timeit.default_timer()
    for _ in range(iterations):
        simulation.update()
    end_time = timeit.default_timer()
    return end_time - start_time

def find_neighbors_list(boids, target_boid, radius):
    neighbors = []
    for boid in boids:
        if np.linalg.norm(target_boid.position - boid.position) <= radius:
            neighbors.append(boid)
    return neighbors

def find_neighbors_kdtree(boids, target_boid, radius):
    kdtree = KDTree([boid.position for boid in boids])
    distances, indices = kdtree.query_radius([target_boid.position], radius, return_distance=True)
    neighbors = [boids[index] for index in indices[0]]
    return neighbors

def benchmark_neighbor_search(boids, radii):
    results = []

    for radius in radii:
        target_boid = np.random.choice(boids)

        start_time = time.time()
        neighbors_list = find_neighbors_list(boids, target_boid, radius)
        list_time = time.time() - start_time

        start_time = time.time()
        neighbors_kdtree = find_neighbors_kdtree(boids, target_boid, radius)
        kdtree_time = time.time() - start_time

        results.append((radius, list_time, kdtree_time))

    return results


if __name__ == "__main__":
    bounds = [100, 400, 100, 400, 100, 400]

    # # Step 1, 2
    # simulation = BoidSimulation(100, bounds)
    # screen_size = (1000, 1000)
    # visualizer = BoidVisualizer(simulation, screen_size)
    # visualizer.run()

    # Step 3: 
    # Define population sizes and neighbor_radius values to test
    population_sizes = [10, 50, 100, 200, 400, 800]
    neighbor_radius_values = [5]

    for pop_size in population_sizes:
        for neighbor_radius in neighbor_radius_values:
            # Create BoidSimulation instances with different population sizes and KD Tree usage
            simulation_no_kdtree = BoidSimulation(pop_size, bounds, use_kdtree=False, neighbor_radius=neighbor_radius)
            simulation_with_kdtree = BoidSimulation(pop_size, bounds, use_kdtree=True, neighbor_radius=neighbor_radius)

            # Measure the execution time for each simulation
            time_no_kdtree = measure_time(simulation_no_kdtree, 100)
            time_with_kdtree = measure_time(simulation_with_kdtree, 100)

            print(f"Population: {pop_size}, Neighbor radius: {neighbor_radius}")
            print(f"Execution time without KD Tree: {time_no_kdtree:.2f} seconds")
            print(f"Execution time with KD Tree: {time_with_kdtree:.2f} seconds")
    

