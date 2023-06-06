import os 
import sys
import pygame 
import numpy as np 
from fluid_simulator import FluidSimulator

# Viewer 
window_width = 1200
window_height = 800
kernel_radius = 20 # range of influence for each particle 
water_bounds = np.array([[0,0],[1,3]]) * 200
wall_bounds = np.array([[2,0],[2.5,1]]) * 200 # (2 - 2.5 : range of x coordinate location (2m far from left wall, 0.5m thick so 2.5m))
house_bounds = np.array([[5.5,0],[6,0.5]]) * 200 # static house location

class Visualizer():
    def __init__(self):
        self.water_simulator = FluidSimulator()
        self.directory = "frames"
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        self.frame_idx = 0
        pygame.init()
        self.screen = pygame.display.set_mode((window_width, window_height))
        pygame.display.set_caption('Fluid Simulation')

    def draw_particles(self):
        for part in self.water_simulator.particles:
            pygame.draw.circle(self.screen, (0, 0, 255), (int(part.position[0]), int(part.position[1])), int(kernel_radius / 1.5))

    def draw_wall(self, bounds, color):
        pygame.draw.rect(self.screen, color, pygame.Rect(bounds[0][0], bounds[0][1], bounds[1][0] - bounds[0][0], bounds[1][1] - bounds[0][1]))

    def draw_house(self, bounds, color):
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
            self.draw_particles()
            self.draw_wall(wall_bounds, (0, 255, 0)) # Green
            self.draw_house(house_bounds, (255, 0, 0)) # Red

            pygame.display.flip()
            flipped_screen = pygame.transform.flip(self.screen, False, True)
            filename = f"{self.directory}/{self.frame_idx}.png"
            self.frame_idx += 1
            pygame.image.save(flipped_screen, filename)

            clock.tick(60)