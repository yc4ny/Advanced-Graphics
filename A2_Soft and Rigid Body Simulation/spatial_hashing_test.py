import time
import numpy as np
import matplotlib.pyplot as plt

class Object:
    def __init__(self):
        self.position = np.random.rand(3)

class Simulator:
    def __init__(self, use_spatial_hashing=False):
        self.objects = []
        self.use_spatial_hashing = use_spatial_hashing
        self.spatial_hash = {}
        self.cell_size = 50  # The size of each cell in the spatial hash.

    def add_object(self):
        self.objects.append(Object())

    def run_one_step(self):
        self.handle_object_collisions()

    def handle_object_collisions(self):
        if self.use_spatial_hashing:
            self.spatial_hash = {}
            for i, obj in enumerate(self.objects):
                cell_coords = tuple((obj.position // self.cell_size).astype(int))
                if cell_coords not in self.spatial_hash:
                    self.spatial_hash[cell_coords] = []
                self.spatial_hash[cell_coords].append(i)

            for i in range(len(self.objects)):
                for j in self.get_nearby_objects(i):
                    if j > i:  # Avoid checking each pair twice.
                        self.handle_collision_between(self.objects[i], self.objects[j])
        else:
            for i in range(len(self.objects)):
                for j in range(i + 1, len(self.objects)):
                    self.handle_collision_between(self.objects[i], self.objects[j])

    def get_nearby_objects(self, i):
        obj = self.objects[i]
        cell_coords = tuple((obj.position // self.cell_size).astype(int))
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    nearby_cell_coords = (cell_coords[0] + dx, cell_coords[1] + dy, cell_coords[2] + dz)
                    if nearby_cell_coords in self.spatial_hash:
                        for j in self.spatial_hash[nearby_cell_coords]:
                            yield j

    def handle_collision_between(self, obj1, obj2):
        pass

    def time_simulation(self, num_steps):
        start_time = time.time()
        for _ in range(num_steps):
            self.run_one_step()
        end_time = time.time()
        return end_time - start_time

def time_comparison(population_sizes, num_steps):
    times_without_hashing = []
    times_with_hashing = []

    for population_size in population_sizes:
        sim = Simulator(use_spatial_hashing=False)
        for _ in range(population_size):
            sim.add_object()
        without_hashing_time = sim.time_simulation(num_steps)
        times_without_hashing.append(without_hashing_time)

        sim = Simulator(use_spatial_hashing=True)
        for _ in range(population_size):
            sim.add_object()
        with_hashing_time = sim.time_simulation(num_steps)
        times_with_hashing.append(with_hashing_time)

        print(f"Population size: {population_size}")
        print(f"Time with spatial hashing: {without_hashing_time}")
        print(f"Time without spatial hashing: {with_hashing_time}")

    plt.figure()
    plt.plot(population_sizes, times_without_hashing, label="With Spatial Hashing")
    plt.plot(population_sizes, times_with_hashing, label="Without Spatial Hashing")
    plt.xlabel("Population Size")
    plt.ylabel("Time (s)")
    plt.legend()
    plt.grid(True)
    plt.savefig('time_comparison.jpg')

# Running the test
time_comparison(population_sizes=[1,2,3,4,5,6,7,8,9,10,100,200,300,400,500,600,700,800,900,1000, 2000, 3000, 4000, 5000, 10000], num_steps= 1)