import open3d as o3d 
import numpy as np 
import json 

class SpatialHash:
    def __init__(self, cell_size):
        self.cell_size = cell_size
        self.hash_map = {}

    def hash(self, position):
        # Divide the position by the cell size and round down to the nearest integer to get the cell coordinates.
        return tuple((position // self.cell_size).astype(int))

    def insert(self, obj):
        # Insert the object into the cell corresponding to its position.
        h = self.hash(obj.position)
        if h not in self.hash_map:
            self.hash_map[h] = []
        self.hash_map[h].append(obj)

    def remove(self, obj):
        # Remove the object from the cell corresponding to its old position.
        h = self.hash(obj.position)
        self.hash_map[h].remove(obj)

    def query(self, obj):
        # Return all objects in the same cell and the neighboring cells.
        h = self.hash(obj.position)
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    cell = (h[0]+dx, h[1]+dy, h[2]+dz)
                    if cell in self.hash_map:
                        neighbors.extend(self.hash_map[cell])
        return neighbors

class Simulator:
    def __init__(self):
        # A list to store all the soft and rigid objects in the simulation.
        self.objects = []
        # Initialize the Open3D visualization.
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window()

        # Initialize the Open3D view control.
        self.view = self.vis.get_view_control()
        
        # Initialize the timestep size and the number of substeps.
        self.timestep_size = 0.01
        self.num_substeps = 1
        self.running = False

        # Initialize the boundary (cube)
        self.boundary = np.array([10, 10, 10])
        self.draw_boundary()
        self.vis.register_key_callback(ord("S"), self.start_simulation)
        self.vis.register_key_callback(ord("P"), self.pause_simulation)
        self.vis.register_key_callback(ord("R"), self.reset_simulation)
        self.vis.register_key_callback(ord("A"), self.add_soft_object_cb)
        self.vis.register_key_callback(ord("B"), self.add_rigid_object_cb)
        self.vis.register_key_callback(ord("2"), self.increase_timestep_size)
        self.vis.register_key_callback(ord("1"), self.decrease_timestep_size)
        self.vis.register_key_callback(ord("4"), self.increase_num_substeps)
        self.vis.register_key_callback(ord("3"), self.decrease_num_substeps)

        self.initialize_objects()

        self.selected_object = None
        self.vis.register_key_callback(ord("M"), self.toggle_select_object)

    def toggle_select_object(self, vis):
        # Cycle through objects each time 'M' is pressed.
        if self.selected_object is None:
            self.selected_object = 0
        else:
            self.selected_object += 1
            if self.selected_object >= len(self.objects):
                self.selected_object = None
        print(f"Selected object: {self.selected_object}")

    def select_object(self, x, y):
        # Convert the mouse pointer coordinates to a ray in world coordinates.
        ray = self.view.convert_to_pinhole_camera_parameters().unproject(x, y, 1)

        # Calculate the closest point on each object to the ray.
        distances = []
        for obj in self.objects:
            distances.append(obj.mesh.distance_to(ray))

        # Select the object with the closest point.
        self.selected_object = self.objects[np.argmin(distances)]
        self.attachment_point = self.selected_object.mesh.closest_point(ray)

    def add_soft_object(self, filename, position=np.array([0.0, 0.0, 0.0])):
        obj = SoftObject(filename, position)
        obj.draw(self.vis)
        self.objects.append(obj)

    def add_rigid_object(self, radius=1):
        obj = RigidObject(radius)
        obj.draw(self.vis)
        self.objects.append(obj)

    def start_simulation(self):
        self.running = True
        self.run()

    def pause_simulation(self):
        self.running = False

    def reset_simulation(self):
        self.objects = []
        self.vis.clear_geometries()
        self.running = False

    def set_timestep_size(self, size):
        self.timestep_size = size

    def set_num_substeps(self, num):
        self.num_substeps = num

    def run(self):
        while self.running:
            for i, obj in enumerate(self.objects):
                for _ in range(self.num_substeps):
                    obj.update_position(self.timestep_size / self.num_substeps)
                    if isinstance(obj, SoftObject):
                        obj.handle_collision(self.boundary)
                    elif isinstance(obj, RigidObject):
                        obj.handle_collision(self.boundary, obj.position, obj.velocity)
                self.handle_object_collisions()

                if i == self.selected_object:
                    # Project the mouse position from 2D screen space to 3D world space.
                    # You might need to adjust this depending on your camera setup.
                    x, y = self.view.convert_to_pinhole_camera_parameters().unproject(*self.vis.get_mouse_position(), 1)
                    obj.position = np.array([x, y, obj.position[2]])  # Assuming z position remains constant.

                self.vis.update_geometry(obj.mesh)
            self.vis.poll_events()
            self.vis.update_renderer()

    def handle_object_collisions(self):
        for i in range(len(self.objects)):
            for j in range(i+1, len(self.objects)):
                if isinstance(self.objects[i], RigidObject) and isinstance(self.objects[j], RigidObject):
                    # Calculate the vector between the objects' positions.
                    delta = self.objects[i].position - self.objects[j].position

                    # Calculate the distance between the objects.
                    dist = np.linalg.norm(delta)

                    # Check if the distance is less than the sum of the radii.
                    if dist < (self.objects[i].radius + self.objects[j].radius):
                        # Calculate the unit vector of the direction of collision.
                        collision_direction = delta / dist

                        # Swap the velocities of the two objects along the direction of collision.
                        # This simulates a perfectly elastic collision.
                        self.objects[i].velocity, self.objects[j].velocity = (
                            self.objects[j].velocity - 2 * (self.objects[j].velocity.dot(collision_direction)) * collision_direction,
                            self.objects[i].velocity - 2 * (self.objects[i].velocity.dot(collision_direction)) * collision_direction
                        )


    def draw_boundary(self):
        # Define the 8 corners of the cube.
        points = [
            [0, 0, 0],
            [0, 0, self.boundary[2]],
            [0, self.boundary[1], 0],
            [0, self.boundary[1], self.boundary[2]],
            [self.boundary[0], 0, 0],
            [self.boundary[0], 0, self.boundary[2]],
            [self.boundary[0], self.boundary[1], 0],
            [self.boundary[0], self.boundary[1], self.boundary[2]],
        ]
        points = o3d.utility.Vector3dVector(points)

        # Define the 12 edges of the cube.
        lines = [
            [0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3], [2, 6], [3, 7], [4, 5], [4, 6], [5, 7], [6, 7]
        ]
        lines = o3d.utility.Vector2iVector(lines)

        # Create a LineSet from the points and lines.
        line_set = o3d.geometry.LineSet(points=points, lines=lines)

        # Draw the boundary in the Open3D visualizer.
        self.vis.add_geometry(line_set)

    def start_simulation(self, vis):
        self.running = True
        self.run()

    def pause_simulation(self, vis):
        self.running = False

    def initialize_objects(self):
        self.add_soft_object("cube.json", position=np.array([5.0, 5.0, 5.0]))
        self.add_rigid_object()

    def reset_simulation(self, vis):
        self.objects = []
        self.vis.clear_geometries()
        self.running = False
        self.initialize_objects()  # Re-add the objects after reset
        self.draw_boundary()

    def add_soft_object_cb(self, vis):
        # Let's add a soft object at a random position within the boundary.
        position = np.random.rand(3) * self.boundary
        self.add_soft_object("cube.json", position)

    def add_rigid_object_cb(self, vis):
        # Add a rigid object.
        self.add_rigid_object()

    def increase_timestep_size(self, vis):
        self.timestep_size *= 1.1  # Increase timestep size by 10%
        print(f"New timestep size: {self.timestep_size}")

    def decrease_timestep_size(self, vis):
        self.timestep_size /= 1.1  # Decrease timestep size by ~9%
        print(f"New timestep size: {self.timestep_size}")

    def increase_num_substeps(self, vis):
        self.num_substeps += 1  # Increase number of substeps by 1
        print(f"New number of substeps: {self.num_substeps}")

    def decrease_num_substeps(self, vis):
        self.num_substeps = max(1, self.num_substeps - 1)  # Decrease number of substeps by 1, but keep it at least 1
        print(f"New number of substeps: {self.num_substeps}")

class SoftObject:
    def __init__(self, filename, position):
        # Load the vertices and triangles from the JSON file.
        with open(filename, "r") as f:
            data = json.load(f)
        vertices = np.array(data["vertices"])
        triangles = np.array(data["triangles"])

        # Create a TriangleMesh from the vertices and triangles.
        self.mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices),
            o3d.utility.Vector3iVector(triangles),
        )
        self.mesh.compute_vertex_normals()

        # Initialize the velocity of the soft object.
        self.velocity = np.random.rand(len(vertices), 3)  # random initial velocities

        # Calculate and store the initial volume of the object.
        self.initial_volume = self.calculate_volume()

        # Set the initial position of the object.
        self.mesh.translate(position, relative=False)

    @property
    def radius(self):
        # Calculate the radius as the maximum distance from the center to the vertices
        center = np.mean(np.asarray(self.mesh.vertices), axis=0)
        return np.max(np.linalg.norm(np.asarray(self.mesh.vertices) - center, axis=1))

    def update_position(self, dt):
        # Apply the velocity to update the position of the vertices.
        self.mesh.vertices = o3d.utility.Vector3dVector(np.asarray(self.mesh.vertices) + self.velocity * dt)

        # Apply gravity to update the velocity.
        self.velocity += np.array([0, -9.8, 0]) * dt

        # Calculate the current volume of the object.
        current_volume = self.calculate_volume()

        # Calculate the volume correction factor.
        correction_factor = (self.initial_volume / current_volume) ** (1 / 3)

        # Apply the volume correction factor to the vertices of the object.
        self.mesh.vertices = o3d.utility.Vector3dVector(np.asarray(self.mesh.vertices) * correction_factor)

    def handle_collision(self, boundary):
        vertices = np.asarray(self.mesh.vertices)
        velocities = self.velocity

        for i in range(3):
            out_of_bounds_min = vertices[:, i] < 0
            out_of_bounds_max = vertices[:, i] > boundary[i]

            # Reflect the velocity of vertices that hit the boundary.
            velocities[out_of_bounds_min, i] *= -1
            velocities[out_of_bounds_max, i] *= -1

            # Clamp the vertices to the boundary.
            vertices[out_of_bounds_min, i] = 0
            vertices[out_of_bounds_max, i] = boundary[i]

        # Update the vertices and velocities.
        self.mesh.vertices = o3d.utility.Vector3dVector(vertices)
        self.velocity = velocities

    def draw(self, vis):
        # Draw the mesh in the Open3D visualizer.
        vis.add_geometry(self.mesh)

    def calculate_volume(self):
        # Calculate the volume of the object by summing the volumes of all tetrahedrons formed by the triangles and a common point.
        volume = 0
        for triangle in np.asarray(self.mesh.triangles):
            v1, v2, v3 = [np.asarray(self.mesh.vertices)[i] for i in triangle]
            volume += np.abs(np.dot(v1, np.cross(v2, v3))) / 6
        return volume

class RigidObject:
    def __init__(self, radius=1):
        # Create a sphere mesh for the rigid object.
        self.mesh = o3d.geometry.TriangleMesh.create_sphere(radius)
        self.mesh.compute_vertex_normals()

        # Initialize the velocity and the position of the rigid object.
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([5.0, 5.0, 5.0])

        # Update the position of the mesh.
        self.mesh.translate(self.position, relative=False)

    @property
    def radius(self):
        # For a sphere, the radius is constant
        return self.mesh.get_max_bound()[0] - self.mesh.get_center()[0]

    def update_position(self, dt):
        # Apply the velocity to update the position.
        self.position += self.velocity * dt

        # Apply gravity to update the velocity.
        self.velocity += np.array([4, -9.8, 0]) * dt

        # Update the position of the mesh.
        self.mesh.translate(self.velocity * dt, relative=True)


    def handle_collision(self, boundary, position, velocity):
        for i in range(3):
            if position[i] < 0:
                position[i] = 0
                velocity[i] *= -1
            elif position[i] > boundary[i]:
                position[i] = boundary[i]
                velocity[i] *= -1


    def draw(self, vis):
        # Draw the mesh in the Open3D visualizer.
        vis.add_geometry(self.mesh)


if __name__ == "__main__":
    sim = Simulator()
    sim.add_soft_object("cube.json", position=np.array([5.0, 5.0, 5.0]))
    sim.add_rigid_object()
    sim.vis.run()