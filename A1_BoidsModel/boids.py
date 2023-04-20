import pygame
import pygame_gui
import random
import math
from pygame.locals import QUIT
from pygame.draw import circle

# Initialize global variables
# Step 1 
width = 800
height = 600
num_boids = 100
visual_range = 75
boids = []
centering_factor = 0
avoid_factor = 0
matching_factor = 0
# Step 2 
goal = {"x": width / 2, "y": height / 2, "dx": 0, "dy": 0}
predator = {"x": random.random() * width, "y": random.random() * height, "dx": random.uniform(-1, 1), "dy": random.uniform(-1, 1)}
obstacles = [{"x": random.random() * width, "y": random.random() * height, "radius": 20} for _ in range(2)]

# Define a function to initialize the boids
def init_boids():
    for _ in range(num_boids):
        boids.append({
            "x": random.uniform(100, width - 100),
            "y": random.uniform(100, height - 100),
            "dx": random.random() * 10 - 5,
            "dy": random.random() * 10 - 5,
            "history": [],
        })
    global obstacles
    obstacles = [{"x": random.uniform(100 + r, width - 100 - r), "y": random.uniform(100 + r, height - 100 - r), "radius": r} for r in [20, 20, 20, 20, 20]]

# Define a function to calculate the distance between two boids
def distance(boid1, boid2):
    return math.sqrt(
        (boid1["x"] - boid2["x"]) * (boid1["x"] - boid2["x"]) +
        (boid1["y"] - boid2["y"]) * (boid1["y"] - boid2["y"])
    )

# Define a function to keep the boids within the bounds of the screen
def keep_within_bounds(boid):
    margin = 200
    turn_factor = 1

    if boid["x"] < margin:
        boid["dx"] += turn_factor
    if boid["x"] > width - margin:
        boid["dx"] -= turn_factor
    if boid["y"] < margin:
        boid["dy"] += turn_factor
    if boid["y"] > height - margin:
        boid["dy"] -= turn_factor

# Define a function to make the boids fly towards the center of mass of nearby boids
def fly_towards_center(boid):
    global centering_factor

    center_x = 0
    center_y = 0
    num_neighbors = 0

    for other_boid in boids:
        if distance(boid, other_boid) < visual_range:
            center_x += other_boid["x"]
            center_y += other_boid["y"]
            num_neighbors += 1

    if num_neighbors:
        center_x = center_x / num_neighbors
        center_y = center_y / num_neighbors

        boid["dx"] += (center_x - boid["x"]) * centering_factor
        boid["dy"] += (center_y - boid["y"]) * centering_factor

def avoid_others(boid):
    global avoid_factor

    min_distance = 20  # The distance to stay away from other boids
    move_x = 0
    move_y = 0
    for other_boid in boids:
        if other_boid != boid:
            if distance(boid, other_boid) < min_distance:
                move_x += boid["x"] - other_boid["x"]
                move_y += boid["y"] - other_boid["y"]

    # Avoid the predator
    if distance(boid, predator) < visual_range:
        move_x += boid["x"] - predator["x"]
        move_y += boid["y"] - predator["y"]

    boid["dx"] += move_x * avoid_factor
    boid["dy"] += move_y * avoid_factor

def match_velocity(boid):
    global matching_factor

    avg_dx = 0
    avg_dy = 0
    num_neighbors = 0

    for other_boid in boids:
        if distance(boid, other_boid) < visual_range:
            avg_dx += other_boid["dx"]
            avg_dy += other_boid["dy"]
            num_neighbors += 1

    if num_neighbors:
        avg_dx = avg_dx / num_neighbors
        avg_dy = avg_dy / num_neighbors

        boid["dx"] += (avg_dx - boid["dx"]) * matching_factor
        boid["dy"] += (avg_dy - boid["dy"]) * matching_factor

def limit_speed(boid):
    speed_limit = 1

    speed = math.sqrt(boid["dx"] * boid["dx"] + boid["dy"] * boid["dy"])
    if speed > speed_limit:
        boid["dx"] = (boid["dx"] / speed) * speed_limit
        boid["dy"] = (boid["dy"] / speed) * speed_limit

def draw_boid(screen, boid):
    angle = math.atan2(boid["dy"], boid["dx"])
    x, y = int(boid["x"]), int(boid["y"])
    
    # Define the boid's triangle points relative to the center
    points = [
        (0, 0),
        (-15, 5),
        (-15, -5),
    ]
    
    # Rotate and translate the points
    rotated_points = []
    for px, py in points:
        rotated_x = px * math.cos(angle) - py * math.sin(angle)
        rotated_y = px * math.sin(angle) + py * math.cos(angle)
        translated_x = rotated_x + x
        translated_y = rotated_y + y
        rotated_points.append((translated_x, translated_y))

    pygame.draw.polygon(screen, (85, 140, 244), rotated_points)

def bound_position(boid):
    Xmin, Xmax, Ymin, Ymax = 100, width - 100, 100, height - 100
    v = {"x": 0, "y": 0}

    if boid["x"] < Xmin:
        v["x"] = 10
    elif boid["x"] > Xmax:
        v["x"] = -10

    if boid["y"] < Ymin:
        v["y"] = 10
    elif boid["y"] > Ymax:
        v["y"] = -10

    return v

def keep_within_bounds(boid):
    v = bound_position(boid)
    boid["dx"] += v["x"]
    boid["dy"] += v["y"]

def seek_goal(boid):
    global goal
    goal_seeking_factor = 0.005

    boid["dx"] += (goal["x"] - boid["x"]) * goal_seeking_factor
    boid["dy"] += (goal["y"] - boid["y"]) * goal_seeking_factor

def dodge_predators(boid):
    global predator

    predator_avoidance_factor = 0.05
    predator_avoidance_distance = 50  # The distance to stay away from the predator

    # Check if the predator is within the avoidance distance
    if distance(boid, predator) < predator_avoidance_distance:
        move_x = boid["x"] - predator["x"]
        move_y = boid["y"] - predator["y"]
        boid["dx"] += move_x * predator_avoidance_factor
        boid["dy"] += move_y * predator_avoidance_factor

def dodge_obstacles(boid):
    global obstacles
    obstacle_avoidance_factor = 0.05
    obstacle_avoidance_distance = 1

    for obstacle in obstacles:
        if distance(boid, obstacle) < obstacle_avoidance_distance + obstacle["radius"]:
            boid["dx"] += (boid["x"] - obstacle["x"]) * obstacle_avoidance_factor
            boid["dy"] += (boid["y"] - obstacle["y"]) * obstacle_avoidance_factor

def main():
    global width, height, centering_factor, avoid_factor, matching_factor
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Boids")

    manager = pygame_gui.UIManager((width, height))

    coherence_label = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect((20, 5), (150, 20)),
        text="Coherence",
        manager=manager,
    )
    coherence_slider = pygame_gui.elements.UIHorizontalSlider(
        relative_rect=pygame.Rect((20, 30), (150, 20)),
        start_value=centering_factor,
        value_range=(0, 0.01),
        manager=manager,
    )
    separation_label = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect((20, 55), (150, 20)),
        text="Separation",
        manager=manager,
    )
    separation_slider = pygame_gui.elements.UIHorizontalSlider(
        relative_rect=pygame.Rect((20, 80), (150, 20)),
        start_value=avoid_factor,
        value_range=(0, 0.1),
        manager=manager,
    )
    alignment_label = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect((20, 105), (150, 20)),
        text="Alignment",
        manager=manager,
    )
    alignment_slider = pygame_gui.elements.UIHorizontalSlider(
        relative_rect=pygame.Rect((20, 130), (150, 20)),
        start_value=matching_factor,
        value_range=(0, 0.01),
        manager=manager,
    )

    coherence_value_label = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect((175, 30), (50, 20)),
        text=f"{centering_factor:.4f}",
        manager=manager,
    )
    separation_value_label = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect((175, 80), (50, 20)),
        text=f"{avoid_factor:.4f}",
        manager=manager,
    )
    alignment_value_label = pygame_gui.elements.UILabel(
        relative_rect=pygame.Rect((175, 130), (50, 20)),
        text=f"{matching_factor:.4f}",
        manager=manager,
    )

    init_boids()

    clock = pygame.time.Clock()
    running = True

    while running:
        time_delta = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                goal["x"], goal["y"] = pygame.mouse.get_pos()

            manager.process_events(event)

        centering_factor = coherence_slider.get_current_value()
        avoid_factor = separation_slider.get_current_value()
        matching_factor = alignment_slider.get_current_value()
        coherence_value_label.set_text(f"{centering_factor:.4f}")
        separation_value_label.set_text(f"{avoid_factor:.4f}")
        alignment_value_label.set_text(f"{matching_factor:.4f}")

        screen.fill((255, 255, 255))
        pygame.draw.rect(screen, (0, 0, 0), (100, 100, width - 200, height - 200), 1)
        for boid in boids:
            fly_towards_center(boid)
            avoid_others(boid)
            match_velocity(boid)
            limit_speed(boid)
            keep_within_bounds(boid)
            # seek_goal(boid)
            # dodge_predators(boid)
            # dodge_obstacles(boid)

            boid["x"] += boid["dx"]
            boid["y"] += boid["dy"]
            boid["history"].append((boid["x"], boid["y"]))
            boid["history"] = boid["history"][-50:]

            draw_boid(screen, boid)

        # goal["x"] += goal["dx"]
        # goal["y"] += goal["dy"]
        # goal["dx"] = (random.random() * 2 - 1) * 0.5
        # goal["dy"] = (random.random() * 2 - 1) * 0.5


        # v = bound_position(predator)
        # predator["dx"] += v["x"]
        # predator["dy"] += v["y"]
        # predator["x"] += predator["dx"]
        # predator["y"] += predator["dy"]
        # bound_position(predator)

        # Draw goal
        # pygame.draw.circle(screen, (0, 255, 0), (int(goal["x"]), int(goal["y"])), 10)

        # Draw predators
        # pygame.draw.circle(screen, (255, 0, 0), (int(predator["x"]), int(predator["y"])), 10)

        # # Draw obstacles
        # for obstacle in obstacles:
        #     pygame.draw.circle(screen, (0, 0, 255), (int(obstacle["x"]), int(obstacle["y"])), obstacle["radius"])     

        manager.update(time_delta)
        manager.draw_ui(screen)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
