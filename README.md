# Graphics and Simulation Toolkit

## Boids Model 3D Simulation

![Banner Image](/images/banner.png)

In this project, the Boids algorithm is implemented to simulate the flocking behavior of birds in a 3D environment. The Boids model was originally proposed by Craig Reynolds and aims to mimic natural flocks such as groups of birds or schools of fish.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## Overview

Boids is an artificial life program that simulates the flocking behavior of birds. This implementation uses the Boids algorithm to demonstrate how complex behaviors can emerge from the interaction of simple rules.

Read Craig Reynold's original paper [here](http://www.red3d.com/cwr/papers/1987/boids.html).

## Features

### Basic Behaviors
- **Separation**: Steer to avoid crowding local flockmates.
- **Alignment**: Steer towards the average heading of local flockmates.
- **Cohesion**: Steer to move toward the average position (center-of-mass) of local flockmates.
- **Boundary**: The agent does not move beyond a specified rectangular or cubic boundary.

### Additional Behaviors
- **Goal Seeking**: Define a moving goal and implement behavior to move toward the goal.
- **Predators**: Define one or multiple moving predators and implement behavior to dodge them.
- **Obstacle Avoidance**: Define obstacles in the scene with primitive geometrics (e.g. sphere, box) and implement behaviors to dodge them.

### Optimization
- **KD-Tree for Large-scale Simulation**: Implement KD-Tree to optimize performance in large-scale simulations.

## Installation

To clone and run this application, you'll need Git and any prerequisites your project may have.

```bash
# Clone this repository
git clone https://github.com/yourusername/boids-model-3d-simulation.git