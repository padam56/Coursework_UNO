# D* Algorithm Implementation for CoppeliaSim

This repository contains the implementation of the D* (Dynamic A*) path planning algorithm, specifically tailored for use within the CoppeliaSim simulation environment. The code facilitates the creation of a workspace, transformation of obstacle coordinates, and visualization of a path between two points.

## Workspace Specification

The workspace is defined within the following boundaries in CoppeliaSim (DEFAULT): 

- X-axis minimum (xmin): -2.5
- X-axis maximum (xmax): 2.5
- Y-axis minimum (ymin): -2.5
- Y-axis maximum (ymax): 2.5

This workspace corresponds to the default floor provided in CoppeliaSim.

## Implementation Details

- The obstacles' coordinates are transformed to fit within the specified CoppeliaSim workspace.
- Two cuboid objects are created to represent the Start and Goal within the simulation.
- The D* algorithm is employed to find a path connecting the Start and Goal locations.
- A safety parameter is introduced to maintain a safe distance from the workspace boundaries.

## CoppeliaSim APIs Used

- `sim.createPrimitiveShape`: To create obstacles and boundary objects within the simulation.
- `sim.addDrawingObject`: To initialize the rendering of the path.
- `sim.addDrawingObjectItem`: To render the path points in the simulation.

## Instructions for Running the Simulation

1. Install CoppeliaSim from the official website.
2. Clone this repository to your local machine.
3. Open CoppeliaSim and load the `.ttt` scene file provided in the HW4 folder of the repository.
4. Execute the Python script to run the D* algorithm and visualize the path planning in CoppeliaSim.

## Homework Submission

- The Python code is written in compliance with the project requirements and is uploaded separately from the README file.
- The source files, including `*.py` and `*.ttt`, are located in the HW4 folder of the GitLab repository.