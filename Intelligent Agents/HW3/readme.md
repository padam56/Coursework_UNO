# CoppeliaSim Project README

## Overview
This project demonstrates a basic implementation of a robot navigation system in CoppeliaSim, focusing on a Pioneer P3DX robot. It calculates the control commands for the robot to navigate towards a target object (Cuboid) based on the robot's current position and orientation. The control strategy is based on a simple proportional controller.

## Requirements
- CoppeliaSim (formerly known as V-REP)
- Python scripting support in CoppeliaSim

## Files
- `HW3.py`: The main Python script that contains all the functions for robot navigation.
- `HW3.ttt`: The CoppeliaSim scene file that contains the simulation environment and embedded scripts.

## Functionality
The script performs the following key functions:
1. **Initialization (`sysCall_init`)**: Sets up the robot by retrieving object handles for the motors and initializing control parameters.
2. **Actuation (`sysCall_actuation`)**: Calculates the control commands to navigate the robot towards the target and applies these commands to the robot's motors.
3. **Sensing (`sysCall_sensing`)**: Placeholder for future implementation of sensing functionalities.
4. **Cleanup (`sysCall_cleanup`)**: Placeholder for cleanup operations, if necessary.

## Key Functions
- `angle_mod`: Normalizes angles to a specified range.
- `unitodiff`: Converts linear and angular velocities to individual wheel velocities for a differential drive robot.
- `getTargetPosition` and `getTargetOrientation`: Retrieve the position and orientation of a specified object in the simulation.
- `calc_control_command`: Calculates the control commands based on the current state of the robot and the target position.

## How to Run
1. Open CoppeliaSim and load the `HW3.ttt` scene file.
2. The scene should contain the Pioneer P3DX robot and a Cuboid object as the target.
3. Attach the `HW3.py` to the Pioneer P3DX robot as a child script if not already attached.
4. Start the simulation by pressing the 'Play' button on the CoppeliaSim toolbar. The robot should navigate towards the Cuboid target.

## Notes
- The control parameters (`Kp_rho`, `Kp_alpha`, `Kp_beta`) can be adjusted to change the robot's navigation behavior.
- This script assumes the robot is equipped with a differential drive system.
- The navigation strategy is basic and may need adjustments for complex scenarios or environments.
