#python

import numpy as np
import random

# Define the obstacle as a polygon with vertices at the specified coordinates
polygon = np.array([[-2.5, 2.5],[0.0, 0.83333333],[2.5, -2.5],[-2.5, 0.83333333]])
boundary = np.array([[-5, -5], [5, -5], [5, 5], [-5, 5]])

# Function to check if a point is inside the polygon
def is_point_inside_polygon(point):
  
  x, y = point
  inside = False

  n = len(polygon)
  p1x, p1y = polygon[0]
  for i in range(n+1):
      p2x, p2y = polygon[i % n]
      if y > min(p1y, p2y):
          if y <= max(p1y, p2y):
              if x <= max(p1x, p2x):
                  if p1y != p2y:
                      xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                  if p1x == p2x or x <= xints:
                      inside = not inside
      p1x, p1y = p2x, p2y

  return inside
    
    
def check_boundary(npos):
    x, y = npos[0], npos[1]
    if x >= -5 and x <= 5 and y >= -5 and y <= 5:
        return True  # if object is inside boundary
    else:
        return False
    
# Initializing the simulation
def sysCall_init():
    # Require the 'sim' module which provides the API to communicate with CoppeliaSim
    sim = require('sim')

# Main function to control the robot
def sysCall_thread():
    # Get the handle of the target object in the simulation
    target = sim.getObject("/target")
    
    # Defining the action space, which are the possible movements the robot can make
    U = np.array([[0.01, 0], 
                  [0, 0.01], 
                  [-0.01, 0], 
                  [0, -0.01]]) # action space
    
    # Main loop to control the robot until the simulation stops
    while not sim.getSimulationStopping():
        # Get the current position of the target
        p = sim.getObjectPosition(target, -1)
        # Choose a random action from the action space
        u = random.choice(U)
        # Calculate the new position of the target
        npos = [p[0] + u[0], p[1] + u[1], 0.5]
        
        if not is_point_inside_polygon(npos[:2]) and check_boundary(npos[:2]):
            # Set the new position of the target
            sim.setObjectPosition(target, -1, npos)
        else:
            print(npos)
            
        sim.step()