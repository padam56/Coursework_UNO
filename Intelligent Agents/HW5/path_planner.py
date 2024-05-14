#python
import math
import numpy as np
from sys import maxsize

#Define constants
OBSTACLE_SIZE = [0.2, 0.2, 0.2]
OBSTACLE_HT   = 0.3
MARKER_SIZE   = [0.2, 0.2, 0.2]
START_COLOR   = [0, 1, 0]
GOAL_COLOR    = [0, 0, 1]
MARKER_HT     = 0.2
PATH_LINE_SIZE = 4
PATH_COLOR    = [0, 1, 1]
MAXIMUM_LINES = 5000
PATH_ALTITUDE = 0.15

def sysCall_init():
    sim = require('sim')


def get_preimage(x, y):
  """
  Transforms coordinates from algorithm space back to simulation environment

  Parameters: float x, float y (coordinates in algorithm space)
  Returns: float x_preimg, float y_preimg (coordinates in simulation)
  """


  def preimage_map(x):
    return (x - 50) / 10
    
  x_preimg = preimage_map(x)
  y_preimg = preimage_map(y)
  return x_preimg, y_preimg
  

# D-star algorithm implementation

"""

D* grid planning

author: Nirnay Roy

See Wikipedia article (https://en.wikipedia.org/wiki/D*)

"""

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list, dilation=0):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")
            if dilation > 0:
                for x_inc in range(dilation):
                    for y_inc in range(dilation):
                        try:
                            self.map[x+x_inc][y+y_inc].set_state("#")
                            self.map[x+x_inc][y-y_inc].set_state("#")
                            self.map[x-x_inc][y+y_inc].set_state("#")
                            self.map[x-x_inc][y-y_inc].set_state("#")
                        except: IndexError


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        if k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(x, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        AddNewObstacle(self.map) # add new obstacle after the first search finished

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def AddNewObstacle(map:Map):
    ox, oy = [], []
    for i in range(5, 21):
        ox.append(i)
        oy.append(40)
    map.set_obstacle([(i, j) for i, j in zip(ox, oy)])
    
#|---------End of D-star algorithm from PythonRobotics---------|


def build_map(ox, oy, start, goal):
    """
    Builds all the obstacles on 10 x 10 floor in Coppeliasim including the borders

    Parameters:
    - ox: x co-ordinates of all the obstacles
    - oy: y co-ordinates of all the obstacles
    - start: co-ordinate of start location
    - goal: co-ordinate of goal location
    """

    
    # Display obstacles
    obstacle_handles = []
    obstacle_type = sim.primitiveshape_cuboid
    
    for obstacle in enumerate(zip(ox, oy)):
        obstacle_handle = sim.createPrimitiveShape(obstacle_type, OBSTACLE_SIZE)
        
        # Convert to simulation coordinates

        x_preimg, y_preimg = get_preimage(obstacle[1][0], obstacle[1][1])
        
        sim.setObjectPosition(obstacle_handle, -1, [x_preimg, y_preimg, OBSTACLE_HT]) 
        obstacle_handles.append(obstacle_handle)
    
    group_handle = sim.groupShapes(obstacle_handles, True)
    sim.setObjectAlias(group_handle, 'Obstacles')
    
    # Display start and goal markers
    
    start_object_type = sim.primitiveshape_cuboid
    goal_object_type = sim.primitiveshape_cuboid
    
    start_handle = sim.createPrimitiveShape(start_object_type, MARKER_SIZE)
    goal_handle = sim.createPrimitiveShape(goal_object_type, MARKER_SIZE)
    
    sx_preimg, sy_preimg = get_preimage(start[0], start[1])
    gx_preimg, gy_preimg = get_preimage(goal[0], goal[1])
    
    sim.setObjectPosition(start_handle, -1, [sx_preimg, sy_preimg, MARKER_HT])
    sim.setObjectPosition(goal_handle, -1, [gx_preimg, gy_preimg, MARKER_HT])
    
    
def display_path(preimg_path_points):
    """
    Visualizes the calculated route in the simulation environment
    Parameter:
    - path_points : coordinates of the path calculated by the algorithm
    """
    
    path_object_handle = sim.addDrawingObject(sim.drawing_lines, PATH_LINE_SIZE, 0.0, -1, MAXIMUM_LINES, PATH_COLOR)
    
    for i, point in enumerate(preimg_path_points):
        if i > 0:
            last = preimg_path_points[i-1].tolist()
            last.append(PATH_ALTITUDE)
            
            current = point.tolist()
            current.append(PATH_ALTITUDE)
            
            last.extend(current)
            sim.addDrawingObjectItem(path_object_handle, last)    


def sysCall_thread():
    
# set dilation here
    dilation = 5
    
    # initialize map
    m = Map(100, 100)
    
    ox, oy = [], []
    
    for i in range(100):
      ox.append(i)
      oy.append(0)

    for i in range(100):
      ox.append(i)
      oy.append(100)

    for i in range(100):
        ox.append(100)
        oy.append(i)

    for i in range(100):
        ox.append(0)
        oy.append(i)

    for i in range(60):
        ox.append(30)
        oy.append(i)

    for i in range(30, 100):
        ox.append(60)
        oy.append(i)

    m.set_obstacle([(i, j) for i, j in zip(ox, oy)], dilation)
    
    start = (20, 20)
    goal = (80, 80)
    
    build_map(ox, oy, start, goal)
    
    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    
    
    path_points = np.column_stack((rx, ry))
        
    # convert all path_points to coppeliasim coordinates
    preimg_path_points = []
    
    for i in range(path_points.shape[0]):
        preimg_path_points.append(list(get_preimage(path_points[i][0], path_points[i][1])))
    
    preimg_path_points = np.array(preimg_path_points)
    
    
    rz = np.tile(np.array([0.1]), (preimg_path_points.shape[0], 1))
    qt = np.tile(np.array([0, 0, 0, 1]), (preimg_path_points.shape[0], 1))
    
    
    path_coordinates = np.concatenate((preimg_path_points, rz, qt), 1)
    
    path = path_coordinates.flatten().tolist()
    
    self.path = sim.createPath(path)
    
    sim.setInt32Signal('path', self.path)
