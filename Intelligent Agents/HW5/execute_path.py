#python
import numpy as np
from math import atan2

def sysCall_init():
    sim = require('sim')
    self.object_to_follow_path = sim.getObject('/Cuboid')
    
    self.velocity = 0.2
    self.posAlongPath = 0
    self.previousSimulationTime = 0
    
    self.path_found = False
    sim.setStepping(True)
    
    # initialization for PioneerP3DX
    # get handle for left and right motors
    self.motorLeft=sim.getObject("/PioneerP3DX/leftMotor")
    self.motorRight=sim.getObject("/PioneerP3DX/rightMotor")
    
    # initialize control params
    self.Kp_rho = 1.4
    self.Kp_alpha = 0.22
    self.Kp_beta = 0
    
    # initialize other params
    self. length = 0.381
    self. radius = 0.195 / 2.0
    
    
# define functions to follow cuboid

def getTargetPosition(object_name):
    """
    Returns the target objects position
    """
    
    objHandle = sim.getObject(object_name)
    
    targetPos = sim.getObjectPosition(objHandle, -1)
    return targetPos
    
def getTargetOrientation(object_name):
    """
    Returns the target object's orientation
    """
    objHandle = sim.getObject(object_name)
    
    return sim.getObjectOrientation(objHandle, -1)
    
def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle
    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle
        

def unitodiff(v, w):
    
    """
    Returns the left and right wheel velocity
    """
    vR = 2 * v + w * self.length / (2 * self.radius)
    vL = 2 * v - w * self.length / (2 * self.radius) 
    
    return vR, vL
    

def calc_control_command(x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w
        
    
def sysCall_thread():
    while not sim.getSimulationStopping():
        pathHandle = sim.getInt32Signal('path')
        
        if pathHandle is not None and not self.path_found:
            
            self.path_found = True
            pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle, 'PATH'))
            self.path = pathHandle
            
            m = np.array(pathData).reshape(len(pathData) // 7, 7)

            self.pathPositions = m[:, :3].flatten().tolist()
            self.pathQuaternions = m[:, 3:].flatten().tolist()
            
            self.pathLengths, self.totalLength = sim.getPathLengths(self.pathPositions, 3)
        
        t = sim.getSimulationTime() # check this
        
        if self.path_found:
            
            self.posAlongPath += self.velocity * (t - self.previousSimulationTime)
            self.posAlongPath %= self.totalLength
            pos = sim.getPathInterpolatedConfig(self.pathPositions, self.pathLengths, self.posAlongPath)
            quat = sim.getPathInterpolatedConfig(self.pathQuaternions, self.pathLengths, 
                self.posAlongPath, None, [2, 2, 2, 2])
            
            sim.setObjectPosition(self.object_to_follow_path, pos, self.path)
            sim.setObjectQuaternion(self.object_to_follow_path, quat, self.path)
            
        # make PioneerP3DX follow cuboid
        target = getTargetPosition('/Cuboid')
        robot = getTargetPosition('/PioneerP3DX')
        
        x_diff = target[0] - robot[0]
        y_diff = target[1] - robot[1]
        
        theta_goal = atan2(y_diff, x_diff)
        theta = getTargetOrientation('/PioneerP3DX')[-1]
        
        rho, v, w = calc_control_command(x_diff, y_diff, theta, theta_goal)
            
        print(f'rho = {rho}')
        # set right and left motor velocity
        vLeft, vRight = unitodiff(v, w)
        
        sim.setJointTargetVelocity(self.motorLeft,vLeft)
        sim.setJointTargetVelocity(self.motorRight,vRight)
                
            
        self.previousSimulationTime = t
        sim.step()
# See the user manual or the available code snippets for additional callback functions and details
