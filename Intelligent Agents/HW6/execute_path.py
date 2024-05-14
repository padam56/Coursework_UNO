#python
import numpy as np
from math import atan2

def sysCall_init():
    sim = require('sim')
    self.object_to_follow_path0 = sim.getObject('/Cuboid[0]')
    self.object_to_follow_path1 = sim.getObject('/Cuboid[1]')
    
    self.velocity = 0.1
    self.posAlongPath0 = 0
    self.posAlongPath1 = 0
    
    self.previousSimulationTime = 0
    
    self.path_found = False
    self.stop_condition0 = 1
    sim.setStepping(True)
    
    # initialization for PioneerP3DX
    # get handle for left and right motors
    self.motorLeft0=sim.getObject("/PioneerP3DX[0]/leftMotor")
    self.motorRight0=sim.getObject("/PioneerP3DX[0]/rightMotor")
    
    self.motorLeft1=sim.getObject("/PioneerP3DX[1]/leftMotor")
    self.motorRight1=sim.getObject("/PioneerP3DX[1]/rightMotor")
    
    # pioneerP3DX handle
    self.robot_handle0 = sim.getObject("/PioneerP3DX[0]")
    self.robot_handle1 = sim.getObject("/PioneerP3DX[1]")
    
    # initialize control params
    self.Kp_rho = 1.0
    self.Kp_alpha = 0.1 # decreased kp_alpha and kp_rho because of high oscillations
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
    
    #print(f'v = {v}')
    #print(f'w = {w}')
    
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
            self.stop_condition0 = 0
            self.stop_condition1 = 0
            sim.setInt32Signal('stop_condition0', self.stop_condition0)
            sim.setInt32Signal('stop_condition1', self.stop_condition1)
            
            pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle, 'PATH'))
            self.path = pathHandle
            
            m = np.array(pathData).reshape(len(pathData) // 7, 7)
            #print(f'm.shape={m.shape}')
            
            m_rev = np.flip(m, axis=0)
            #print(f'm_rev.shape={m_rev.shape}')
            
            self.pathPositions0 = m[:, :3].flatten().tolist()
            self.pathQuaternions0 = m[:, 3:].flatten().tolist()
            
            self.pathPositions1 = m_rev[:, :3].flatten().tolist()
            self.pathQuaternions1 = m_rev[:, 3:].flatten().tolist()
            
            self.pathLengths0, self.totalLength0 = sim.getPathLengths(self.pathPositions0, 3)
            self.pathLengths1, self.totalLength1 = sim.getPathLengths(self.pathPositions1, 3)
            
            self.start_point0 = m[0][:3]
            self.start_point0[1] = self.start_point0[1] - 1 # to put the robot behind the object following path
            
            self.start_point1 = m_rev[0][:3]
            self.start_point1[1] = self.start_point1[1] + 1
            
            sim.setObjectPosition(self.robot_handle0, self.start_point0, -1)
            sim.setObjectOrientation(self.robot_handle0, [0, 0, 180], -1)
            
            sim.setObjectPosition(self.robot_handle1, self.start_point1, -1)
            sim.setObjectOrientation(self.robot_handle1, [0, 0, 90], -1)

        
        t = sim.getSimulationTime() # check this
        
        if self.path_found and self.stop_condition0 == 0:
            
            self.posAlongPath0 += self.velocity * (t - self.previousSimulationTime)
            self.posAlongPath0 %= self.totalLength0
    
            
            # check if the path ends
            if round(self.posAlongPath0, 2) == 0:
                self.stop_condition0 = 1
                sim.setInt32Signal('stop_condition0', self.stop_condition0)
                
            
            pos0 = sim.getPathInterpolatedConfig(self.pathPositions0, self.pathLengths0, self.posAlongPath0)
            quat0 = sim.getPathInterpolatedConfig(self.pathQuaternions0, self.pathLengths0, 
                self.posAlongPath0, None, [2, 2, 2, 2])
                
            sim.setObjectPosition(self.object_to_follow_path0, pos0, self.path)
            sim.setObjectQuaternion(self.object_to_follow_path0, quat0, self.path)
            
            
        if self.path_found and self.stop_condition1 == 0:
            
            self.posAlongPath1 += self.velocity * (t - self.previousSimulationTime)
            #print(f'totalLength1 = {self.totalLength1}')
            self.posAlongPath1 %= self.totalLength1
            
            
            # check if the path ends
                
            if round(self.posAlongPath1, 2) == 0:
                self.stop_condition1 = 1
                sim.setInt32Signal('stop_condition1', self.stop_condition1)
            
            pos1 = sim.getPathInterpolatedConfig(self.pathPositions1, self.pathLengths1, self.posAlongPath1)
            quat1 = sim.getPathInterpolatedConfig(self.pathQuaternions1, self.pathLengths1, 
                self.posAlongPath1, None, [2, 2, 2, 2])
            
            sim.setObjectPosition(self.object_to_follow_path1, pos1, self.path)
            sim.setObjectQuaternion(self.object_to_follow_path1, quat1, self.path)
            
        # make PioneerP3DX follow cuboid
        target0 = getTargetPosition('/Cuboid[0]')
        robot0 = getTargetPosition('/PioneerP3DX[0]')
        
        target1 = getTargetPosition('/Cuboid[1]')
        robot1 = getTargetPosition('/PioneerP3DX[1]')
        
        x_diff0 = target0[0] - robot0[0]
        y_diff0 = target0[1] - robot0[1]
        
        x_diff1 = target1[0] - robot1[0]
        y_diff1 = target1[1] - robot1[1]
        
        theta_goal0 = atan2(y_diff0, x_diff0)
        theta0 = getTargetOrientation('/PioneerP3DX[0]')[-1]
        
        theta_goal1 = atan2(y_diff1, x_diff1)
        theta1 = getTargetOrientation('/PioneerP3DX[1]')[-1]
        
        rho0, v0, w0 = calc_control_command(x_diff0, y_diff0, theta0, theta_goal0)
        rho1, v1, w1 = calc_control_command(x_diff1, y_diff1, theta1, theta_goal1)
            
        #print(f'rho = {rho}')
        # set right and left motor velocity
        vLeft0, vRight0 = unitodiff(v0, w0)
        vLeft1, vRight1 = unitodiff(v1, w1)
        
        #print(f'vLeft: {vLeft0}')
        #print(f'vRight: {vRight0}')        
        #print(f'sent vLeft={vLeft}')
        # send the motor velocities to robot
        sim.setFloatSignal('left-vel0', vLeft0)
        sim.setFloatSignal('right-vel0', vRight0)
        
        sim.setFloatSignal('left-vel1', vLeft1)
        sim.setFloatSignal('right-vel1', vRight1)
        
        #sim.setJointTargetVelocity(self.motorLeft,vLeft)
        #sim.setJointTargetVelocity(self.motorRight,vRight)
                
            
        self.previousSimulationTime = t
        sim.step()
# See the user manual or the available code snippets for additional callback functions and details
