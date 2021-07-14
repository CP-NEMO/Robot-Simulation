"""odometer controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

def run_robot(robot):

    timestamp = 64
    max_speed = 6.28
    # creating motor instances
    left_motor = robot.getDevice('Motor_1')
    right_motor = robot.getDevice('Motor_2')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # Create position sensor instances
    left_ps = robot.getPositionSensor('ps_1')
    left_ps.enable(timestamp)
    
    right_ps = robot.getPositionSensor('ps_2')
    right_ps.enable(timestamp)
    
    ps_values = [0,0]
    
    dist_values = [0,0]
    
    wheel_radius = 0.025
    distance_between_wheels = 0.09
    
    wheel_cirum = 2*3.14*wheel_radius
    encoder_unit = wheel_cirum/6.28
    
    robot_pose = [0,0,0] # x, y, theta
    last_ps_values = [0,0]
    
    while robot.step(timestamp) != -1:
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        
        print("----------------------------")
        print("position sensor values: {} {}" . format(ps_values[0], ps_values[1]))
        
        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 0.001:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] = ps_values[ind] * encoder_unit
    
        v = (dist_values[0] + dist_values[1])/2.0    
        w = (dist_values[0] - dist_values[1])/distance_between_wheels
        
        dt = 1
        robot_pose[2] += (w * dt)
        
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += (vx * dt)
        robot_pose[1] += (vy * dt)
        
        print("robot_pose: {}" .format(robot_pose))
            
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(-max_speed)
        
        for ind in range(2):
            last_ps_values[ind] = ps_values[ind]
    
    
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)