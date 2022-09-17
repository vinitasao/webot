from controller import Motor, Supervisor, Display, math
import numpy as np
import math
TIME_STEP = 64
MAX_SPEED = 6.28

robot = Supervisor()
goal = np.array([1.23,0.006,0.000])
source = np.array([-1.13, 0.003,0.000])
object = robot.getFromDef('epuck') 
epuck_orientation = object.getOrientation()
heading_angle = np.arctan2(epuck_orientation[0], epuck_orientation[3])
heading_angle = np.array([np.cos(heading_angle),np.sin(heading_angle)])

def draw_line(self,x1,y1,x2,y2):
            x_values = [x1, x2]
            y_values = [y1, y2]
            
            plt.plot(x_values, y_values, 'bo', linestyle = '--')
            
    

m_line = draw_line(-0.0289,1.1411,-0.001188, -0.11573 )
print(m_line)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
    
 #initialize devices
ps = []
psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)    
    

def perpendicular_distance(x, y, a, b, c):
    d = abs((a*x + b*y + c ))/math.sqrt((a*a + b*b))
    return d





while robot.step(TIME_STEP) != -1:
    gps_value = gps.getValues()
    #print(gps_value)
       
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    direct = goal - current_loc
    direct = direct[:2]     
    object = robot.getFromDef('epuck') 
    epuck_orientation = object.getOrientation()
    heading_angle = np.arctan2(epuck_orientation[0], epuck_orientation[3])
    heading_angle = np.array([np.cos(heading_angle),np.sin(heading_angle)])    
    angle = np.dot(direct, heading_angle)
    angle = np.cos(angle)
    direction = np.cross(direct, heading_angle)
    
        
    #detect obstacles
    thre = 80
    back_obstacle = (psValues[3] > thre or psValues[4] >thre)
    front_obstacle =  (psValues[7] > thre or psValues[0] >thre)
    right_obstacle = ( psValues[7] > thre or psValues[0] >thre)
    left_obstacle = (psValues[7] > thre or psValues[0] >thre)
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    
    if perpendicular_distance(gps_value[0], gps_value[1],300, -236000, 1047) <= 0.001:
        while (heading_angle - direct) <= 0.001:
            leftSpeed = -0.5* MAX_SPEED
            rightSpeed = 0.5*MAX_SPEED
         
         
     leftSpeed = 0.5 * MAX_SPEED
     rightSpeed = 0.5 * MAX_SPEED
     leftMotor.setVelocity(leftSpeed)
     rightMotor.setVelocity(rightSpeed)
    else:
        if front_obstacle:
            leftSpeed = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            
        elif right_obstacle:
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
            
        else: 
            leftSpeed = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
     
     
     
     
     
     
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)  
