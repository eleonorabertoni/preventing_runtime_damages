from controller import Robot
from controller import Accelerometer
from controller import DistanceSensor
#from controller import GPS
from math import sin, cos, sqrt, atan2, asin, pi

# GLOBAL VARIABLES  

MAX_SPEED = 9.53 # rad/s

L = 9.35

PROX_THRESHOLD = 2000

GROUND_THRESHOLD = 700

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# ************ INITIALIZATION ************ #

# MOTOR
left_motor = robot.getDevice('motor.left')
right_motor = robot.getDevice('motor.right')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# GROUND
ground_sensors = []
ground_names = [
        'prox.ground.0', #left
        'prox.ground.1' #right
]

for name in ground_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ground_sensors.append(sensor)

# PROXIMITY SENSORS
prox_sensors = []
prox_names = [
    'prox.horizontal.0', #left
    'prox.horizontal.1', #left
    'prox.horizontal.2', #fw
    'prox.horizontal.3', #right
    'prox.horizontal.4'  #right
]
for name in prox_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    prox_sensors.append(sensor)

# ACCELEROMETER
accelerometer = robot.getDevice('acc')
accelerometer.enable(timestep)

# TEMPERATURE (SIMULATED)
#gps = robot.getDevice('gps')
#gps.enable(timestep)

# ************ PROXY METHODS ************ #

def set_velocity(vl, vr):
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)
        
def read_proximity_sensor(index):
    return prox_sensors[index].getValue()
    
def read_accelerometer(index):
    return accelerometer.getValues()[index]

# only delta    
def read_ground_sensor(index):
    return ground_sensors[index].getValue()

# TODO    
#def read_temperature():
#    return gps.getValues()

# ************ VECTOR LIBRARY ************ #
def from_polar_to_cartesian(w, v):
    x = v * cos(w)
    y = v * sin(w)
    return [x, y]

def from_cart_to_polar(vec):
    v = sqrt((vec[0] * vec[0] + vec[1] * vec[1])) 
    w = atan2(vec[1], vec[0])
    return [w, v]

    
def cart_sum(v1, v2):
    vx = v1[0] + v2[0]
    vy = v1[1] + v2[1]
    return [vx, vy]

def polar_sum(v1, v2):
    if v1[1] == 0:
        return v2
    if v2[1] == 0:
        return v1
    v1_cart = from_polar_to_cartesian(v1[0], v1[1])
    v2_cart = from_polar_to_cartesian(v2[0], v2[1])
    sum = cart_sum(v1_cart, v2_cart)
    return from_cart_to_polar(sum)

def from_vector_to_differential(w, v):
    vl = v - w * L/2
    vr = v + w * L/2
    return [vl, vr]
    
def limit_speed(v):
    if v > MAX_SPEED:
        return MAX_SPEED
    if v < -MAX_SPEED:
         return -MAX_SPEED
    return v
    
# BASE BEHAVIOUR (FOR NOW FULL SPEED AHEAD)
def base_behaviour():
    return [0, MAX_SPEED]
    
# AVOID FALLING
def avoid_falling():
    left = read_ground_sensor(0)
    right = read_ground_sensor(1)
    
    if left < GROUND_THRESHOLD or right < GROUND_THRESHOLD:
        return [-pi, MAX_SPEED]      
    return [0, 0]
    
# AVOID FAST CRASHES
def avoid_fast_crashes():
    max = [-1, 0] # index, value
    for i in range(4):
        d = read_proximity_sensor(i)
        if d > PROX_THRESHOLD and d > max[1]:
            max = [i, d]
    if max[0] == -1:
        return [0, 0]
    else:
        return [-pi, MAX_SPEED]
  
# TODO        
#def avoid_inclinations():
#    v = read_accelerometer(0)
#    return [0, 0]
    
while robot.step(timestep) != -1:
  
  fields = [base_behaviour(), avoid_falling(), avoid_fast_crashes()]

  sum_v = [0, 0]
  for f in fields:
      sum_v = polar_sum(sum_v, f)

  move = from_vector_to_differential(sum_v[0], sum_v[1])
  vl = limit_speed(move[0])
  vr = limit_speed(move[1])
  #print(vl, vr)
  set_velocity(vl, vr)
