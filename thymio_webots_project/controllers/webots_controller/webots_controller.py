from controller import Robot
from controller import Accelerometer
from controller import DistanceSensor
from controller import GPS
from math import sin, cos, sqrt, atan2, asin, pi

# GLOBAL VARIABLES  

MAX_SPEED = 9.53 # rad/s

PROX_THRESHOLD = 2000

GROUND_THRESHOLD = 700

OVERHEATING_THRESHOLD = 1000 # ms

SLOW_DOWN_TIME = 500 # ms

STEP_COUNTER = [0]

SLOW_DOWN_COUNTER = [0]

SLOW_DOWN = [0]

LOWEST_TEMPERATURE = -40

HIGHEST_TEMPERATURE = 70

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
gps = robot.getDevice('gps')
gps.enable(timestep)

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

# simulated temperature readings    
def read_temperature():
    RANGE = 0.3
    temperature = 20
    v = gps.getValues()
    x = v[0]
    y = v[1]
    if x < -RANGE and y < -RANGE:
        temperature = LOWEST_TEMPERATURE
    if x > RANGE and y > RANGE:
        temperature = HIGHEST_TEMPERATURE
    return temperature    
    
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
    if w == 0:
         return [v, v]
    vr = v * sin(w + pi/4)
    vl = v * cos(w + pi/4)
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
    for i in range(5):
        d = read_proximity_sensor(i)
        if d > PROX_THRESHOLD:
            return [-pi, MAX_SPEED/2]
    return [0, 0]

# AVOID FULL SPEED FOR TOO LONG
def avoid_overheating_motors():
    if SLOW_DOWN[0]:
        if STEP_COUNTER[0] * timestep <= SLOW_DOWN_TIME:
            return [-pi, MAX_SPEED/3]
        else:
            SLOW_DOWN[0] = 0
            STEP_COUNTER[0] = 0
            return [0, 0]
    elif STEP_COUNTER[0] * timestep >= OVERHEATING_THRESHOLD:
        STEP_COUNTER[0] = 0
        SLOW_DOWN[0] = 1
        return [-pi, MAX_SPEED/3]  
    else:
        return [0, 0]
        
# AVOID EXTREME TEMPERATURE
def avoid_extreme_temperature():
    temperature = read_temperature()
    if temperature <= LOWEST_TEMPERATURE or temperature >= HIGHEST_TEMPERATURE:
        return [-pi/2, MAX_SPEED]
    return [0, 0]
    
# TODO        
#def avoid_inclinations():
#    v = read_accelerometer(0)
#    return [0, 0]
 
while robot.step(timestep) != -1:
  STEP_COUNTER[0] = STEP_COUNTER[0] + 1
  fields = [base_behaviour(), avoid_falling(), avoid_fast_crashes(), avoid_overheating_motors(), avoid_extreme_temperature()]
  sum_v = [0, 0]
  for f in fields:
      sum_v = polar_sum(sum_v, f)
  
  move = from_vector_to_differential(sum_v[0], sum_v[1])
  vl = limit_speed(move[0])
  vr = limit_speed(move[1])
  if (vl < MAX_SPEED or vr < MAX_SPEED) and not SLOW_DOWN[0]:
      STEP_COUNTER[0] = 0
  #print(vl, vr)
  set_velocity(vl, vr)