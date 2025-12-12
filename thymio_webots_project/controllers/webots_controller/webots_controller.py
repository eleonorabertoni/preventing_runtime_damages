from controller import Robot
from controller import Accelerometer
from controller import DistanceSensor
#from controller import GPS
from math import *

# GLOBAL VARIABLES  

MAX_SPEED = 9.53

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
        'prox.ground.0', #sx
        'prox.ground.1' #dx
]

for name in ground_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ground_sensors.append(sensor)

# PROXIMITY SENSORS
prox_sensors = []
prox_names = [
    'prox.horizontal.0', #sx
    'prox.horizontal.1', #sx
    'prox.horizontal.2', #fw
    'prox.horizontal.3', #dx
    'prox.horizontal.4'  #dx
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

def set_velocity(vr, vl):
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

while robot.step(timestep) != -1: 
  # TODO
  set_velocity(MAX_SPEED, MAX_SPEED)