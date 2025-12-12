# GLOBAL VARIABLES
MAX_SPEED = 500

# ************ PROXY METHODS ************ #

def set_velocity(vr, vl):
    global motor_left_target, motor_right_target
    motor_left_target = vl
    motor_right_target = vr
        
def read_proximity_sensor(index):
    return prox_horizontal[index]

def read_accelerometer(index):
    return acc[index]

def read_ground_sensor(index):
    return prox_ground_delta[index]

def read_temperature():
    return temperature
    
@onevent
def prox():
   # TODO
   set_velocity(MAX_SPEED, MAX_SPEED)

