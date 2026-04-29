from controller import Robot
from controller import Accelerometer
from controller import DistanceSensor
from controller import GPS
from math import sin, cos, sqrt, atan2, asin, pi
from evaluator import Evaluator
import enn

# ************ GLOBAL VARIABLES ************ #

MAX_SPEED = 9.53 # rad/s

# THRESHOLDS

PROX_THRESHOLD = 2000

GROUND_THRESHOLD = 600

OVERHEATING_THRESHOLD = 1000 # ms

LOWEST_TEMPERATURE = -40

HIGHEST_TEMPERATURE = 70

TILT_THRESHOLD = 0.15 

TILT_COUNTER = [0] 

# COOL DOWN

SLOW_DOWN_TIME = 500 # ms

STEP_COUNTER = [0]

SLOW_DOWN_COUNTER = [0]

SLOW_DOWN = [0]

# FEEDFORWARD NEURAL NETWORK

SEED = 1 # Seed of the experiment for reproducibility
MUTATION_PROBABILITY = 0.2

EPOCH_STEPS = 600 # ~30s: 600 * 50ms

# ************ INITIALIZATION ************ #

robot = Robot()
timestep = int(robot.getBasicTimeStep())

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

# FEEDFORWARD NEURAL NETWORK 
NUM_INPUTS = len(prox_sensors)
MAX_PROX = prox_sensors[0].getMaxValue()

# Create NN with an input for each proximity sensor and 2 outputs
enn.set_seed(SEED) 
curr_ann = enn.create(NUM_INPUTS, 4, 2) 

# Initialize the best mapping and state of the NN
best_ann = enn.copy(curr_ann)
best_prf = 0

steps_count = 0

# Start the first evaluation epoch
evaluator = Evaluator()
evaluator.new_epoch(robot)

# VELOCITIES
# vl = 0
# vr = 0

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
    if (vl < MAX_SPEED or vr < MAX_SPEED) and not SLOW_DOWN[0]:
      STEP_COUNTER[0] = 0
    else:
        STEP_COUNTER[0] = STEP_COUNTER[0] + 1
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
    
# AVOID TILTS        
def avoid_tilts():
    ax = read_accelerometer(0)
    ay = read_accelerometer(1)
    az = read_accelerometer(2)
    
    v = sqrt(ax*ax + ay*ay + az*az)
    
    pitch = asin(ax/v)
    roll = atan2(ay, az)
    
    if abs(pitch) > TILT_THRESHOLD or abs(roll) > TILT_THRESHOLD:
        TILT_COUNTER[0] = TILT_COUNTER[0] + 1
        if TILT_COUNTER[0] >= 5: # the counter is needed to be sure there is a tilt
            return [-pi, MAX_SPEED]            
    else:
        # reset counter 
        TILT_COUNTER[0] = 0
    return [0, 0]
    
while robot.step(timestep) != -1:
  
    # Log the experiment seed and start phase one
    if steps_count == 0:
        print("seed : ", SEED)

    # Evaluation and adaptation

    # Increment the step counter
    steps_count = steps_count + 1
        
    # Update the robot performance according to the latter step
    inputs = [read_proximity_sensor(i) / MAX_PROX for i in range(NUM_INPUTS)]

    # End of epoch: log results and check if current evaluation is equal to or
    # better than previous one
    if steps_count % EPOCH_STEPS == 0: 
        # Get the robot performance since the start of the epoch
        prf = evaluator.performance

        #file_out.write("- current-ann: \t\t")
        # enn.printEnn(curr_ann)
        print('* performance: \t\t', prf)

        # Every odd epoch we starts a re-evaluation of the best configuration;
        # Every even epoch we search for better configuration
        exploratory_epoch = (steps_count / EPOCH_STEPS) % 2 == 0

        # If we are starting an exploration, it means we just re-evaluated
        # the best configuration: update its performance;
        # Alternatively, if we found a better configuration during the 
        # exploration, set it as the new best.
        if exploratory_epoch:
            best_prf = 0.75 * best_prf + 0.25 * prf
        elif prf > best_prf:
            best_ann = enn.copy(curr_ann)
            best_prf = prf

        # Set the best coupling as the starting one
        curr_ann = enn.copy(best_ann)

        # If we are starting an exploratory epoch, modify the best coupling
        if exploratory_epoch:
            enn.change(curr_ann, MUTATION_PROBABILITY, best_prf)

        # Start a new evaluation epoch
        evaluator.new_epoch(robot)
    # Start the first evaluation epoch
    curr_ann, outputs = enn.compute(curr_ann, inputs)

    #fields = [base_behaviour(), avoid_falling(), avoid_tilts(), avoid_fast_crashes(), avoid_overheating_motors(), avoid_extreme_temperature()]

    #fields = [[*(outputs * MAX_SPEED)], avoid_falling(), avoid_tilts(), avoid_fast_crashes(), avoid_overheating_motors(), avoid_extreme_temperature()]
    
    # fields = [avoid_falling(), avoid_tilts(), avoid_fast_crashes(), avoid_overheating_motors(), avoid_extreme_temperature()]
    fields = [avoid_falling()]
    sum_v = [0, 0]
    for f in fields:
        sum_v = polar_sum(sum_v, f)
    
    if(sum_v == [0, 0]):
        set_velocity(*(outputs * MAX_SPEED))
    else:
        move = from_vector_to_differential(sum_v[0], sum_v[1])
        vl = limit_speed(move[0])
        vr = limit_speed(move[1])
        set_velocity(vl, vr) 

    # real speed or nn output?
    evaluator.update(inputs, left_motor.getVelocity(), right_motor.getVelocity())
    #evaluator.update(inputs, vl, vr)

  