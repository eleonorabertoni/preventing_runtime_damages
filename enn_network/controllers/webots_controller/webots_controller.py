from controller import Robot
from controller import Accelerometer
from controller import DistanceSensor
from controller import GPS
from math import sin, cos, sqrt, atan2, asin, pi
from evaluator import Evaluator
import enn
# GLOBAL VARIABLES  

MAX_SPEED = 9.53 # rad/s

PROX_THRESHOLD = 2000

GROUND_THRESHOLD = 600

OVERHEATING_THRESHOLD = 1000 # ms

TILT_THRESHOLD = 0.15 

TILT_COUNTER = [0]

SLOW_DOWN_TIME = 500 # ms

STEP_COUNTER = [0]

SLOW_DOWN_COUNTER = [0]

SLOW_DOWN = [0]

LOWEST_TEMPERATURE = -40

HIGHEST_TEMPERATURE = 70

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# ************ ENV FOR ENN ************ #

# Seed of the experiment for reproducibility
SEED = 1
MUTATION_PROBABILITY = 0.2

EPOCH_STEPS = 200
#PHASE_1_EPOCHS = 720
#PHASE_THRESHOLD = PHASE_1_EPOCHS * EPOCH_STEPS

steps_count = 0

#season = ££ SEASON ££

curr_ann = {}
best_ann = {}
best_prf = 0

# ************ INITIALIZATION ************ #

# MOTOR
left_motor = robot.getDevice('motor.left')
right_motor = robot.getDevice('motor.right')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# PROXIMITY SENSORS
prox_sensors = []
prox_names = [
    'prox.horizontal.0', #left
    'prox.horizontal.1', #left
    'prox.horizontal.2', #fw
    'prox.horizontal.3', #right
    'prox.horizontal.4',  #right 
]

for name in prox_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    prox_sensors.append(sensor)
    
gps = robot.getDevice('gps')
gps.enable(timestep)

# ************ PROXY METHODS ************ #

def set_velocity(vl, vr):
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)
        
def read_proximity_sensor(index):
    return prox_sensors[index].getValue()
    
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
    
# ************ INIT NETWORK ************ #
NUM_INPUTS = len(prox_sensors)

enn.set_seed(SEED)

# Create ENN with an input for each proximity sensor and 2 outputs
curr_ann = enn.create(NUM_INPUTS, 8, 2)

# Initialize the best mapping and state of the ann
best_ann = enn.copy(curr_ann)
best_prf = 0

steps_count = 0

# Start the first evaluation epoch
evaluator = Evaluator()
evaluator.new_epoch(robot)
vl = 0
vr = 0
MAX_PROX = prox_sensors[0].getMaxValue()
#file_out = open("output.txt", "w")
while robot.step(timestep) != -1:
    
    # Phase-logs and damage set up

    # Log the experiment seed and start phase one
    if steps_count == 0:
        print("seed : ", SEED)
        print("PHASE 1")

    # At half experiment switch the season
    #if steps_count == PHASE_THRESHOLD:
    #    print('\n# PHASE 2')
    #    season = 1 - season

    # Evaluation and adaptation

    # Increment the step counter
    steps_count = steps_count + 1
        
    # Update the robot performance according to the latter step
    inputs = [read_proximity_sensor(i) / MAX_PROX for i in range(NUM_INPUTS)]
    evaluator.update(inputs, vl, vr)

    # End of epoch: log results and check if current evaluation is equal to or
    # better than previous one
    if steps_count % EPOCH_STEPS == 0: 
        # Get the robot performance since the start of the epoch
        prf = evaluator.performance

        #file_out.write("- current-ann: \t\t")
        enn.printEnn(curr_ann)
        print('* performance: \t\t', prf)

        # Every odd epoch we starts a re-evaluation of the best configuration;
        # Every even epoch we search for better configuration
        exploratory_epoch = (steps_count / EPOCH_STEPS) % 2 == 0

        # If we are starting an exploration, it means we just re-evaluated
        # the best configuration: update its performance;
        # Alternatively, if we found a better configuration during the 
        # exploration, set it as the new best.
        if exploratory_epoch:
            best_prf = 0.5 * best_prf + 0.5 * prf
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
    vl = limit_speed(outputs[0][0] * MAX_SPEED)
    vr = limit_speed(outputs[1][0] * MAX_SPEED)
    set_velocity(vl, vr)
#file_out.close()