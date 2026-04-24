import math
class Evaluator:
    def __init__(self):
            self.performance = 0


    # Initialize a new evaluation period
    def new_epoch(self, robot):
        print("new epoch")
        #gps = robot.getDevice('gps')
        #print(" position [x,y,z]: ", gps.getValues()) 
        self.performance = 0

    # Evaluates the performance of the controller as the proximity from obstacles, direction, speed, penalize the robot when going backward
    def update(self, inputs, vleft, vright):
        MAX_SPEED = 9.53

        vleft = max(0, vleft) / MAX_SPEED
        vright = max(0, vright) / MAX_SPEED

        V = (abs(vleft) + abs(vright)) / 2
        dV = abs(vleft - vright)
        max_value = max(inputs)

        self.performance = V * (1 - math.sqrt(dV)) * (1 - max_value)
