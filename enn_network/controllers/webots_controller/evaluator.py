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

    # Evaluates the performance of the controller as the proximity from obstacles, direction, speed
    def update(self, inputs, vleft, vright):
        MAX_SPEED = 9.53
        
        max_value = 0
        #max_index = -1
        for i in range(len(inputs)):
            value = inputs[i]
            if max_value < value:
                max_value = value
                #max_index = i

        V = (abs(vleft) + abs(vright)) / (2 * MAX_SPEED)
        dV = (abs(vleft - vright)) / (2 * MAX_SPEED)

        self.performance = V * (1 - math.sqrt(dV)) * (1 - max_value)