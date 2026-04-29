# Elman Recurrent Neural Network
import numpy as np
import math
from copy import deepcopy

enn = {}
seed = 0
weight = 2

# Set seed used to create and update the RNN
def set_seed(s):
    global seed
    seed = s
    np.random.seed(seed)


# Get seed used to create and update the RNN
def get_seed():
    return seed

# Create a feedforward neural network. All the weights are in [-weight, weight], 
# except the output one, that will be adapted runtime.
def create(incount, hidcount, outcount):

    # Create the weights matrix from input to hidden nodes and randomly set the weights in [-weight, weight];
    # consider also the bias weights
    l1 = np.random.randint(-weight * 100, weight * 100, (hidcount, incount + 1)) / 100

    #-- Create the weights matrix from hidden to output nodes and set the weights to zero;
    # consider also the bias weights
    l2 = np.zeros((outcount, hidcount + 1))

    return { "l1" : l1, "l2" : l2 } 
     
# Compute the output according to the input
def compute(enn, inputs):

    # Set the input and bias values in a column matrix
    ist = np.asmatrix(np.append(inputs, [1])).T

    # Compute the value of the hidden nodes according to the inputs and bias, 
    # and then add the value of context nodes
    hs = np.matmul(enn["l1"], ist)

    # Apply the activation function on the hidden nodes
    hs = np.tanh(hs)
    # os = 1 / (1 + np.exp(-os))

    # Compute the value of the output nodes according to
    # the hidden nodes and bias
    hs = np.vstack((hs, [1]))
    os = np.matmul(enn["l2"], hs)

    # Apply the activation function on the hidden nodes
    os = np.tanh(os)
    # os = 1 / (1 + np.exp(-os))

    return enn, np.asarray(os).flatten()

# Random change the weights of the neural network, but keep them in [-weight, weight]
def change(enn, p, perf):
    
    # Multiplier to increase intensity of perturbation
    # with decrease of performance
    # mult = math.exp(-5 * perf)

    changes = np.random.choice(a=[False, True], size=enn["l1"].shape, p=[1 - p, p])
    enn["l1"] = enn["l1"] + changes * np.random.normal(size=enn["l1"].shape)
    enn["l1"] = np.clip(enn["l1"], -weight, weight)

    changes = np.random.choice(a=[False, True], size=enn["l2"].shape, p=[1 - p, p])
    enn["l2"] = enn["l2"] + changes * np.random.normal(size=enn["l2"].shape)
    enn["l2"] = np.clip(enn["l2"], -weight, weight)


# Copy the weights
def copy(enn):
      return {
         "l1" : deepcopy(enn["l1"]),
         "l2" : deepcopy(enn["l2"])
      }


# Print RNN
def printEnn(enn):
    print("Network:")
    print("l1")
    print(enn["l1"])
    print("l2")
    print(enn["l2"])
