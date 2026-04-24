# Elman Recurrent Neural Network
import numpy as np
import random
import math
from copy import deepcopy

enn = {}
seed = random.random()
weight = 2

# Set seed used to create and update the RNN
def set_seed(s):
    global seed
    seed = s
    random.seed(seed)
    np.random.seed(seed)


# Get seed used to create and update the RNN
def get_seed():
    return seed

# Create RNN based on the Elman Network. All the weights are in [-1, 1], 
# except the output one, that will be  adapted runtime.
# The starting context nodes value is 1.
def create(incount, hidcount, outcount):
    # Initialize context units state with 0s
    # cs = np.zeros((hidcount, 1))

    # Create the weights matrix from input to hidden nodes and randomly set the weights in [-1, 1];
    # consider also the bias weights
    l1 = np.random.randint(-weight * 100, weight * 100, (hidcount, incount + 1)) / 100

    # # Create the weights matrix from context to hidden nodes and randomly set the weights in [-1, 1]
    # lc = np.random.randint(-weight * 100, weight * 100, (hidcount, 1)) / 100

    #-- Create the weights matrix from hidden to output nodes and set the weights to zero;
    # consider also the bias weights
    l2 = np.zeros((outcount, hidcount + 1))

    return { "l1" : l1, "l2" : l2 } # { "l1" : l1, "lc" : lc, "cs" : cs, "l2" : l2 }

# Sigmoid function: compute the sigmoid value of an input
# value x. Useful as ANNs activation function.
#def sigmoid(x):
#     return 1 / (1 + np.exp(-x))
     
# Compute the output according to the input
def compute(enn, inputs):

    # Set the input and bias values in a column matrix
    ist = np.asmatrix(np.append(inputs, [1])).T

    # Compute the value of the hidden nodes according to the inputs and bias, 
    # and then add the value of context nodes
    # hs = np.add(np.matmul(enn["l1"], ist), enn["lc"] * enn["cs"])
    hs = np.matmul(enn["l1"], ist)

    # Apply the activation function on the hidden nodes
    hs = np.tanh(hs)

    # Copy the value of the hidden nodes to the context
    # enn["cs"] = np.asarray(deepcopy(hs))

    # Compute the value of the output nodes according to
    # the hidden nodes and bias
    hs = np.vstack((hs, [1]))
    os = np.matmul(enn["l2"], hs)

    # Apply the activation function on the hidden nodes
    os = np.tanh(os)
    # os = 1 / (1 + np.exp(-os))

    return enn, np.asarray(os).flatten()


# Random change the weights of the RNN, but keep them in
# [-1, 1]
def change(enn, p, perf):
    # Multiplier to increase intensity of perturbation
    # with decrease of performance
    mult = math.exp(-5 * perf)

    changes = np.random.choice(a=[False, True], size=enn["l1"].shape, p=[1 - p, p])
    enn["l1"] = enn["l1"] + changes * np.random.normal(size=enn["l1"].shape)
    enn["l1"] = np.clip(enn["l1"], -weight, weight)

    # changes = np.random.choice(a=[False, True], size=enn["lc"].shape, p=[1 - p, p])
    # enn["lc"] = enn["lc"] + changes * np.random.normal(size=enn["lc"].shape)
    # enn["lc"] = np.clip(enn["lc"], -weight, weight)

    changes = np.random.choice(a=[False, True], size=enn["l2"].shape, p=[1 - p, p])
    enn["l2"] = enn["l2"] + changes * np.random.normal(size=enn["l2"].shape)
    enn["l2"] = np.clip(enn["l2"], -weight, weight)


# Copy the RNN weights
def copy(enn):
      return {
         "l1" : deepcopy(enn["l1"]),
        #  "lc" : deepcopy(enn["lc"]),
        #  "cs" : deepcopy(enn["cs"]),
         "l2" : deepcopy(enn["l2"])
      }


# Print RNN
def printEnn(enn):
    print("Network:")
    print("l1")
    print(enn["l1"])
    # print("lc")
    # print(enn["lc"].T)
    # print("cs")
    # print(enn["cs"].T)
    print("l2")
    print(enn["l2"])
