# Elman Recurrent Neural Network
import numpy as np
import random
import math
from copy import deepcopy

enn = {}
seed = random.random()

# Set seed used to create and update the RNN
def set_seed(s):
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
    # Initialize context units state with 1s
    cs = np.ones((hidcount, 1))
    #print(cs.shape)

    # Create the weights matrix from input to hidden nodes and randomly set the weights in [-1, 1];
    # consider also the bias weights
    l1 = np.random.randint(-100, 100, (hidcount, incount + 1))
    l1 = l1 / 100
    #print(l1.shape)

    # Create the weights matrix from context to hidden nodes and randomly set the weights in [-1, 1]
    lc = np.random.randint(-100, 100, (hidcount, 1))
    lc = lc / 100
    #print(lc.shape)

    #-- Create the weights matrix from hidden to output nodes and set the weights to zero;
    # consider also the bias weights
    l2 = np.zeros((outcount, hidcount + 1))
    #print(l2.shape)

    return { "l1" : l1, "lc" : lc, "cs" : cs, "l2" : l2 }

# Sigmoid function: compute the sigmoid value of an input
# value x. Useful as ANNs activation function.
def sigmoid(x):
     return 1 / (1 + np.exp(-x))
     
# Compute the output according to the input
def compute(enn, inputs):

      # Set the input and bias values in a column matrix
      ist = np.array(inputs)
      ist = np.append(inputs, [1])
      print("ist", ist.shape)
      ist = np.reshape(ist, (-1, 1))
      print("ist", ist.shape)

      # Compute the value of the hidden nodes according to the inputs and bias, 
      # and then add the value of context nodes
      m1 = np.matmul(enn["l1"], ist)
      #m2 = np.matmul(enn["lc"].T, enn["cs"])
      m2 = np.matmul(enn["lc"], enn["cs"])
      print("m1", m1.shape)
      print("m2", m2.shape)
      hs = np.add(m1, m2)
      print("hs", hs.shape)

      # Apply the activation function on the hidden nodes
      for i in range(len(hs[0])):
        hs[0][i] = sigmoid(hs[0][i])
      
      # Copy the value of the hidden nodes to the context
      enn["cs"] = hs

      # Compute the value of the output nodes according to
      # the hidden nodes and bias
      hs = np.append(hs, [1])
      hs = np.reshape(hs, (-1, 1))
      print("hs", hs.shape)
      print("enn", enn["l2"].shape)
      os = np.matmul(enn["l2"], hs) 
      print("os", os.shape)

      # Apply the activation function on the hidden nodes
      for i in range(len(os[0])):
           os[0][i] = sigmoid(os[0][i])
      return enn, os


# Random change the weights of the RNN, but keep them in
# [-1, 1]
def change(enn, p, perf):
      # Multiplier to increase intensity of perturbation
      # with decrease of performance
      mult = math.exp(-5 * perf)
      dev = 0.75

      for i in range(len(enn["l1"])):
         for j in range(len(enn["l1"][0])):
             if random.random() <= p:
                 enn["l1"][i][j] = enn["l1"][i][j] + mult * random.randint(-dev * 100, dev * 100) / 100.0
                 enn["l1"][i][j] = max(-1, enn["l1"][i][j])
                 enn["l1"][i][j] = min( 1, enn["l1"][i][j])

      for i in range(len(enn["lc"])):
         for j in range(len(enn["lc"][0])):
             if random.random() <= p:
                 enn["lc"][i][j] = enn["lc"][i][j] + mult * random.randint(-dev * 100, dev * 100) / 100.0
                 enn["lc"][i][j] = max(-1, enn["lc"][i][j])
                 enn["lc"][i][j] = min(1, enn["lc"][i][j])

      for i in range(len(enn["l2"])):
         for j in range(len(enn["l2"][0])):
             if random.random() <= p:
                 enn["l2"][i][j] = enn["l2"][i][j] + mult * random.randint(-dev * 100, dev * 100) / 100.0
                 enn["l2"][i][j] = max(-1, enn["l2"][i][j])
                 enn["l2"][i][j] = min( 1, enn["l2"][i][j])

# Copy the RNN weights
def copy(enn):
      return {
         "l1" : deepcopy(enn["l1"]),
         "lc" : deepcopy(enn["lc"]),
         "cs" : deepcopy(enn["cs"]),
         "l2" : deepcopy(enn["l2"])
      }


# Print RNN
def printEnn(enn):
     print("Network:")
     print("l1")
     print(enn["l1"])
     print("lc")
     print(enn["lc"].T)
     print("cs")
     print(enn["cs"].T)
     print("l2")
     print(enn["l2"])