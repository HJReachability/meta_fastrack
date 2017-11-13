"""
Copyright (c) 2017, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Please contact the author(s) of this library if you have any questions.
Authors: Vicenc Rubies Royo     ( vrubies@eecs.berkeley.edu )
         David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
         Sylvia Herbert         ( sylvia.herbert@eecs.berkeley.edu )
         Somil Bansal           ( somil@eecs.berkeley.edu )
         Jaime Fisac            ( jfisac@eecs.berkeley.edu )
"""

################################################################################
#
# Class to provide access to a neural network optimal controller.
#
################################################################################

import tensorflow as tf
from Auxiliary import TransDef, Normalize
import pickle
import numpy as np
import itertools

class NeuralPolicy(object):
    def __init__(self, filename, params):
        # Set control parameters
        self.max_list = [0.1,0.1,11.81];
        self.min_list = [-0.1,-0.1,7.81];

        # Generate set of possible actions (i.e. all possible bang-bang configurations)
        self.perms = list(itertools.product([-1,1], repeat=len(self.max_list)))
        self.true_ac_list = [];
        for i in range(len(self.perms)): #2**num_actions
            ac_tuple = self.perms[i];
            ac_list = [(tmp1==1)*tmp3 +  (tmp1==-1)*tmp2 for tmp1,tmp2,tmp3 in zip(ac_tuple,self.min_list,self.max_list)];
            self.true_ac_list.append(ac_list);

        # Load layers and create neural net computational graph
        layers = params["layers"]
        states,y,Tt,L,l_r,lb,reg,cross_entropy = TransDef("PolicyNet",False,layers)
        self.states = states
        self.y = y
        self.Tt = Tt
        self.L = L
        self.l_r = l_r
        self.lb = lb
        self.reg = reg
        self.cross_entropy = cross_entropy

        # Initialize TF Session
        self.theta = tf.get_collection(tf.GraphKeys.VARIABLES, scope='PolicyNet')
        self.sess = tf.Session()
        self.init = tf.initialize_all_variables()
        self.sess.run(self.init)

        #Modify set of weights for the neural net policies
        self.PIs = pickle.load( open(filename, "rb" ))
        self.ALL_PI = self.PIs[0] # Pick out the policy for the controller (not disturbance).
        self.PI = self.ALL_PI[4] # Pick out the desired controller policy.
        for ind in range(len(self.PI)):
            try:
                self.sess.run(self.theta[ind].assign(self.PI[ind]))
            except IndexError:
                print("Pickable file doesn't correspond to the architecture: " + str(layers))


    def OptimalControl(self, relative_state):
        #Get porbability distribution over actions
        action = self.sess.run(self.Tt,{self.states:Normalize(relative_state)})
        #Compute the argmax of the probability distribution
        action = action.argmax(axis=1);
        #Get the corresponding bang-bang control
        action = np.asarray([self.true_ac_list[i] for i in action]);
        return action
