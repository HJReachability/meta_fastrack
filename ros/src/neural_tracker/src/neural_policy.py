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
import Utils
import pickle
import numpy as np
import itertools

# Picklefile format:
# {"weights":<np.array>,
#  "c_layers":<[list ints]>,
#  "d_layers":<[list ints]>
#  "control_bounds_upper":<[list of doubles]>
#  "control_bounds_lower":<[list of doubles]>
#  "tracking_error_bound":<[list of doubles]>
#  "planner_params":<{
#                     "max_speed":[list doubles],
#                     "max_vel_dist":[list doubles],
#                     "max_acc_dist":[list doubles]}>
#  "normalization_args":<[list of doubles]> (if entry is -1 it is an angle)
# }


class NeuralPolicy(object):
    def __init__(self, filename, _id, sess=None, ppick=-1, pick_=-1):
        self.sess = sess

        self.ppick = ppick
        self.ppick_ = pick_

        content = pickle.load( open(filename, "rb" ))
        controllers = content["weights"]
        PI_control = controllers[0] #set of control policies
        PI_disturb = controllers[1] #set of control disturbances

        self.c_layers = content["c_layers"]
        self.d_layers = content["d_layers"]
        self.max_list = content["control_bounds_upper"] # [0.1,0.1,11.81];
        self.min_list = content["control_bounds_lower"] # [-0.1,-0.1,7.81];
        self.norm_args = content["normalization_args"]

        planner_params = content["planner_params"]

        # HACK! Should fix this in the pickle file itself later.
        self.max_speed = [0.25 * s for s in planner_params["max_speed"]]
        self.max_vel_dist = [0.75 * s for s in planner_params["max_speed"]]

#        self.max_speed = planner_params["max_speed"]
#        self.max_vel_dist = planner_params["max_vel_dist"]
        self.max_acc_dist = planner_params["max_acc_dist"]
        self.tracking_error_bound = content["tracking_error_bound"]

        print("Layers: " + str(self.c_layers))
        print("Length of PI_CONTROL: " + str(len(PI_control)))


        # Generate set of possible actions (i.e. all possible bang-bang configurations)
        self.perms = list(itertools.product([-1,1], repeat=len(self.max_list)))
        self.true_ac_list = [];
        for i in range(len(self.perms)): #2**num_actions
            ac_tuple = self.perms[i];
            ac_list = [(tmp1==1)*tmp3 +  (tmp1==-1)*tmp2 for tmp1,tmp2,tmp3 in zip(ac_tuple,self.min_list,self.max_list)];
            self.true_ac_list.append(ac_list);

        # Load layers and create neural net computational graph
        self.states = []
        self.y = []
        self.Tt = []
        self.L = []
        self.l_r = []
        self.lb = []
        self.reg = []
        self.cross_entropy = []
        self.theta = []
        self.init = []
        for cd in ["c","d"]:
            if(cd == "c"):
                states,y,Tt,L,l_r,lb,reg,cross_entropy = Utils.TransDef(str(_id)+cd,False,self.c_layers)
            else:
                states,y,Tt,L,l_r,lb,reg,cross_entropy = Utils.TransDef(str(_id)+cd,False,self.d_layers)
            self.states.append(states)
            self.y.append(y)
            self.Tt.append(Tt)
            self.L.append(L)
            self.l_r.append(l_r)
            self.lb.append(lb)
            self.reg.append(reg)
            self.cross_entropy.append(cross_entropy)
            self.theta.append(tf.get_collection(tf.GraphKeys.VARIABLES, scope=str(_id)))
            self.init.append(tf.variables_initializer(self.theta[-1]))
            self.sess.run(self.init[-1])

        self.PI_control = PI_control[self.ppick]
        self.PI_disturb = PI_disturb[self.ppick_]

        # Load weights of the controller in index "ppick" in list self.PI_control
        for ind in range(len(self.PI_control)):
            try:
                self.sess.run(self.theta[0][ind].assign(self.PI_control[ind]))
                print("Loaded all weights at index %d of the NNController. Controller used: %d." % (ind, self.ppick))
            except IndexError:
                print("Pickable file doesn't correspond to the architecture of policy controller: " + str(self.c_layers))
        # Load weights of the disturbance
        for ind in range(len(self.PI_disturb)):
            try:
                self.sess.run(self.theta[1][ind].assign(self.PI_disturb[ind]))
                print("Loaded all weights at index %d of the NNDisturbance. Disturbance used: %d" % (ind, self.ppick_))
            except IndexError:
                print("Pickable file doesn't correspond to the architecture of disturbance controller: " + str(self.d_layers))



    def OptimalControl(self, relative_state):
        #Get probability distribution over actions.
        control = self.sess.run(self.Tt[0], { self.states[0] :
                                              Utils.Normalize(relative_state,self.norm_args) })

        #Compute the argmax of the probability distribution
        control = control.argmax(axis=1);
        #Get the corresponding bang-bang control
        control = np.asarray([self.true_ac_list[i] for i in control]);
        return control
