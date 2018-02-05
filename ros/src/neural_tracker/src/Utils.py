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

import tensorflow as tf
import numpy as np
from meta_planner_msgs.msg import *
from geometry_msgs.msg import Vector3

def lrelu(x):
  return tf.nn.relu(x) - 0.01*tf.nn.relu(-x)

def Normalize(ALL_x):
    pos = ALL_x[:,[0,1,2]]/5.0;
    vel = ALL_x[:,[3,4,5]]/10.0;
    ret_val = np.concatenate((pos,vel),axis=1)
    return ret_val

def TransDef(scope=None, reuse=None, lsizes = None):
    with tf.variable_scope(scope, reuse=reuse):
        states = tf.placeholder(tf.float32,shape=(None,lsizes[0]),name="states");
        y = tf.placeholder(tf.float32,shape=(None,lsizes[-1]),name="y");

        lw = [];
        lb = [];
        l = [];
        reg = 0.0;
        for i in xrange(len(lsizes) - 1):
            lw.append(0.1*tf.Variable(tf.random_uniform([lsizes[i],lsizes[i + 1]],-1.0,1.0,dtype=tf.float32),name="H"+str(i)));
            lb.append(0.1*tf.Variable(tf.random_uniform([1,lsizes[i + 1]],-1.0,1.0,dtype=tf.float32),name="B"+str(i)));
            reg = reg + tf.reduce_sum(tf.abs(lw[-1])) + tf.reduce_sum(tf.abs(lb[-1]));

        l.append(lrelu(tf.add(tf.matmul(states,lw[0]), lb[0])))
        for i in xrange(len(lw)-2):
            l.append(lrelu(tf.add(tf.matmul(l[-1],lw[i+1]), lb[i+1])));

        last_ba = tf.add(tf.matmul(l[-1],lw[-1]), lb[-1],name="A_end");
        l.append(tf.nn.softmax(last_ba));
        cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits=last_ba,labels=y)
        L = tf.reduce_mean(cross_entropy)

        PI = l[-1];

    return states,y,PI,L,l,lb,reg,cross_entropy

def PackState(state):
    msg = meta_planner_msgs.msg.State()
    msg.state = list(state)
    msg.dimension = len(state)
    return msg

def UnpackState(msg):
    state = np.array([msg.state])
    return state
#    return state.reshape((state.shape[1],))

def PackControl(control):
    msg = meta_planner_msgs.msg.Control()
    msg.control = list(control[0])

    # Transpose pitch and roll.
#    pitch = msg.control[1]
#    msg.control[1] = msg.control[0]
#    msg.control[0] = pitch

    # Flip roll.
    msg.control[1] = -msg.control[1]

    msg.dimension = len(msg.control)
    return msg

def UnpackControl(msg):
    control = np.array(msg.control)
    return control

def PackPoint(point):
    msg = geometry_msgs.msg.Vector3()
    msg.x = point[0]
    msg.y = point[1]
    msg.z = point[2]
    return msg

def UnpackPoint(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    point = np.array([x,y,z])
    return point
