#!/usr/bin/env python


# This script tests if the self-created inverted pendulum can be controled by joint velocity controller


from __future__ import print_function

import math
import random
import time

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import numpy as np
import control as ct


MASS_CART = 20;
MASS_POLE = 2;
LENGTH_POLE  = 0.5;
INERTIA_POLE = MASS_POLE*LENGTH_POLE*LENGTH_POLE/3;
INERTIA_POLE = 0.05;

g = 9.8;

Denominator = MASS_CART*MASS_POLE*LENGTH_POLE*LENGTH_POLE + 4*MASS_CART*INERTIA_POLE + 4*MASS_POLE*INERTIA_POLE;

M = MASS_CART;
m = MASS_POLE;
I = INERTIA_POLE;
l = LENGTH_POLE;

A = np.matrix([[0,1,0,0],
       [0, 0, 5, 0],
       [0, 0, 0, 1],
       [0, 0, 5, 0]
       ])
A[1,2] = -(m**2) * (l**2) * g/Denominator;
A[3,2] = 2*l*m*(g*M+g*m)/Denominator;
print(-(m**2) * (l**2) * g/Denominator)
print(2*l*m*(g*M+g*m)/Denominator)
print(A)       
B = np.matrix([0, (4*I+M*l*l)/Denominator, 0, -2*l*m/Denominator]).T

Q = np.diag([1, 1, 1, 1])
R = np.diag([0.01])

K, S, E = ct.lqr(A, B, Q, R)

class Testbed(object):
    """ Testbed, for the pupose of testing cart-pole system """
    def __init__(self):
        # init topics and services
        self.state_subscriber = rospy.Subscriber('/invpend/joint_states', JointState, self.jsCB)
        self.velocity_publisher = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
        
        # init parameters
        self.reset_dur = 1 # reset duration, sec
        self.freq = 50 # topics pub and sub frequency, Hz
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.goal_x = 1;
        self.goal_th = 0;

    def jsCB(self, data):
        rospy.loginfo("~~~Getting Inverted pendulum joint states~~~")
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]
        
        
    def lqr_control_loop(self):        
        rate = rospy.Rate(50)
        Xdesired = np.matrix([[1],[0],[0],[0]]);
        #X_current = np.matrix([[self.pos_cart], [self.vel_cart], [self.pos_pole], [self.vel_pole]]);
        while not rospy.is_shutdown():
            X_current = np.matrix([[self.pos_cart], [self.vel_cart], [self.pos_pole], [self.vel_pole]]);
            print("X_current :",X_current);
            error = Xdesired - X_current;
            U = np.matmul(K, (error))
            cmd_vel = float(U)
            self.velocity_publisher.publish(cmd_vel)
            rate.sleep()

          
        
def main():
    """ Perform testing actions provided by Testbed class
    """
    print("Initializing node... ")
    rospy.init_node('cart_wobble')
    cart = Testbed()
    #rospy.on_shutdown(cart.clean_shutdown)
    cart.lqr_control_loop()
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
    
    
