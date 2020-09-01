#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import sys

import math 
import time 

from control_talker import control_talker


def main():
    
    # control parameter
    system_param = {'moment_arm': 0.2, 'mass': 10.0,'inertia': 1.62}
    control_param = {'position_control_frequency':1,'attitude_control_frequency':2,
         'gain_attitude':{'Kp':1.0,'Kd':1.0},'gain_position' : {'Kp':2.0,'Kd':2.0}}

    thruster_param = {'manifold_pressure': 40}
    waypoint_error = {'position_error':0.1,'attitude_error':0.1}

    filter_param = {'alpha_position':0.1,'alpha_attitude':0.1}
    
    talker = control_talker(system_param,thruster_param,control_param,filter_param,waypoint_error)

if __name__ == "__main__":
    # read the spacecraft name from the launch file argument 
    main()

    