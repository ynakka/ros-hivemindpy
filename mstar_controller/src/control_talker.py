#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
# import Constants
# import serial
import math 
import time 

from std_msgs.msg import Bool, String, Float32, Float64

from mstar_controller.msg import Thrusters8 as Thrusters8
from mstar_controller.msg import State6 as State6  # Change to navigation msg 

from geometry_msgs.msg import TransformStamped as TransformStamped

import controller_3dof as cntrl



class control_talker:

	def __init__(self,gain_position,gain_attitude,system_param,thruster_param,control_param,spacecraft_name):

		# dummy variables 

		self.state_old = np.zeros(6)
		self.state = np.zeros(6)

		self.control_old = np.zeros(8)
		self.control = np.zeros(8)


		self.system_param = system_param

		self.gain_position = control_param['gain_position']
		self.gain_attitude = control_param['gain_attitude']

		self.manifold_pressure = thruster_param['manifold_pressure']

		self.attitude_control_frequency = control_param['attitude_control_frequency']
		self.position_control_frequency = control_param['position_control_frequency']

		self.thruster = Thrusters8()

		
		return 

	def state_call_back(self,):


		return 

	def update_thruster_times(self,thruster_times_ms):
		self.thruster.FXmMZm = F[0]
        self.thruster.FXmMZp = F[1]
        self.thruster.FYmMZm = F[2]
        self.thruster.FYmMZp = F[3]
        self.thruster.FXpMZm = F[4]
        self.thruster.FXpMZp = F[5]
        self.thruster.FYpMZm = F[6]
        self.thruster.FYpMZp = F[7]





if __name__ == "__main__":
    pass










