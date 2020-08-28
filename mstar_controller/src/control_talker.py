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

		# system parameters
		self.system_param = system_param

		self.gain_position = control_param['gain_position']
		self.gain_attitude = control_param['gain_attitude']

		self.manifold_pressure = thruster_param['manifold_pressure']

		self.attitude_control_frequency = control_param['attitude_control_frequency']
		self.position_control_frequency = control_param['position_control_frequency']

		# dummy variables 
		self.state_old = np.zeros(6)
		self.state = np.zeros(6)

		self.control_old = np.zeros(8)
		self.control = np.zeros(8)

		# messages 

		self.thruster = Thrusters8()
		self.control_guidance = Thrusters8()

		self.state_guidance = State6()
		self.state_navigation = State6()

		

		
		return 

	def guidance_call_back(self):





		return

	

	def state_nav_call_back(self,state_current,state_current_old,state_desired,state_desired_old,\
		alpha_position,alpha_attitude,system_param,attitude_control_frequency,position_control_frequency,\
		max_impulse_bit, min_impulse_bit, thrust_max, thrust_min):

		


		return 
	def update_thruster_times(self,thruster_times_ms):
		self.thruster.FXmMZm = thruster_times_ms[0]
		self.thruster.FXmMZp = thruster_times_ms[1]
		self.thruster.FYmMZm = thruster_times_ms[2]
		self.thruster.FYmMZp = thruster_times_ms[3]
		self.thruster.FXpMZm = thruster_times_ms[4]
		self.thruster.FXpMZp = thruster_times_ms[5]
		self.thruster.FYpMZm = thruster_times_ms[6]
		self.thruster.FYpMZp = thruster_times_ms[7]





if __name__ == "__main__":
    pass










