#!/usr/bin/env python3

import numpy as np 
import math as mt
# import cvxpy as cp

from low_pass_filter import first_order_iir_param, first_order_iir

"""
Methods for nonlinear control and control allocation 
- control to thruster force 
- thruster force to PWM times in milli sec.
- control allocation methods:
    - psuedo inverse 
    - feasible convex optimization 
"""

def filtered_state_error(state_desired,state_current,state_desired_old,state_current_old,\
    alpha_position,alpha_attitude):

    position_desired = state_desired[0:2]
    velocity_desired = state_desired[3:5]
    velocity_desired_old = state_desired_old[3:5]

    theta_desired = state_desired[2]
    thetad_desired = state_desired[5]
    thetad_desired_old = state_desired_old[5]

    position_current = state_current[0:2]
    velocity_current = state_current[3:5]
    velocity_current_old = state_current_old[3:5]

    theta_current = state_current[2]
    thetad_current = state_current[5]
    thetad_current_old = state_current_old[5]
      
    
    position_error = position_current - position_desired
    velocity_error = velocity_current - velocity_desired
    velocity_error_old = velocity_current_old - velocity_desired_old
    
    # filter velocity error 
    velocity_error = first_order_iir(velocity_error,velocity_error_old,alpha_position)
    
    theta_error = theta_current - theta_desired
    thetad_error = thetad_current - thetad_desired
    thetad_error_old = thetad_current_old - thetad_desired_old

    # filter angular velocity error 

    thetad_error = first_order_iir(thetad_error,thetad_error_old,alpha_attitude)

    return position_error, velocity_error, theta_error, thetad_error


def s2ms(t):
    return t*1000


def controller(position_error, velocity_error, theta_error, thetad_error,\
     gain_position, gain_attitude, position_control_frequency, attitude_control_frequency):

    control_force = np.zeros(3)

    control_force[0:2] = -gain_position['Kp']*position_error - gain_position['Kd']*velocity_error*position_control_frequency
    control_force[2] = -gain_attitude['Kp']*theta_error - gain_attitude['Kd']*thetad_error*attitude_control_frequency

    return control_force


def rotate_bodyframe_to_inertialframe(theta):
    return np.array([[mt.cos(theta),mt.sin(theta),0],[-mt.sin(theta),mt.cos(theta),0],[0,0,1]]) 


def control_allocation(theta,control_force_inertial,system_param,\
    thruster_force_desired = np.zeros(8),cvx=False,pseudoinv = True):
    
    # thruster_force_desired is computed from the planning algorithm
    #     
    ## Control Allocation Matrix
    H = np.array([[-1,-1,0,0,1,1,0,0],[0,0,-1,-1,0,0,1,1],[-1,1,-1,1,-1,1,-1,1]])
    control = control_force_inertial.reshape((3,1))
    
    l = system_param['moment_arm']
    physical_parameter = np.array([[system_param['mass'],0,0],[0,system_param['mass'],0],[0,0,system_param['inertia']/l]])


    #psuedo inverse based control allocation
    if pseudoinv == True:
        # psuedo inverse based method for control allocation 
        inv_allocation = physical_parameter@control
        inv_force = np.linalg.pinv(H)
        rotate_inertialframe_to_bodyframe = rotate_bodyframe_to_inertialframe(theta).T
        thruster_force = thruster_force_desired.reshape((8,1)) + (inv_force@rotate_inertialframe_to_bodyframe@inv_allocation) # thruster force 

    # if cvx == True: 
    #     # feasible convex optimization based method for control allocation

    #     beta = 1e+8
    #     F = cp.Variable(int(8))
    #     delta = cp.Variable(1)
    #     cost = cp.norm1(F) + beta* delta
    #     constraint = [cp.norm1()]
    return thruster_force.reshape((8))

def get_slope(x1, x2, y1, y2):
    return (y2 - y1)/(x2 - x1)



def thruster_force_saturation(thruster_force,thrust_max,thrust_min = 0):

    for i in range(8):
        if thruster_force[i] <= 0:
            thruster_force[i] = 0

        elif thruster_force[i] >0 and thruster_force[i] <= thrust_min:
            thruster_force[i] = thrust_min

        elif thruster_force[i] >  thrust_min and thruster_force[i] <= thrust_max:
            thruster_force[i] = thruster_force[i]

        else:
            thruster_force[i] = thrust_max

    return thruster_force


def impulse_bit_filter(thruster_force_times,max_impulse_bit, min_impulse_bit):

    # Note max_impulse_bit = 0.75/control_frequency

    thrust_time_msec = np.zeros(8)

    for i in range(8):
        if (thruster_force_times[i] > max_impulse_bit):
            thrust_time_msec[i] = s2ms(max_impulse_bit)
        else:
            if (thruster_force_times[i] < min_impulse_bit):
                thrust_time_msec[i] = 0
            else:
                thrust_time_msec[i] = s2ms(thruster_force_times)


    return thrust_time_msec


def thruster_model_force_to_times(control_frequency,a,b):

    def thruster_force_to_times(thruster_force):
        thruster_force_times = np.zeros(8)
        for i in range(8):
            thruster_force_times[i] = a*(thruster_force[i]/control_frequency) + b
        return thruster_force_times
    return thruster_force_to_times


def thruster_linear_force_to_times(thruster_force,control_frequency,thrust_min,\
     thrust_max,min_impulse_bit,max_impulse_bit):

    # used in demos pospos--2018--version.

    # thrust is in Newtons
    # impulse_bit in sec to be converted to millisec

    slope = get_slope(thrust_min,thrust_max,min_impulse_bit,max_impulse_bit)
    intercept = -slope*thrust_min + min_impulse_bit

    thruster_time_ms = np.zeros(8)

    for i in range(8):
        if (thruster_force[i] > thrust_max):
            thruster_time_ms[i] = s2ms(max_impulse_bit)
        else:
            if (thruster_force[i] < thrust_min):
                thruster_time_ms[i] = 0
            else:
                thruster_time_ms[i] = s2ms(slope*thruster_force[i] + intercept)

    return thruster_time_ms


def thuster_model(manifold_pressure):
    if manifold_pressure <=40:
        a = 7.863
        b = -0.009727
        thrust_max = 0.25
    elif manifold_pressure>40 and manifold_pressure<=50:
        a = 4.829 
        b = -0.007686
        thrust_max = 0.3
    else:
        a = 3.51
        b = -0.006035
        thrust_max = 0.35
    
    return a, b, thrust_max

