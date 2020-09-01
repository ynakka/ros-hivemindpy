#!/usr/bin/env python3

import numpy as np
import scipy as sp
import math as math 


"""
----------------------------------------------------------------
-----------------Discrete Time Low Pass Filters-----------------
----------------------------------------------------------------
"""


###-----------------------First-Order-IIR-Fiter--------------------------
### wc -- cut-off angular frequency [rad/s]
### fc - cut-off frequency [1/s] = wc/2*pi
### Ts -- Sampling Period [s]
### fs -- Sampling Frequency [1/s]

def first_order_iir_param(fs,fc):
    wc = 2*math.pi*fc
    Ts = 1/fs
    alpha =  1- (1/(1+(wc*Ts)))
    return alpha 

def first_order_iir(u_input,y_old,alpha):

    y_filtered = alpha*u_input + (1-alpha)*y_old

    return y_filtered 

###-----------------------Exponential-Smoothing--------------------------

def exponential_smoothing(u_input,y_old,fc,fs):
    
    Ts = 1/fs
    Dt = 1/fc

    alpha =  1- math.exp(-Ts/Dt)
    y_smoothed = alpha*u_input + (1-alpha)*y_old
    
    return y_smoothed

#-----------------------Moving-Average-Filter(FIR)---------------------