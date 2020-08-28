#!/usr/bin/env python3

#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import sys

import math 
import time 

from control_talker import control_talker


def main():
    # Read the parameters
    
    talker = control_talker(gain_attitude,gain_position,\
        system_param,thruster_param,control_param,spacecraft_name)

if __name__ == "__main__":
    main()

    