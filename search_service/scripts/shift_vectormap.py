#! /usr/bin/python

import rospy
import actionlib
import numpy as np
import os
import sys
import nav_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose



_FRAME_ID = 'map'
_INPUT_FILENAME = '/home/knapsack/workspaces/test_ws/src/mrsearch/tmp/vectormaps/ahg-alley.vectormap.txt'
_OUTPUT_FILENAME = '/home/knapsack/workspaces/test_ws/src/mrsearch/tmp/vectormaps/ahg-alley-shift.vectormap.txt'


def vectormap_manager():    
    
    map_txt = open(_INPUT_FILENAME, 'r')
    shift_txt = open(_OUTPUT_FILENAME, 'w')

    for line in map_txt:
    	

        values = line.split(',')
        new_values = np.zeros(4)
        new_values[0] = float(values[0]) - 1.0
        new_values[1] = float(values[1]) + 1.0
        new_values[2] = float(values[2]) - 1.0
        new_values[3] = float(values[3]) + 1.0
        
        new_line = str(new_values[0]) + ", " + str(new_values[1]) + ", " + str(new_values[2]) + ", " + str(new_values[3])
        shift_txt.write(new_line)
        shift_txt.write('\n')
    

    map_txt.close()
    shift_txt.close()
    rospy.loginfo("Shifted Vectormap Written ...")





if __name__ == '__main__':
    rospy.init_node('vectormap_manager')
    vectormap_manager()
    # mains()
