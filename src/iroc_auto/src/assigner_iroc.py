#!/usr/bin/python
import rospy
import cv2
import numpy as np

def main_node():
    my_client = client()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        waypt1 = [2.9, -2.1]
        q = [0,0,1/np.sqrt(2),1/np.sqrt(2)]
        my_client.move_to_goal(waypt1, q)     #default in map frame
        found, pos, orient, timestamp = my_client.detect_target()
        eps = my_client.cal_error(pos, orient)
        counter = 0
        if found:
            while(eps > 0.3 or counter < 100):
                my_client.correct_pose(pos, orient)
                found, pos, orient, timestamp = my_client.detect_target()
                if not found:
                   found, pos, orient, timestamp = my_client.recovery() #TODO: make this into another function
                eps = my_client.cal_error(pos, orient)
                counter += 1
        else:
            found, pos, orient, timestamp = my_client.recovery()
        my_client.pick_object(pos,orient)

        waypt2 = [9.25, -3.25]
        q = [0,0,0,1]
        my_client.move_to_goal(waypt2,q)
        found, pos, orient, timestamp = my_client.detect_container()
        eps = my_client.cal_error(pos, orient)
        counter = 0
        if found:
            while(eps > 0.3 or counter < 100):
                my_client.correct_pose(pos, orient)
                found, pos, orient, timestamp = my_client.detect_container()
                if not found:
                   found, pos, orient, timestamp = my_client.recovery() #TODO: make this into another function
                eps = my_client.cal_error(pos, orient)
                counter += 1
        else:
            found, pos, orient, timestamp = my_client.recovery()
        my_client.drop_object(pos,orient)  
        waypt3 = [0, -0.5]
        my_client.move_to_goal(waypt3, q=None)

if __name__== "__main__":
    try:
        main_node()
    except rospy.ROSInternalException:
        print("Exiting........")    

