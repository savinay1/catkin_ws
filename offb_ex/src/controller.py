#!/usr/bin/env python

import rospy
from mavros_msgs.msg import control,Waypoint,CommandCode
import pdb

def main_control():

    #pdb.set_trace()
    rospy.init_node('offb_node', anonymous=True)
    
    pub = rospy.Publisher("contoller",control,queue_size = 10)

    while True:
        n,lat,lon,alt=raw_input().split()

        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = CommandCode.NAV_WAYPOINT
        wp.is_current = True
        wp.autocontinue = True
        wp.param1 = 0
        wp.param2 = 0
        wp.param3 = 0
        wp.param4 = 0
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt

        cont = control()
        cont.waypoint = wp
        cont.uav_no = n

        pub.publish(cont)


if __name__ == '__main__':
    try:
        main_control()
    except rospy.ROSInterruptException:
        pass
