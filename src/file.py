#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

def main():
    # initialize node
    rospy.init_node('sliding_mode_node', anonymous=True)
    rate = rospy.Rate(20.0)

    # define setpoint
    setpoint = PoseStamped()
    setpoint.pose.position.x = 0.0
    setpoint.pose.position.y = 0.0
    setpoint.pose.position.z = 0.5

    # define publisher
    pub  = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)    

    start = rospy.get_rostime()
    armed_once = False
    offb = False

    while not rospy.is_shutdown():
        pub.publish(setpoint)
        rate.sleep()

        if (rospy.get_rostime().secs - start.secs)  > 10.0 and not armed_once:
            armed_once = True
            rospy.wait_for_service('mavros/cmd/arming')
            try:
                armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                armService(True)
                print("Tried arming.")
            except rospy.ServiceException as e:
                print("Service arming call failed: %s"%e)      

        if armed_once and not offb:
            offb = True
            rospy.wait_for_service('mavros/set_mode')
            try:
                flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                flightModeService(custom_mode='OFFBOARD')
            except rospy.ServiceException as e:
                print("service set_mode call failed: %s. Offboard Mode could not be set."%e)   

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
