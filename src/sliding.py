#!/usr/bin/env python

import rospy
from cmath import tanh
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Controller:
    def __init__(self):
        self.pos_z = 0.0
        self.vel_z = 0.0

    ## Callbacks
    def local_pos_callback(self,msg):
        self.pos_z = msg.pose.position.z
    
    def local_vel_callback(self,msg):
        self.vel_z = msg.twist.linear.z
    ## End

def main():

    rospy.init_node('sliding_mode_node', anonymous=True)
    rate = rospy.Rate(50.0)

    myc = Controller()

    setpoint = PositionTarget()

    pub  = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)    

    local_pos = rospy.Subscriber('mavros/local_position/pose', PoseStamped, myc.local_pos_callback)
    local_vel = rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, myc.local_vel_callback)

    ## Parameters
    lambda_param = 5
    k_param = 2
    pos_z_des = 0.85
    ## End

    start = rospy.get_rostime()
    armed_once = False
    offb = False

    Ts = 1/50
    B = [0 Ts 0] #column vector or numpy
    F = [1 Ts 0; 0 1 -Ts; 0 0 1] # numpy??
    Q = identity 3x3 # numpy

    R = [1000] #numpy vector
    H = [1 0 0] #line vector or numpy


    myEKF = KalmanFilter(dim_x=3, dim_z=1)

    while not rospy.is_shutdown():

        ## EKF

        myEKF.predict(acc_setpoint, B, F, Q)
        myEKF.update(myc.pos_z, R, H)
        
        ##

        ## Acceleration setpoint calculation using Sliding Mode
        sz = myc.vel_z + lambda_param * (myc.pos_z - pos_z_des)
        val = tanh(sz/0.1).real
        acc_setpoint = -k_param * val  - lambda_param * myEKF.x(1) #assuming from zero

        #The third one is accelearation of the elevator

        ## End

        if (rospy.get_rostime().secs - start.secs) > 45:
            setpoint.type_mask = PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_YAW_RATE
            setpoint.coordinate_frame = 1
            setpoint.position.x = 0.0
            setpoint.position.y = 0.0
            # setpoint.acceleration_or_force.x = 0.0
            # setpoint.acceleration_or_force.y = 0.0
            setpoint.acceleration_or_force.z = acc_setpoint
            # print("switched to acc")
            # print(acc_setpoint)
        else:
            setpoint.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_YAW
            setpoint.coordinate_frame = 1
            setpoint.position.x = 0.0
            setpoint.position.y = 0.0
            setpoint.position.z = 0.5


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
                print("Service set_mode call failed: %s. Offboard Mode could not be set."%e)   

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass