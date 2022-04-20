#!/usr/bin/env python

import rospy
from cmath import tanh
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import numpy as np
from kalmanfilter import KalmanFilter

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
    u = 0.0
    ## End

    start = rospy.get_rostime()
    armed_once = False
    offb = False

    Ts = 1/50 # sampling time    

    myEKF = KalmanFilter(dim_x=3, dim_z=1, dim_u=1)
    myEKF.F = np.array([[1., Ts, 0.],
                        [0., 1., -Ts],
                        [0., 0., 1.]])

    myEKF.H = np.array([[1., 0., 0.]])
    myEKF.R = 1000
    myEKF.P = 1000
    myEKF.B = np.array([[0.,Ts, 0.]]).T
    myEKF.u = np.array([u])
    

    while not rospy.is_shutdown():

        ## EKF

        myEKF.predict()
        myEKF.update(myc.pos_z)
        
        ##

        ## Acceleration setpoint calculation using Sliding Mode
        sz = myc.vel_z + lambda_param * (myc.pos_z - pos_z_des)
        val = tanh(sz/0.1).real
        u = -k_param * val  - lambda_param * myEKF.x[1] # accessing numpy array as array[0]
        myEKF.u = np.array([u])

        #The third one is accelearation of the elevator

        ## End

        if (rospy.get_rostime().secs - start.secs) > 45:
            setpoint.type_mask = PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_YAW_RATE
            setpoint.coordinate_frame = 1
            setpoint.position.x = 0.0
            setpoint.position.y = 0.0
            setpoint.acceleration_or_force.z = u
        else:
            setpoint.type_mask = PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_YAW_RATE + PositionTarget.IGNORE_YAW
            setpoint.coordinate_frame = 1
            setpoint.position.x = 0.0
            setpoint.position.y = 0.0
            setpoint.position.z = 0.5


        pub.publish(setpoint)

        # var.publish(myEFK.x)
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