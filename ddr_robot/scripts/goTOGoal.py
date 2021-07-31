#!/usr/bin/env python
# this script is for implement Go to Goal control on differential drive robot
import math
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *
import time


class DDR:

    n         = 0
    mode      = 0
    mode_int  = 0
    angle_old = 0
    kp_lin    = 0.2
    kp_ang    = 0.6
    def __init__(self):
        rospy.init_node('ddrobot_control', anonymous=True)
        rospy.Subscriber('/ddr_control/joint_states', JointState, self.state_Callback)
        rospy.Subscriber('/imu',Imu,self.imu_Callback)
        # publisher variables
        self.pub_plot = rospy.Publisher('/plot_data',Float64MultiArray, queue_size=10)
        self.pub_ddr  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state     =  None
        self.imu_data  =  None
        
        self.rate = rospy.Rate(100)
        
    def state_Callback(self, data):
        self.state = data
        
        
    def imu_Callback(self, data):
        self.imu_data = data
    
       
    def proportionalControl(self, goal_pose,angle):
        error_linear  = math.sqrt( pow((goal_pose.data[0] - self.x),2) + pow((goal_pose.data[1]-self.y),2) ) 
        Ref_ang = math.atan2(goal_pose.data[1]-self.y, goal_pose.data[0]-self.x)
        linear_input = self.kp_lin*error_linear
        angular_input = self.kp_ang*(Ref_ang - angle)
        return linear_input, angular_input, error_linear


    def simulation(self):
        dt        = 0.01 # sample time: 10 mili sec. 
        b         = 0.36  # distance between two wheels
        r         = 0.1  # radius of two wheel
        yaw_j     = 0   
        self.x    = 0
        self.y    = 0
        t         = 0
        
        self.tim       = []
        self.data1     = []
        self.data2     = []
        self.data3     = []
        self.data4     = []
        self.data5     = []
        self.data6     = []
        self.data7     = []
        self.data8     = []
        self.data9     = []
        dataTr = Float64MultiArray()
        cmd_velocity = Twist()
        goal_pose    = Float64MultiArray()
        

        while self.state is None or self.imu_data is None:
            pass
        goal_pose.data.append( float(input("X goal pose: ")) )
        goal_pose.data.append( float(input("Y goal Pose: ")) )
        
        # data from jointState
        th_left  = self.state.position[0]
        th_right = self.state.position[1]
        w_left   = self.state.velocity[0]
        w_righ   = self.state.velocity[1]
        
        # data from imu
        q0_imu   = self.imu_data.orientation.x
        q1_imu   = self.imu_data.orientation.y
        q2_imu   = self.imu_data.orientation.z
        q3_imu   = self.imu_data.orientation.w
                                        
        # Quaternion to Euler angles conversion
        (roll_imu, pitch_imu, yaw_imu)    =  (euler_from_quaternion([q0_imu,q1_imu,q2_imu,q3_imu]))
        yaw = yaw_imu
        
        #yaw_imu  = yaw_imu*(180/math.pi)
             
        # x-y position by Jointstate
        v_x        =  (r*(w_left + w_righ))/2
        yawD_j     =  -(r*(w_righ-w_left))/b
        yaw_j      =  yaw_j + yawD_j*dt
        self.x     =  self.x + (v_x*math.cos(yaw_j))*dt
        self.y     =  self.y + (v_x*math.sin(yaw_j))*dt
        
        yaw_j_deg  = yaw_j*(180/math.pi)

        # Go to goal control
        lin_cmd, ang_cmd, e = self.proportionalControl(goal_pose,yaw)
        while e >= 0.1:
            tic = time.time()
            # data from jointState
            th_left  = self.state.position[0]
            th_right = self.state.position[1]
            w_left   = self.state.velocity[0]
            w_righ   = self.state.velocity[1]
            # x-y position by Jointstate
            v_x        =  (r*(w_left + w_righ))/2
            yawD_j     =  -(r*(w_righ-w_left))/b
            yaw_j      =  yaw_j + yawD_j*dt
            self.x     =  self.x + (v_x*math.cos(yaw_j))*dt
            self.y     =  self.y + (v_x*math.sin(yaw_j))*dt
            # subscribe to imu topic
            q0_imu   = self.imu_data.orientation.x
            q1_imu   = self.imu_data.orientation.y
            q2_imu   = self.imu_data.orientation.z
            q3_imu   = self.imu_data.orientation.w
            # Quaternion to Euler angles conversion
            (roll_imu, pitch_imu, yaw_imu)    =  (euler_from_quaternion([q0_imu,q1_imu,q2_imu,q3_imu]))
            yaw = yaw_imu
            lin_cmd, ang_cmd, e = self.proportionalControl(goal_pose,yaw)
            cmd_velocity.linear.x  =  lin_cmd
            cmd_velocity.angular.z =  -ang_cmd
            # gives command velocity to robot
            self.pub_ddr.publish(cmd_velocity)
            
            # Feedback of robot for analysis
            dataTr.data = [t, yaw,yaw_j,self.x, self.y]
            self.pub_plot.publish(dataTr)
            toc =time.time()
            dtt = toc-tic
            print(round(self.x,2),round(self.y,2))
            self.rate.sleep()
            t = t + dt
            
            
        
        cmd_velocity.linear.x  = 0
        cmd_velocity.angular.z = 0            
        self.pub_ddr.publish(cmd_velocity)
        print('robot reached the goal')
          
        rospy.spin()
        


if __name__ == '__main__':
    try:
        x = DDR()
        x.simulation()
        
    except rospy.ROSInterruptException:
        pass