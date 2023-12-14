#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# Initialize the angular tolerance
angl_tol = 3
vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

step = 0
bulldoze_init = False
bulldoze_init_time = 0


def callback(data):
    vel_msg  = Twist()
    if step == 0:         
        # if tag 10 is detected
        if data.z == 1.0:
            # if need to turn more
            if data.x >= 0.25 and data.x <=-0.25:
                vel_msg.angular.z = -0.1
            # if don't need to turn more
            else:
                # stop turning and go straight
                vel_msg.angular.z = 0
                step = 1
        else:
            vel_msg.angular.z = -0.25
    # get close to first marker
    elif step == 1:
        vel_msg.angular.z = 0
        if data.z == 1.0:
            if data.y > 0.25:
                vel_msg.linear.x = 0.1
            else:
                vel_msg.linear.x = 0
                step = 2
    # turn to 2nd marker 
    elif step == 2:
        if data.z == 10.0:
            # if need to turn more
            if data.x >= 0.25 and data.x <=-0.25:
                vel_msg.angular.z = -0.1
            # if don't need to turn more
            else:
                # stop turning and go straight
                vel_msg.angular.z = 0
                step = 3
        else:
            vel_msg.angular.z = -0.25
    # get close to 2nd marker
    elif step == 3:
        if data.z == 20 or data.z == 10:
            vel_msg.angular.z = 0
            if data.y > 0.25:
                vel_msg.linear.x = 0.1
            else:
                vel_msg.linear.x = 0
                step = 4
    # bulldoze
    elif step == 4:
        if not bulldoze_init:
            bulldoze_init_time = rospy.get_rostime().to_sec()
            bulldoze_init = True
        else:
            if rospy.get_rostime().to_sec() - bulldoze_init_time < 3:
                vel_msg.linear.x = 0.1
            else:
                vel_msg.linear.x = 0
                rospy.loginfo("kkk lynching done")
                
        
        

    # rospy.loginfo(data.data)
    # if (data.data > 0):
    #     vel_msg.linear.x  =  0.00
    #     vel_msg.angular.z =  0.25
    #     if (data.data < angl_tol):
    #         vel_msg.linear.x  =  1.00
    #         vel_msg.angular.z =  0.00
    # elif (data.data < 0):
    #    vel_msg.linear.x  = 0.00
    #    vel_msg.angular.z = - 0.25
    #    if (abs(data.data) < angl_tol):
    #         vel_msg.linear.x  =  1.00
    #         vel_msg.angular.z =  0.00

    # Since the Wall-E moving just in x-axis
    # vel_msg.linear.x = speed

    
    vel_pub.publish(vel_msg)



while not rospy.is_shutdown():

    # Initialize a sub nod
    rospy.init_node("subcriber", anonymous=True)

    # Subcribe the alignment angular error
    anglSub = rospy.Subscriber("/mix", Vector3, callback)

    rospy.spin()











    


