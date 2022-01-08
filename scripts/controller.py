#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import JointState
from math import sin, cos, acos, atan2, pi, sqrt

def desired_thetas(px, py, t):
    l1=1
    l2=1
    
    ctheta2 = (px**2 + py**2 - l1**2 - l2**2) / (2 *l1 * l2)
    stheta2 = math.sqrt(1-math.pow(ctheta2, 2))
    ctheta1 = (px * (l1 + l2 * ctheta2) + py * l2 *stheta2) / (px**2 + py**2)
	
    theta1p=math.atan2(math.sqrt(1-math.pow(ctheta1,2)), ctheta1)
    theta1n=math.atan2(math.sqrt(1-math.pow(ctheta1,2))*(-1), ctheta1)
	
    theta2p=math.atan2(math.sqrt(1-math.pow(ctheta2,2)), ctheta2)
    theta2n=math.atan2(math.sqrt(1-math.pow(ctheta2,2))*(-1), ctheta2)
    
    #defining workspace
    if (px>=1 and py>=1):
        theta1sign=theta1p
        theta2sign=theta2p
    elif (px>=1 and py<=1):
        theta1sign=theta1n
        theta2sign=theta2p
    elif (px<=1 and py>=1):
        theta1sign=theta1p
        theta2sign=theta2p
    elif (px<=1 and py<=1):
        theta1sign=theta1n
        theta2sign=theta2p
    
    if t<100:
        theta1=theta1sign*t/100
        theta2=theta2sign*t/100
    else:	
        theta1=theta1sign
        theta2=theta2sign
    return [theta1, theta2]




def sender():
    jspub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.init_node('controller_node')
    R = rospy.get_param('~controller_pub_rate')
    rate = rospy.Rate(R)

    #T = rospy.get_param('~period')
    cmd = JointState()
    queue_element=0

    while not rospy.is_shutdown():
        cmd.header.stamp = rospy.Time.now()
        t = rospy.get_time()
        cmd.name = ['baseHinge', 'interArm']
        #limits of workspace are:
        # 0<sqrt(px^2 + py^2)<2
        # 
        
        px= 0.1
        py= 0.1
        cmd.position = desired_thetas(px, py, queue_element)
        queue_element=queue_element+1
        jspub.publish(cmd)

        rate.sleep()


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
