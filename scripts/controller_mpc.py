#!/usr/bin/env python3
import rospy
import math
import array as arr
import numpy as np
from sensor_msgs.msg import JointState
from math import sin, cos, acos, atan2, pi, sqrt

k1 = 15.99
k2 = 5.65
k3 = k1
g = 9.81
  
  #declaring all variables as float
x1=np.zeros(2)
x2=np.zeros(2)

#ref_ang=np.zeros(2)

def desired_thetas(p):
    px=p[0]
    py=p[1]
    l1=1
    l2=1
    ctheta2 = (px**2 + py**2 - l1**2 - l2**2) / (2 *l1 * l2)
    stheta2 = math.sqrt(1-math.pow(ctheta2, 2))
    ctheta1 = (px * (l1 + l2 * ctheta2) + py * l2 *stheta2) / (px**2 + py**2)

    theta1p=math.atan2(math.sqrt(1-math.pow(ctheta1,2)), ctheta1)
    theta1n=math.atan2(math.sqrt(1-math.pow(ctheta1,2))*(-1), ctheta1)

    theta2p=math.atan2(math.sqrt(1-math.pow(ctheta2,2)), ctheta2)
    theta2n=math.atan2(math.sqrt(1-math.pow(ctheta2,2))*(-1), ctheta2)
    
    theta2sign=theta2p
    
    if (px>=1 and py>=1):
        theta1sign=theta1p    
    elif (px>=1 and py<=1):
        theta1sign=theta1n
    elif (px<=1 and py>=1):
        theta1sign=theta1p
    else:
        theta1sign=theta1n
        
    r=[theta1sign, theta2sign]
    return r

def step1(r,x,dx):   
    v = np.array([k3*r[0] - k1*x[0] - k2*dx[0], k3*r[1] - k1*x[1] - k2*dx[1]])
    return v

def step2(x,dx,v):    
    M = np.array([[-1 + 2*cos(x[1]), 1 + cos(x[1])],[1 + cos(x[1]),  1]])
    tau1 = np.array([M[0][0]*v[0] + M[0][1]*v[1], M[1][0]*v[0] + M[1][1]*v[1]])
    G = np.array([g*(-2-sin(x[0]+x[1])), -g*sin(x[0]+x[1])])
    C = np.array([(2*dx[0]*dx[1] + (dx[0])**2)*sin(x[1]),-dx[0]*dx[1]*sin(x[1])]) 
    u = np.array(tau1 + C + G)
    return u

def step3(x,dx,u):
    M = np.array([[-1 + 2*cos(x[1]), 1 + cos(x[1])],[1 + cos(x[1]),  1]])
    invM=np.linalg.inv(M)
    
    C = np.array([(2*dx[0]*dx[1] + (dx[0])**2)*sin(x[1]),-dx[0]*dx[1]*sin(x[1])])
    G = np.array([g*(-2-sin(x[0]+x[1])), -g*sin(x[0]+x[1])]) 
    v=np.array(G+C+u)
    
    ddx = np.array([invM[0][0]*(-v[0]) + invM[0][1]*(-v[1]), invM[1][0]*(-v[0]) + invM[1][1]*(-v[1])])
    return ddx

def intn(ddx, dx2, x2, t):
    x=np.array(ddx*(t**2)+dx2*t+x2)
    return (x[0],x[1])

def sender():
    
    jspub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.init_node('controller_node')
    t2=rospy.get_time()
    R = rospy.get_param('~controller_pub_rate')
    rate = rospy.Rate(R)

    cmd = JointState()
    #event = rospy.timer.TimerEvent
    #rospy.Timer(rospy.Duration(1), diffn, oneshot=False)
    
    global x1
    global x2
    
    ref_pos=np.array([1.1,-1.5])
    ref_ang=desired_thetas(ref_pos)
    
    dx2=np.array([0.0,0.0])
    x1=np.array([0.0,0.0])
    x2=np.array([0.0,0.0])    
    
    while not rospy.is_shutdown ():
        cmd.header.stamp = rospy.Time.now()
        cmd.name = ['baseHinge', 'interArm']

        t1= rospy.get_time() 
        dx1=np.array(np.subtract(x1,x2)/np.subtract(t1,t2))
        
        #MPC algorithm
        v=np.array(step1(ref_ang,x1,dx1))
        u=np.array(step2(x1,dx1,v))
        ddx=np.array(step3(x1,dx1,u))
        
        #new position obtained from MPC
        x1=np.array(intn(ddx, dx2, x2,float(t1-t2)))
        
        #publishing new position unless final position reached
        if ((x1[0]==x2[0])and(x1[1]==x2[1])):
            rate.sleep()
        else:
            cmd.position = desired_thetas(x1)
            jspub.publish(cmd)
        x2=x1
        t2=t1
        dx2=dx1
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
