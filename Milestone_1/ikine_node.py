import rospy
from std_msgs.msg import Float64
import numpy as np

X_End_Effector=0
Y_End_Effector=0
Z_End_Effector=0
def callback(msg):
    global X_End_Effector
    X_End_Effector = msg.data[0]
    global Y_End_Effector
    Y_End_Effector = msg.data[1]
    global Z_End_Effector
    Z_End_Effector = msg.data[2]
if __name__ == "__main__":

    rospy.init_node("my_inverse_kinematics_node")
    rospy.Subscriber("/target_goal", Float64, callback)
    joint1_pub=  rospy.Publisher("/joint1_position/command", Float64, queue_size=10)
    joint2_pub=  rospy.Publisher("/joint2_position/command", Float64, queue_size=10)
    joint3_pub=  rospy.Publisher("/joint3_position/command", Float64, queue_size=10)


    rate = rospy.Rate(10) 

    d1 = 0.077
    d2 = 0.13
    d3 = 0.25 #we will treat the link three and four as one rigid link
    const_angle=np.arctan2(0.024,0.128)#10.619
    
    joint1_angle=np.arctan2(Y_End_Effector,X_End_Effector)
    r = np.sqrt(X_End_Effector**2 + Y_End_Effector**2)

    R = np.sqrt(r**2 + (Z_End_Effector-d1)**2)
    theta3 = np.arccos((R**2 - d2**2 - d3**2)/(2*d2*d3))
   #in quadrant 1,4 it will be positive else it will be negative and i need to remove the sign
    #to get theta 2 
    alfa = np.arctan2((Z_End_Effector-d1),r)
    gamma = np.arcsin(d3*np.sin(theta3)/R)
    if(0<theta3<np.pi/2) or (3/2*np.pi<theta3<2*np.pi):
        theta2 = alfa - gamma
    else:
        theta2 = alfa + gamma
        theta3=-theta3

    joint2_angle = -theta2 -const_angle +np.pi/2
    joint3_angle = -theta3 + const_angle -np.pi/2
    i=1
    while not rospy.is_shutdown():
        joint1_pub.publish(Float64(joint1_angle))
        joint2_pub.publish(Float64(joint2_angle))
        joint3_pub.publish(Float64(joint3_angle))
 
        if(i==1):
            i+=1
            rospy.loginfo(joint1_angle)
            rospy.loginfo(joint2_angle)
            rospy.loginfo(joint3_angle)
           
        rate.sleep()