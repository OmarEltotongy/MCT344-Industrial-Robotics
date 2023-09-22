import rospy
from std_msgs.msg import Float64, Float32MultiArray
import numpy as np

def DH_parameters(theta,d,a,alpha):
    DH_table=np.array([[np.cos(theta),  -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha),     a*np.cos(theta)],
                       [np.sin(theta),  np.cos(theta)*np.cos(alpha),  -np.cos(theta)*np.sin(alpha),    a*np.sin(theta)],
                       [0,               np.sin(alpha),                np.cos(alpha),                   d],
                       [0,               0,                            0,                               1]])
    return DH_table

def forward_kin(a,b,c,d):
    const_angle=np.arctan2(0.024,0.128)
    _0T1 = DH_parameters(a,                          0.077,           0,    np.pi/2)
    _1T2 = DH_parameters(-b-const_angle+np.pi/2,         0,       0.130,          0)
    _2T3 = DH_parameters(-c+const_angle-np.pi/2,         0,       0.124,          0)
    _3T4 = DH_parameters(-d,                             0,       0.126,          0)
    _0T4 = _0T1 @ _1T2 @ _2T3 @ _3T4
    return _0T4

if __name__ == "__main__":

    rospy.init_node("my_forward_kinematic_node")

    # Setup wheel speed publishers
    joint1_pub=  rospy.Publisher("/joint1_position/command", Float64, queue_size=10)
    joint2_pub=  rospy.Publisher("/joint2_position/command", Float64, queue_size=10)
    joint3_pub=  rospy.Publisher("/joint3_position/command", Float64, queue_size=10)
    joint4_pub=  rospy.Publisher("/joint4_position/command", Float64, queue_size=10)
    robot_pose_pub= rospy.Publisher("/robot_pose", Float32MultiArray, queue_size=10)

    joint1_angle=np.deg2rad(90)
    joint2_angle=np.deg2rad(20)
    joint3_angle=np.deg2rad(50)
    joint4_angle=np.deg2rad(-35)
    # Setup ROS rate 
    rate = rospy.Rate(10) 

   # rospy.loginfo("simulation has just started")

    while not rospy.is_shutdown():

        rospy.loginfo("Starting..............")
        joint1_pub.publish(Float64(joint1_angle))
        joint2_pub.publish(Float64(joint2_angle))
        joint3_pub.publish(Float64(joint3_angle))
        joint4_pub.publish(Float64(joint4_angle))   
        T =forward_kin(joint1_angle,joint2_angle,joint3_angle,joint4_angle)
        x=T[0,3]
        y=T[1,3]
        z=T[2,3]
        roll=np.arctan2(T[2,1],  T[2,2])
        pitch=np.arctan2(-T[2,0], np.sqrt(T[2,1]**2 + T[2,2]**2))
        yaw=np.arctan2(T[1,0], T[0,0])
        
        pose=[x,y,z,roll,pitch,yaw]

        rospy.loginfo("calculated robot pose is:   ")
        rospy.loginfo(np.array(pose).astype(np.float16))
        pose_msg=Float32MultiArray()
        pose_msg.data=pose
        robot_pose_pub.publish(pose_msg)        
        rate.sleep()