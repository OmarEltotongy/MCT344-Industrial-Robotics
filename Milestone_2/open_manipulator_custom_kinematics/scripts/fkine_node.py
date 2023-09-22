import rospy
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import JointState
import numpy as np

# Initialize the joint angles as 0
joint1_angle=0
joint2_angle=0
joint3_angle=0
joint4_angle=0

# Function to compute the DH parameters for a given joint
def DH_parameters(theta,d,a,alpha):
    # Create the DH parameter matrix
    DH_table=np.array([[np.cos(theta),  -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha),     a*np.cos(theta)],
                       [np.sin(theta),  np.cos(theta)*np.cos(alpha),  -np.cos(theta)*np.sin(alpha),    a*np.sin(theta)],
                       [0,               np.sin(alpha),                np.cos(alpha),                   d],
                       [0,               0,                            0,                               1]])
    return DH_table

# Function to compute the forward kinematics
def forward_kin(a,b,c,d):
    # Compute the constant angle (angle between two links in the robot arm)
    const_angle=np.arctan2(0.024,0.128)
    
    # Compute the DH parameter matrices for each joint
    _0T1 = DH_parameters(a,                          0.077,           0,    np.pi/2)
    _1T2 = DH_parameters(-b-const_angle+np.pi/2,         0,       0.130,          0)
    _2T3 = DH_parameters(-c+const_angle-np.pi/2,         0,       0.135,          0)
    _3T4 = DH_parameters(-d,                             0,       0.126,          0)
    
    # Compute the transformation matrix from the base to the end effector
    _0T4 = _0T1 @ _1T2 @ _2T3 @ _3T4
    return _0T4

# Callback function to update the joint angles
def callback(msg):
    # Declare the joint angles as global so that they can be accessed outside the function
    global joint1_angle
    global joint2_angle
    global joint3_angle
    global joint4_angle
    
    # Update the joint angles based on the message from the joint_states topic
    joint1_angle = msg.position[2]
    joint2_angle = msg.position[3]
    joint3_angle = msg.position[4]
    joint4_angle = msg.position[5]

if __name__ == "__main__":

    rospy.init_node("my_forward_kinematic_node")
    rospy.Subscriber("/joint_states", JointState, callback)
    robot_pose_pub= rospy.Publisher("/robot_pose", Float32MultiArray, queue_size=10)

    # Setup ROS rate 
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():

        rospy.loginfo("Starting..............")
       
        # Call forward kinematics function and calculate pose
        T = forward_kin(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
        x = T[0,3]
        y = T[1,3]
        z = T[2,3]
        roll = np.arctan2(T[2,1],  T[2,2])
        pitch = np.arctan2(-T[2,0], np.sqrt(T[2,1]**2 + T[2,2]**2))
        yaw = np.arctan2(T[1,0], T[0,0])
        
        pose = [x, y, z, roll, pitch, yaw]

        rospy.loginfo("calculated robot pose is:   ")
        rospy.loginfo(np.array(pose).astype(np.float16))

        # Publish calculated pose
        pose_msg = Float32MultiArray()
        pose_msg.data = pose
        robot_pose_pub.publish(pose_msg)        
        rate.sleep()
