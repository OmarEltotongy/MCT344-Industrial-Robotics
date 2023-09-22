import rospy
from std_msgs.msg import Float64,Float32MultiArray
import numpy as np

# Initialize the end effector coordinates
X_End_Effector=0
Y_End_Effector=0
Z_End_Effector=0

# Callback function to update the end effector coordinates
def callback(msg):
    global X_End_Effector, Y_End_Effector, Z_End_Effector
    # Extract the end effector coordinates from the message
    X_End_Effector = msg.data[0]
    Y_End_Effector = msg.data[1]
    Z_End_Effector = msg.data[2]

    # Define the link lengths
    d1 = 0.077
    d2 = 0.13
    d3 = 0.25 #we will treat the link three and four as one rigid link

    # Define a constant angle to simplify the inverse kinematics calculation
    const_angle=np.arctan2(0.024,0.128)#10.619

    # Calculate the joint angles using inverse kinematics
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
    
    # Publish the joint angles to the robot
    joint1_pub.publish(Float64(joint1_angle))
    joint2_pub.publish(Float64(joint2_angle))
    joint3_pub.publish(Float64(joint3_angle))

    # Log the joint angles for debugging purposes
    rospy.loginfo(joint1_angle)
    rospy.loginfo(joint2_angle)
    rospy.loginfo(joint3_angle)
    

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("my_inverse_kinematics_node")
    # Subscribe to the /target_goal topic to receive end effector coordinates
    rospy.Subscriber("/target_goal", Float32MultiArray, callback)
    # Initialize publishers to send joint angles to the robot
    joint1_pub=  rospy.Publisher("/joint1_position/command", Float64, queue_size=10)
    joint2_pub=  rospy.Publisher("/joint2_position/command", Float64, queue_size=10)
    joint3_pub=  rospy.Publisher("/joint3_position/command", Float64, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(10) 
    # Start the node
    rospy.spin()
