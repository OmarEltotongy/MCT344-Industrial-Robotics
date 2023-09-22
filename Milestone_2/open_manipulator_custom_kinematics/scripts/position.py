# Import necessary modules
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

# Define end effector coordinates
X_End_Effector = 0.14
Y_End_Effector = -0.20
Z_End_Effector = 0.1
p=[X_End_Effector,Y_End_Effector,Z_End_Effector]

# Initialize node and publisher
if __name__ == "__main__":
    rospy.loginfo("Starting..............")
    rospy.init_node("position_node")
    target_pub= rospy.Publisher("/target_goal", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10) 
    
    # Publish target position message at a specified rate
    while not rospy.is_shutdown():
        target_msg=Float32MultiArray()
        target_msg.data=p
        target_pub.publish(target_msg)  
        rate.sleep()
