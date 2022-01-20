#!/usr/bin/env python3

# To write a ROS Node
import rospy
from std_msgs.msg import Bool
# Used for subscribing to messages
def callback(data):
    # Prints to the screen
    rospy.loginfo(rospy.get_caller_id() + ': I heard %s', data.data)

def listener():
    # Create a Listener Node
    rospy.init_node('ir_top_listener', anonymous=True)

    # Used for subscribing to 'messages' topic
    rospy.Subscriber('ir_topL', Bool, callback)
    rospy.Subscriber('ir_topR', Bool, callback)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

# To execute when invoked directly 
if __name__ == '__main__':
    # Run code
    listener()
