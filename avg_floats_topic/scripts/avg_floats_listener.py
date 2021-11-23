## Listener that listens to strings published to the 'messages' topic
# With help from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code-1

#!/usr/bin/env python3

# To write a ROS Node
import rospy
# Use the "string" type
from std_msgs.msg import String, Float64 

# Used for subscribing to messages
def callback(data):
    # Prints to the screen
    rospy.loginfo(data.data)

def listener():
    # Create a Listener Node
    rospy.init_node('listener', anonymous=True)

    # Used for subscribing to 'messages' topic
    rospy.Subscriber('average', Float64, callback)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

# To execute when invoked directly 
if __name__ == '__main__':
    print(f"I'm listening.\n")
    # Run code
    listener()
