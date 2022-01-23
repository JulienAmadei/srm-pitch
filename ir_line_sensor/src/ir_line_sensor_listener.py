## Listener that listens to strings published to the 'messages' topic
# With help from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code-1

#!/usr/bin/env python

# To write a ROS Node
import rospy
# Use the "string" type
from std_msgs.msg import Bool 

# Used for subscribing to messages
def callback(data):
    # Prints to the screen
    rospy.loginfo(rospy.get_caller_id() + ': I heard %s', data.data)

def listener():
    # Create a Listener Node
    rospy.init_node('ir_line_sensor_listener', anonymous=True)

    # Used for subscribing to 'messages' topic
    rospy.Subscriber('ir_line_sensor', Bool, callback)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

# To execute when invoked directly 
if __name__ == '__main__':
    # Run code
    listener()
