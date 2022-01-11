## Listener that listens to strings published to the 'remote_input' topic

#!/usr/bin/env python

import rospy
from std_msgs.msg import String 


def callback(data):
    
    if "None (None)" != data.data:
        rospy.loginfo('Button Pressed:  %s', data.data)
        
    
def listener():
    # Create a Listener Node
    rospy.init_node('remote_listener', anonymous=True)

    # Used for subscribing to 'messages' topic
    rospy.Subscriber('remote_input', String, callback)

    # Keeps python from exiting until this node is stopped
    rospy.spin()

# To execute when invoked directly 
if __name__ == '__main__':
    print("[IR LISTENER WINDOW]")
    # Run code
    listener()
