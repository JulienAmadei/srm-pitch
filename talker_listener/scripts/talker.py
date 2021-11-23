## Talker that publishes a message to the messages topic.
# With help from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code

#!/usr/bin/env python

# To write a ROS Node
import rospy
# To get the date
import datetime
# Use the "string" type
from std_msgs.msg import String 

def talker():
    # Node is publishing to the messages topic using the message type String.
    # Queue size prevents from displaying too many messages if any subscriber is not receiving them fast enough.
    pub = rospy.Publisher('average', String, queue_size = 10) 
    
    # Tells rospy the name of the node -- until rospy has this information, it cannot start communicating with the ROS Master.
    # anonymous = True ensures that the node has a unique name by adding random numbers to the end of it.
    rospy.init_node('talker', anonymous=True)

    # This line creates a Rate object rate. 
    # Offers a convenient way for looping at the desired rate. 
    # With an argument of 10, we will go through the loop 10 times per second.
    rate = rospy.Rate(10) # 10hz

    # Checking the rospy.is_shutdown() flag and then looping. 
    # We check is_shutdown() to see if we should exit.  
    while not rospy.is_shutdown():
        # String to be published : Output at time (current_time and date)
        msg_str = "Output at time (%s)" % datetime.datetime.fromtimestamp(rospy.get_time())
        # Prints the messages to screen, write them to the Node's log file, and write them to rosout.
        rospy.loginfo(msg_str)
        # Publishes a string to our messages topic
        pub.publish(msg_str)
        # Sleeps just long enough to maintain the desired rate through the loop. 
        rate.sleep()

# To execute when invoked directly 
if __name__ == '__main__':
    try:
        # Run code
        talker()
    # Catches a rospy.ROSInterruptException exception for when the Node is shutdown
    # so that we don't accidentally continue executing code after the sleep().
    except rospy.ROSInterruptException:
        pass
