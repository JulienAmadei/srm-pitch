#########################
# Imports
import rospy
from std_msgs.msg import Bool 

#########################
# Functions
def callback(data):
    """
    Generic callback function
    """
    rospy.loginfo(rospy.get_caller_id() + ': I heard %s', data.data)

def listener():
    """
    Generic listener function
    """
    rospy.init_node('ir_line_sensor_listener', anonymous=True)
    rospy.Subscriber('ir_line_sensor', Bool, callback)
    rospy.spin()

# To execute when invoked directly 
if __name__ == '__main__':
    listener()
