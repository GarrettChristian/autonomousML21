import rospy
from sensor_msgs.msg import Image

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

sub_image = rospy.Subscriber("/zed/image_raw", Image, image_callback)

while not rospy.is_shutdown():
    rospy.spin()

