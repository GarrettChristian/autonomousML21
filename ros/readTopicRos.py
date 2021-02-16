import roslib
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from cv_bridge.boost.cv_bridge_boost import getCvType

# following this tutorial 
# https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html
# moved to 
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/zed/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)







# ATTEMPT #1

# # Define a function to show the image in an OpenCV Window
# def show_image(img):
#     cv2.imshow("Image Window", img)
#     cv2.waitKey(3)

# # Define a callback for the Image message
# def image_callback(img_msg):
#     # log some info about the image topic
#     rospy.loginfo(img_msg.header)

#     # Try to convert the ROS Image message to a CV2 Image
#     try:
#         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     except CvBridgeError:
#         rospy.logerr("CvBridge Error")

#     # Show the converted image
#     show_image(cv_image)


# # this was not in the tutorial but it fixed some errors
# rospy.init_node('test_node_name', anonymous=True)

# # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
# rospy.loginfo("Hello ROS!")

# # Initialize the CvBridge class
# bridge = CvBridge()

# # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# sub_image = rospy.Subscriber("/zed/image_raw", Image, image_callback)

# # Initialize an OpenCV Window named "Image Window"
# cv2.namedWindow("Image Window", 1)

# # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
# while not rospy.is_shutdown():
#     rospy.spin()
