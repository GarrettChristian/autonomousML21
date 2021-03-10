import roslib
import rospy
import sys
import os
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
# add the path to our openpose and import
sys.path.append(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/build/python', ''))
from openpose import pyopenpose as op

class pose_tracking:

    def __init__(self):
        rospy.init_node('pose_tracker')
        rospy.loginfo("Started pose tracking node!")

        # publishing to and subscribing to
        self.passenger_safe_pub = rospy.Publisher('/passenger_safe', Bool, queue_size=10)
<<<<<<< HEAD
        # rospy.Subscriber('/zed/image_raw', Image, self.classify_image)
        rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.classify_image)
=======
        rospy.Subscriber('/zed/image_raw', Image, self.classify_image)
>>>>>>> 53b46c4a9a63bd1daddba765a459464fdf39281a

        self.bridge = CvBridge()

        # set up openpose params
        self.params = dict()
        self.params["model_folder"] = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/models/', '')
        self.params["number_people_max"] = 1

        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
        cv2.destroyAllWindows()


    ################################
    ### ROS Topics To Publish To ###
    ################################

    def sendPassengerUnsafe(self):
        print("unsafe posted")
        self.passenger_safe_pub.publish(False)

    def sendPassengerSafe(self):
        print("safe posted")
        self.passenger_safe_pub.publish(True)

    def sendPassengerExit(self):
        print("exit posted")
        self.passenger_exit_pub.publish(True)


    def classify_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv_image = cv_image[0:rows, 0:cols/2]
        flip_image = cv2.flip(cv_image, -1) # flip the zed image right side up
        print(flip_image.shape)

        # visually draw boundaries
        line_thickness = 2
        left_boundary = 150
        right_boundary = cols/2 - 150
        cv2.line(flip_image, (left_boundary, 0), (left_boundary, rows), (0, 255, 0), thickness=line_thickness)
        cv2.line(flip_image, (right_boundary, 0), (right_boundary, rows), (0, 255, 0), thickness=line_thickness)

        # Process image
        datum = op.Datum()
        datum.cvInputData = flip_image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

        # Display Image
        poseKeypoints = datum.poseKeypoints
        # print("pose keypoints")
        # print(poseKeypoints)
        # print("left shoulder ", poseKeypoints[0][2])
        # print("right shoulder ", poseKeypoints[0][5])

        # validate that we're within the boundaries
        left_shoulder = poseKeypoints[0][2][0]
        right_shoulder = poseKeypoints[0][5][0]

        if (left_shoulder > left_boundary
            and left_shoulder < right_boundary
            and right_shoulder > left_boundary
            and right_shoulder < right_boundary):
            self.sendPassengerSafe()
        else:
            self.sendPassengerUnsafe()

        cv2.imshow("Frame", datum.cvOutputData)
        cv2.waitKey(3)


if __name__ == "__main__":
    try:
        pose_tracking()
    except rospy.ROSInterruptException:
        pass
