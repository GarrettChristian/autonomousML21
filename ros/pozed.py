import roslib
import rospy
import sys
import os
import math
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

        # publishing to and subscribing to get_poses
        rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image, self.get_poses)
        rospy.Subscriber("/zed/zed_node/depth/depth_registered",Image, self.classify_depth)
        # rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image, self.classify_image)
        # rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.depth_image)
        self.passenger_safe_pub = rospy.Publisher('/passenger_safe', Bool, queue_size=10)
        self.bridge = CvBridge()

        # set up openpose params
        self.params = dict()
        self.params["model_folder"] = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/models/', '')
        # self.params["number_people_max"] = 1

        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()

        # Variable to publish poses
        # self.found_poses = []
        self.recorded_depth = None

        rate = rospy.Rate(10)
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


    def get_poses(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        
        # flip_image = cv_image
        # print(flip_image.shape)

        y, x, _ = np.shape(cv_image)

        left_boundary = 290
        right_boundary = x - 290
        cv2.line(cv_image, (left_boundary, 0), (left_boundary, y), (0, 255, 0), thickness=2)
        cv2.line(cv_image, (right_boundary, 0), (right_boundary, y), (0, 255, 0), thickness=2)

        # shoulders in boundary and are in range max of two detected
        valid_pose = 0
        # some in cart but shoulders not within boundary
        invalid_pose = 0
        # no one detected
        if self.recorded_depth is not None:

            # pre process image
            # mask = self.recorded_depth where it > 1.2
            mask = cv2.inRange(self.recorded_depth, 0.1, 1.5)
            cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # cv_image = cv2.flip(cv_image, -1) # flip the zed image right side up

            # Process image for poses
            datum = op.Datum()
            datum.cvInputData = cv_image
            self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

            # Display Image
            poseKeypoints = datum.poseKeypoints
            # print("pose keypoints")
            # print(poseKeypoints)
            # print("left shoulder ", poseKeypoints[0][2])
            # print("right shoulder ", poseKeypoints[0][5])
            # self.found_poses = poseKeypoints

            if poseKeypoints is not None and len(poseKeypoints) > 0:
                for pose in poseKeypoints:
                    # get x and y locations of neck
                    x_neck = int(pose[1][0])
                    y_neck = int(pose[1][1])

                    # print("distance valid")
                    x_left_shoulder = int(pose[2][0])
                    y_left_shoulder = int(pose[2][1])
                    x_right_shoulder = int(pose[5][0])
                    y_right_shoulder = int(pose[5][1])


                    # print("NECK AT NAN OR GOOD: ", (y_neck, x_neck))
                    # print("LEFT SHOUL AT NAN OR GOOD: ", (y_left_shoulder, x_left_shoulder))
                    # print("RIGHT SHOUL AT NAN OR GOOD: ", (y_right_shoulder, x_right_shoulder))

                    # print("NECK AT NAN OR GOOD: ", self.recorded_depth[(y_neck, x_neck)])
                    # print("LEFT SHOUL AT NAN OR GOOD: ", self.recorded_depth[(y_left_shoulder, x_left_shoulder)])
                    # print("RIGHT SHOUL AT NAN OR GOOD: ", self.recorded_depth[(y_right_shoulder, x_right_shoulder)])
                    
                    neck_distance = self.recorded_depth[(y_neck, x_neck)]
                    left_shoulder_dist = self.recorded_depth[(y_left_shoulder, x_left_shoulder)]
                    right_shoulder_dist = self.recorded_depth[(y_right_shoulder, x_right_shoulder)]

                    distance = float("-inf")

                    if not math.isnan(neck_distance):
                        distance = max(distance, neck_distance)
                    if not math.isnan(left_shoulder_dist):
                        distance = max(distance, left_shoulder_dist)
                    if not math.isnan(right_shoulder_dist):
                        distance = max(distance, right_shoulder_dist)
                    if distance == float("-inf"):
                        distance = float('NaN')



                    # distance = self.recorded_depth[(y_neck, x_neck)]
                    # print("meters at pose ", distance)

                    # if (math.isnan(distance) or (distance > 0 and distance < 1.5)):
                    # if (not math.isnan(distance) and distance > 0 and distance < 1.5):

                    # cv2.line(cv_image, (0, y_neck), (x, y_neck), (0, 255, 0), thickness=2)
                    # cv2.line(cv_image, (x_neck, 0), (x_neck, y), (0, 255, 0), thickness=2)

                    distance_text = "dist: {}".format(distance)
                    # print(distance_text)
                    

                    in_boundaries = (x_left_shoulder > left_boundary
                        and x_left_shoulder < right_boundary
                        and x_right_shoulder > left_boundary
                        and x_right_shoulder < right_boundary)

                    # in boundaries with within our distance threshold
                    if (in_boundaries):
                        valid_pose += 1
                        cv2.putText(cv_image, distance_text, (x_left_shoulder + 5, y_left_shoulder + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA, False)
                        cv2.circle(cv_image, (x_neck, y_neck), 4, (0, 255, 0), thickness=2)
                        cv2.circle(cv_image, (x_left_shoulder, y_left_shoulder), 4, (0, 255, 0), thickness=2)
                        cv2.circle(cv_image, (x_right_shoulder, y_right_shoulder), 4, (0, 255, 0), thickness=2)

                    else:
                        invalid_pose += 1

                        cv2.putText(cv_image, distance_text, (x_left_shoulder + 5, y_left_shoulder + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA, False)
                        cv2.circle(cv_image, (x_neck, y_neck), 4, (0, 0, 255), thickness=2)
                        cv2.circle(cv_image, (x_left_shoulder, y_left_shoulder), 4, (0, 0, 255), thickness=2)
                        cv2.circle(cv_image, (x_right_shoulder, y_right_shoulder), 4, (0, 0, 255), thickness=2)

                    # not in boundaries and you have a within our distance threshold
                    # else not in boundaries and bad dist so cart is empty



        if (invalid_pose > 0 or valid_pose > 3):
            # post invalid pose
            print("invalid")
        elif(valid_pose > 0) :
            # no valid cart patrons detected!
            print("valid!")
        else:
            # cart empty increment empty counter 
            # if empty counter is at "threshold of max empty" post cart empty 
            print("empty?")

        # cv2.imshow("Frame", datum.cvOutputData)
        cv2.imshow("Frame", cv_image)
        cv2.waitKey(3)  

    def classify_depth(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # cv_image = cv2.flip(cv_image, -1)

        mask = cv2.inRange(cv_image, 0.0, 1.5)
        cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # try smoothing the image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # cv_image = cv2.morphologyEx(cv_image, cv2.MORPH_CLOSE, kernel)
        cv_image = cv2.dilate(cv_image, kernel, 1)
        # cv_image = cv2.morphologyEx(cv_image, cv2.MORPH_OPEN, kernel)
        # cv_image = cv2.morphologyEx(cv_image, cv2.MORPH_GRADIENT, kernel)

        self.recorded_depth = cv_image

        # cv2.imshow("Frame", cv_image)
        # cv2.waitKey(3)

if __name__ == "__main__":
    try:
        pose_tracking()
    except rospy.ROSInterruptException:
        pass
