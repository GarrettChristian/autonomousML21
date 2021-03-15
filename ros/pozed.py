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

        cv_image = cv2.flip(cv_image, -1) # flip the zed image right side up
        # flip_image = cv_image
        # print(flip_image.shape)

        # Process image
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

        if self.recorded_depth is not None and poseKeypoints is not None and len(poseKeypoints) > 0:
            for pose in poseKeypoints:
                # get x and y locations of neck
                x_neck = int(pose[1][0])
                y_neck = int(pose[1][1])
                # print("x_neck ", x_neck)
                # print("y_neck ", y_neck)

                distance = self.recorded_depth[(y_neck, x_neck)]
                print("meters at pose ", self.recorded_depth[(y_neck, x_neck)], " from here ", (y_neck, x_neck))

                # if (math.isnan(distance) or (distance > 0 and distance < 1.1)):
                if (not math.isnan(distance) and distance > 0 and distance < 1.5):

                    print("Secondary at pose ", self.recorded_depth[(y_neck, x_neck)], " from here ", (y_neck, x_neck))

                    # cv2.line(cv_image, (0, y_neck), (x, y_neck), (0, 255, 0), thickness=2)
                    # cv2.line(cv_image, (x_neck, 0), (x_neck, y), (0, 255, 0), thickness=2)

                    # print("distance valid")
                    x_left_shoulder = pose[2][0]
                    y_left_shoulder = pose[2][1]
                    x_right_shoulder = pose[5][0]
                    y_right_shoulder = pose[5][1]

                    cv2.circle(cv_image, (x_neck, y_neck), 4, (0, 255, 0), thickness=2)
                    cv2.circle(cv_image, (x_left_shoulder, y_left_shoulder), 4, (0, 255, 0), thickness=2)
                    cv2.circle(cv_image, (x_right_shoulder, y_right_shoulder), 4, (0, 255, 0), thickness=2)

                    if (x_left_shoulder > left_boundary
                        and x_left_shoulder < right_boundary
                        and x_right_shoulder > left_boundary
                        and x_right_shoulder < right_boundary):
                        # and not math.isnan(distance)):
                        valid_pose += 1
                    else:
                        invalid_pose += 1



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

        cv_image = cv2.flip(cv_image, -1)

        self.recorded_depth = cv_image
        
        # y, x = np.shape(cv_image)
        # # center_point = (y//2, x//2)
        # # print("ceneter point ", cv_image[center_point])
        # left_boundary = 290
        # right_boundary = x - 290
        # cv2.line(cv_image, (left_boundary, 0), (left_boundary, y), (0, 255, 0), thickness=2)
        # cv2.line(cv_image, (right_boundary, 0), (right_boundary, y), (0, 255, 0), thickness=2)

        # # valid_poses = []

        # # shoulders are cool and are in range max of two detected
        # valid_pose = 0
        # # some in cart but shoulders not cool
        # invalid_pose = 0
        # # no one detected

        # if self.found_poses is not None and len(self.found_poses) > 0:
        #     for pose in self.found_poses:
        #         # get x and y locations of neck
        #         x_neck = int(pose[1][0])
        #         y_neck = int(pose[1][1])
        #         # print("x_neck ", x_neck)
        #         # print("y_neck ", y_neck)

        #         neck_loc = (y_neck, x_neck)
        #         distance = cv_image[(y_neck, x_neck)]
        #         print("meters at pose ", cv_image[neck_loc], " from here ", neck_loc)

        #         # if (not math.isnan(distance) and distance > 0 and distance < 1.5):
        #         if (distance > 0 and distance < 1.2):

        #             # cv2.line(cv_image, (0, y_neck), (x, y_neck), (0, 255, 0), thickness=2)
        #             # cv2.line(cv_image, (x_neck, 0), (x_neck, y), (0, 255, 0), thickness=2)

        #             # print("distance valid")
        #             x_left_shoulder = pose[2][0]
        #             y_left_shoulder = pose[2][1]
        #             x_right_shoulder = pose[5][0]
        #             y_right_shoulder = pose[5][1]

        #             cv2.circle(cv_image, (x_neck, y_neck), 4, (0, 255, 0), thickness=2)
        #             cv2.circle(cv_image, (x_left_shoulder, y_left_shoulder), 4, (0, 255, 0), thickness=2)
        #             cv2.circle(cv_image, (x_right_shoulder, y_right_shoulder), 4, (0, 255, 0), thickness=2)

        #             if (x_left_shoulder > left_boundary
        #                 and x_left_shoulder < right_boundary
        #                 and x_right_shoulder > left_boundary
        #                 and x_right_shoulder < right_boundary):
        #                 valid_pose += 1
        #             else:
        #                 invalid_pose += 1



        # if (invalid_pose > 0 or valid_pose < 3):
        #     # post invalid pose
        #     print("invalid")
        # elif(valid_pose > 0) :
        #     # no valid cart patrons detected!
        #     print("valid!")
        # else:
        #     # cart empty increment empty counter 
        #     # if empty counter is at "threshold of max empty" post cart empty 
        #     print("empty?")
                
        # print("num valid ", valid_pose)
        # print("num invalid ", invalid_pose)

        # cv2.imshow("Frame", cv_image)
        # cv2.waitKey(3)

if __name__ == "__main__":
    try:
        pose_tracking()
    except rospy.ROSInterruptException:
        pass
