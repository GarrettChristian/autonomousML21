import roslib
import rospy
import sys
import os
import math
import json
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
        # self.passenger_safe_pub = rospy.Publisher('/passenger_safe', Bool, queue_size=10)
        # self.cart_empty_pub = rospy.Publisher('/cart_empty', Bool, queue_size=10)
        self.cart_empty_safe_pub = rospy.Publisher('/cart_empty_safe', String, queue_size=10)
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
        self.depth_threshold = 1.2 # meters

        # empty check
        self.usual_chair_dist = [1.18, 1.18, 1.19, 1.13, 1.05, 1.13]

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
        cv2.destroyAllWindows()


    ################################
    ### ROS Topics To Publish To ###
    ################################

    # def sendPassengerUnsafe(self):
    #     print("unsafe posted")
    #     self.passenger_safe_pub.publish(False)

    # def sendPassengerSafe(self):
    #     print("safe posted")
    #     self.passenger_safe_pub.publish(True)

    # def sendCartNotEmpty(self):
    #     print("not empty posted")
    #     self.cart_empty_pub.publish(False)

    # def sendCartEmpty(self):
    #     print("empty posted")
    #     self.cart_empty_pub.publish(True)

    def sendCartEmptyPose(self, empty, safe):
        empty_safe_state = json.dumps({'passenger': not empty, 'safe': safe})
        # print(empty_safe_state)
        # print(type(empty_safe_state))
        print(empty_safe_state)
        self.cart_empty_safe_pub.publish(empty_safe_state)


    def get_poses(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # draw the left and right boundaries for valid a cart position
        y, x, _ = np.shape(cv_image)
        left_boundary = 290
        right_boundary = x - 290
        bottom_boundary = 200
        cv2.line(cv_image, (left_boundary, 0), (left_boundary, y), (0, 255, 0), thickness=2)
        cv2.line(cv_image, (right_boundary, 0), (right_boundary, y), (0, 255, 0), thickness=2)
        cv2.line(cv_image, (0, bottom_boundary), (x, bottom_boundary), (0, 255, 0), thickness=2)

        # valid if in range and shoulders are within the boundries
        valid_pose = 0
        # some in cart but shoulders not within boundary
        invalid_pose = 0
        # empty checks four points to determine if cart is opcupado 
        empty = True

        # no one detected
        if self.recorded_depth is not None:

            # flip the zed image right side up
            cv_image = cv2.flip(cv_image, -1)

            # pre process image to remove anything that is not within our threshold (excludes 0 since that is black)
            mask = cv2.inRange(self.recorded_depth, 0.1, self.depth_threshold)
            cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # CHECK IF EMPTY

            # # back of chair
            # left_x = 450
            # center_x = x / 2
            # right_x = x - 450
            # back_y = 450
            # cv2.circle(cv_image, (left_x, back_y), 4, (255, 0, 0), thickness=2)
            # cv2.circle(cv_image, (center_x, back_y), 4, (255, 0, 0), thickness=2)
            # cv2.circle(cv_image, (right_x, back_y), 4, (255, 0, 0), thickness=2)

            # # seat of chair
            # seat_y = 700
            # cv2.circle(cv_image, (left_x, seat_y), 4, (255, 0, 0), thickness=2)
            # cv2.circle(cv_image, (center_x, seat_y), 4, (255, 0, 0), thickness=2)
            # cv2.circle(cv_image, (right_x, seat_y), 4, (255, 0, 0), thickness=2)

            # chair_top_left = self.recorded_depth[(back_y, left_x)]
            # chair_top_center = self.recorded_depth[(back_y, center_x)]
            # chair_top_right = self.recorded_depth[(back_y, right_x)]
            # chair_top_left = self.recorded_depth[(seat_y, left_x)]
            # chair_top_center = self.recorded_depth[(seat_y, center_x)]
            # chair_top_right = self.recorded_depth[(seat_y, right_x)]

            # cv2.putText(cv_image, "dist: {:.2f}".format(chair_top_left), (left_x + 5, back_y + 5), 
            #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA, False)


            left_x = 450
            center_x = x / 2
            right_x = x - 450
            back_y = 450
            seat_y = 700
            num_not_empty = 0
            empty_points = [(left_x, back_y), (center_x, back_y), (right_x, back_y),
                (left_x, seat_y), (center_x, seat_y), (right_x, seat_y)]
            for index in range(len(empty_points)):
                x, y = empty_points[index]
                cv2.circle(cv_image, (x, y), 4, (255, 0, 0), thickness=2)
                chair_distance = self.recorded_depth[(y, x)]
                cv2.putText(cv_image, "dist: {:.2f}".format(chair_distance), (x + 5, y + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA, False)
                usual_dist = self.usual_chair_dist[index]

                if (chair_distance > usual_dist + 0.08 or chair_distance < usual_dist - 0.08):
                    num_not_empty += 1

            if num_not_empty > 1:
                empty = False

            # Process image for poses
            datum = op.Datum()
            datum.cvInputData = cv_image
            self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))

            # Get the poses found
            poseKeypoints = datum.poseKeypoints

            # for those poses found 
            if poseKeypoints is not None and len(poseKeypoints) > 0:
                for pose in poseKeypoints:

                    # get x and y locations of neck and shoulders 
                    x_neck = int(pose[1][0])
                    y_neck = int(pose[1][1])
                    x_left_shoulder = int(pose[2][0])
                    y_left_shoulder = int(pose[2][1])
                    x_right_shoulder = int(pose[5][0])
                    y_right_shoulder = int(pose[5][1])
                    
                    # extract the distance from those recorded locations
                    neck_distance = self.recorded_depth[(y_neck, x_neck)]
                    left_shoulder_dist = self.recorded_depth[(y_left_shoulder, x_left_shoulder)]
                    right_shoulder_dist = self.recorded_depth[(y_right_shoulder, x_right_shoulder)]

                    # take the max as our distance
                    distance = float("-inf")

                    if not math.isnan(neck_distance):
                        distance = max(distance, neck_distance)
                    if not math.isnan(left_shoulder_dist):
                        distance = max(distance, left_shoulder_dist)
                    if not math.isnan(right_shoulder_dist):
                        distance = max(distance, right_shoulder_dist)
                    if distance == float("-inf"):
                        distance = float('NaN')

                    distance_text = "dist: {:.2f}".format(distance)
                    # print(distance_text)
                    

                    in_boundaries = (x_left_shoulder > left_boundary
                        and x_left_shoulder < right_boundary
                        and x_right_shoulder > left_boundary
                        and x_right_shoulder < right_boundary
                        and y_left_shoulder > bottom_boundary
                        and y_right_shoulder > bottom_boundary)

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

        passenger_safe = True
        # check recorded poses that were within the depth
        if ((invalid_pose > 0 or valid_pose > 3 or valid_pose == 0) and not empty):
            # post invalid pose
            passenger_safe = False
        # elif(valid_pose > 0 and not empty):
        #     # valid detected!
        #     passenger_safe = True

        # if not empty:
        #     self.sendCartNotEmpty()
        # else:
        #     self.sendCartEmpty()

        # passenger 
        # passenger_safe_pub = "{'passenger': {0}, 'safe': {1}}".format(empty, passenger_safe)

        self.sendCartEmptyPose(empty, passenger_safe)

        # cv2.imshow("Frame", datum.cvOutputData)
        cv2.imshow("Frame", cv_image)
        cv2.waitKey(3)  

    def classify_depth(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # flip
        cv_image = cv2.flip(cv_image, -1)

        # remove areas that are farther than our cart distance threshold
        mask = cv2.inRange(cv_image, 0.0, self.depth_threshold)
        cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # smooth the image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cv_image = cv2.dilate(cv_image, kernel, 1)

        self.recorded_depth = cv_image

        # cv2.imshow("Frame", cv_image)
        # cv2.waitKey(3)

if __name__ == "__main__":
    try:
        pose_tracking()
    except rospy.ROSInterruptException:
        pass
