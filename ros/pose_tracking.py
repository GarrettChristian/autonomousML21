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

        # subscribing to depth image and color version
        rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image, self.get_poses)
        rospy.Subscriber("/zed/zed_node/depth/depth_registered",Image, self.classify_depth)
        
        # publish cart empty
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

        # depth (set by classify_depth)
        self.recorded_depth = None

        # depth threshold (filter cv image to be black beyond this distance)
        self.depth_threshold = 1.2 # meters

        # chair threshold marks as not empty if a key point is +- this value
        self.chair_threshold = 0.08

        # the X and Y of key points (set once camera feed begins)
        self.key_points = None

        # get default distance to chair on startup (set once camera feed begins)
        self.usual_chair_dist = None

        # The number of consecutive invalid positions recorded
        self.num_consecutive_invalid = 0

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
        cv2.destroyAllWindows()

    def sendCartEmptyPose(self, empty, safe):
        '''
        Ros topic passenger is present and passenger is safe
        '''
        empty_safe_state = json.dumps({'passenger': not empty, 'safe': safe})
        print(empty_safe_state)
        self.cart_empty_safe_pub.publish(empty_safe_state)

    def get_usual_dist(self, cv_image):
        '''
        Sets the distances for the six chair points at launch
        '''
        # default values
        usual_dist = [1.18, 1.18, 1.19, 1.13, 1.05, 1.13]

        # get the distances of key points
        for index in range(len(self.key_points)):
            point_x, point_y = self.key_points[index]
            if (self.recorded_depth[(point_y, point_x)] > 0):
                usual_dist[index] = self.recorded_depth[(point_y, point_x)]
        return usual_dist

    def get_poses(self, data):
        '''
        Callback for left color zed image runs open pose publishes to empty safe
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # draw the left and right boundaries for valid a cart position
        # NOTE 290 hardcoded # GREEN
        y, x, _ = np.shape(cv_image)
        left_boundary = 290
        right_boundary = x - 290
        bottom_boundary = 200
        cv2.line(cv_image, (left_boundary, 0), (left_boundary, y), (0, 255, 0), thickness=2)
        cv2.line(cv_image, (right_boundary, 0), (right_boundary, y), (0, 255, 0), thickness=2)
        cv2.line(cv_image, (0, bottom_boundary), (x, bottom_boundary), (0, 255, 0), thickness=2)

        # valid if in range and shoulders are within the boundaries
        valid_pose = 0
        # passenger in cart but shoulders not within boundary
        invalid_pose = 0
        # empty checks four points to determine if a passenger is present 
        empty = True

        if self.recorded_depth is not None:

            # set the locations of the chair key points 
            if(self.key_points is None):
                left_x = 450
                center_x = x / 2
                right_x = x - 450
                back_y = 450
                seat_y = 700
                self.key_points = [(left_x, back_y), (center_x, back_y), (right_x, back_y),
                (left_x, seat_y), (center_x, seat_y), (right_x, seat_y)]
                print(self.key_points)

            # get the default chair distances if not already done
            if(self.usual_chair_dist is None):
                self.usual_chair_dist = self.get_usual_dist(cv_image)
                print(self.usual_chair_dist)

            # flip the zed image right side up
            cv_image = cv2.flip(cv_image, -1)

            # pre process image to remove anything that is not within our threshold (excludes 0 since that is black)
            mask = cv2.inRange(self.recorded_depth, 0.1, self.depth_threshold)
            cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            # how many points are NOT empty
            num_not_empty = 0

            # determine how many points are NOT empty
            for index in range(len(self.key_points)):
                # get x,y of empty points
                point_x, point_y = self.key_points[index]

                # get the distance set on startup to compare to the actual distance
                usual_dist = self.usual_chair_dist[index]
                chair_distance = self.recorded_depth[(point_y, point_x)]

                color = (255, 0, 0) # BLUE

                # if the current distance is different than the usual distance (plus a threshold)
                # increment the NOT empty spaces
                # also set color to red instead of blue
                # NOTE threshold hardcoded
                if (chair_distance > usual_dist + self.chair_threshold \
                    or chair_distance < usual_dist - self.chair_threshold):
                    num_not_empty += 1
                    color = (0, 0, 255) # RED


                # draw circle and print info at that point
                cv2.circle(cv_image, (point_x, point_y), 4, color, thickness=2)
                cv2.putText(cv_image, "dist: {:.2f}".format(chair_distance), (point_x + 5, point_y + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA, False)

            # set empty to false if a point is NOT empty (self-explanitory but i'm commenting literally everything)
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

                    distance = float("-inf")

                    # Set distance as max of neck, left, and right shoulders, or defualt to nan
                    if not math.isnan(neck_distance):
                        distance = max(distance, neck_distance)
                    if not math.isnan(left_shoulder_dist):
                        distance = max(distance, left_shoulder_dist)
                    if not math.isnan(right_shoulder_dist):
                        distance = max(distance, right_shoulder_dist)
                    if distance == float("-inf"):
                        distance = float('NaN')

                    # define what is considered in the boundaries
                    in_boundaries = (x_left_shoulder > left_boundary
                        and x_left_shoulder < right_boundary
                        and x_right_shoulder > left_boundary
                        and x_right_shoulder < right_boundary
                        and y_left_shoulder > bottom_boundary
                        and y_right_shoulder > bottom_boundary)

                    # define distance text
                    distance_text = "dist: {:.2f}".format(distance)
                    
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

        passenger_safe = True
        
        # check recorded poses that were within the depth
        if ((invalid_pose > 0 or valid_pose > 3 or valid_pose == 0) and not empty):
            # post invalid pose
            self.num_consecutive_invalid += 1
            if (self.num_consecutive_invalid > 5):
                passenger_safe = False
        else:
            self.num_consecutive_invalid = 0


        self.sendCartEmptyPose(empty, passenger_safe)

        # cv2.imshow("Frame", datum.cvOutputData)
        cv2.imshow("Frame", cv_image)
        cv2.waitKey(3)  

    def classify_depth(self, data):
        '''
        Callback for depth zed image
        Removes anything farther than the threshold
        '''
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
