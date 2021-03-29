# https://github.com/stereolabs/zed-ros-wrapper/issues/371
# https://github.com/stereolabs/zed-ros-wrapper/issues/348

import rospy
import sys
import os
import subprocess
# sys.path.append(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/zed-ros-wrapper/zed_interfaces/srv', ''))
# from node.srv import start_svo_recording
# from zed_interfaces import start_svo_recording
from zed_interfaces.srv import *


def main():
    rospy.wait_for_service('/zed/zed_node/start_svo_recording')

    try:
        start_recording = rospy.ServiceProxy('/zed/zed_node/start_svo_recording', start_svo_recording)
        resp1 = start_recording('/home/jacart/catkin_ws/src/autonomousMLgc/zedVideo/new_svo.svo')
        print(resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")

if __name__ == "__main__":
    main()
