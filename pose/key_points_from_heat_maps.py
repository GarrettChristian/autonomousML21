# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform
import argparse
import numpy as np

try:
    sys.path.append(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/build/python', ''))
    from openpose import pyopenpose as op

    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_path", default="../../../examples/media/COCO_val2014_000000000294.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
    args = parser.parse_known_args()

    # Load image
    imageToProcess = cv2.imread(args[0].image_path)
    print("here, ", args[0].image_path)

    def get_sample_heatmaps():
        # These parameters are globally set. You need to unset variables set here if you have a new OpenPose object. See *
        params = dict()
        params["model_folder"] = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/models/', '')
        params["heatmaps_add_parts"] = True
        params["heatmaps_add_bkg"] = True
        params["heatmaps_add_PAFs"] = True
        params["heatmaps_scale"] = 3
        params["upsampling_ratio"] = 1
        params["body"] = 1

        # Starting OpenPose
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()

        # Process Image and get heatmap
        datum = op.Datum()
        imageToProcess = cv2.imread(args[0].image_path)
        datum.cvInputData = imageToProcess
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        poseHeatMaps = datum.poseHeatMaps.copy()
        opWrapper.stop()

        return poseHeatMaps

    # Get Heatmap
    poseHeatMaps = get_sample_heatmaps()
    print("here2")

    # Starting OpenPose
    params = dict()
    params["model_folder"] = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/models/', '')
    params["body"] = 2  # Disable OP Network
    params["upsampling_ratio"] = 0 # * Unset this variable
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    print("here3")

    # Pass Heatmap and Run OP
    datum = op.Datum()
    datum.cvInputData = imageToProcess
    datum.poseNetOutput = poseHeatMaps
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))

    # Display Image
    print("Body keypoints: \n" + str(datum.poseKeypoints))
    cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", datum.cvOutputData)
    cv2.waitKey(0)
except Exception as e:
    print(e)
    sys.exit(-1)