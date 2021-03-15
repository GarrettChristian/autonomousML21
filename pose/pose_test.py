# import numpy as np
# import tensorflow as tf
# from tensorflow import keras
# import openpose as OpenPose
import cv2
import sys
import os
# from  openpose  import pyopenpose as op


# Working off of this tutorial
# https://medium.com/pixel-wise/real-time-pose-estimation-in-webcam-using-openpose-python-2-3-opencv-91af0372c31c
# 
#  and
# https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_python/01_body_from_image.py

#Setting OpenPose parameters
def set_params():
    params = dict()
    params["logging_level"] = 3
    params["output_resolution"] = "-1x-1"
    params["net_resolution"] = "-1x368"
    params["model_pose"] = "BODY_25"
    params["alpha_pose"] = 0.6
    params["scale_gap"] = 0.3
    params["scale_number"] = 1
    params["render_threshold"] = 0.05
    # # If GPU version is built, and multiple GPUs are available, set the ID here
    # params["num_gpu_start"] = 0
    params["disable_blending"] = False
    # Ensure you point to the correct path where models are located
    # params["default_model_folder"] = "./openpose/models" # dir_path + "/../../../models/"
    params["model_folder"] = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/models/', '')
    return params


def main():

    try:
        # Change these variables to point to the correct folder (Release/x64 etc.)
        sys.path.append(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/build/python/', ''))
        print(sys.path)
        # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
        # sys.path.append('/usr/local/python')
        from openpose import pyopenpose as op
    except ImportError as e:
        print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
        raise e

    params = set_params()

    print("setting params")

    #Constructing OpenPose object allocates GPU memory
    # openpose = op(params)
    # Starting OpenPose
    opWrapper = op.WrapperPython(op.ThreadManagerMode.Synchronous)
    opWrapper.configure(params)
    opWrapper.execute()
    
    print("about to capture")

    #ADDED BY ME(RESIZE)
    scale_percent = 10 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
    #Opening OpenCV stream
    stream = cv2.VideoCapture(0)

    font = cv2.FONT_HERSHEY_SIMPLEX

    while True:

        ret,img = stream.read()

        #ADDED BY ME (RESIZE)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

        # Output keypoints and the image with the human skeleton blended on it
        keypoints, output_image = openpose.forward(img, True)

        # Print the human pose keypoints, i.e., a [#people x #keypoints x 3]-dimensional numpy object with the keypoints of all the people on that image
        if len(keypoints) > 0:
            print('Human(s) Pose Estimated!')
            print(keypoints)
        else:
            print('No humans detected!')


        # Display the stream
        cv2.putText(output_image,'OpenPose using Python-OpenCV',(20,30), font, 1,(255,255,255),1,cv2.LINE_AA)

        cv2.imshow('Human Pose Estimation',output_image)

        key = cv2.waitKey(1)

        if key==ord('q'):
            break

    stream.release()
    cv2.destroyAllWindows()

if __name__=="__main__": 
    main() 
