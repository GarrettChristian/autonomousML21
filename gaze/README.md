# Gaze Tests

gaze360:

- http://gaze360.csail.mit.edu/index.php
- https://github.com/erkil1452/gaze360/tree/master/code
- https://colab.research.google.com/drive/1AUvmhpHklM9BNt0Mn5DjSo3JRuqKkU4y


Steps I have take so far
-  git clone -q --depth 1 https://github.com/facebookresearch/DensePose.git
- cd DensePose/ && pip install -q -r requirements.txt
- following https://github.com/Johnqczhang/densepose_installation/blob/master/README.md#requirements
- pip install cython matplotlib
- pip install pydot future networkx
- pip install torch torchvision (https://pytorch.org/get-started/locally/#linux-pip)
- python -c 'from caffe2.python import core' 2>/dev/null && echo "Success" || echo "Failure"

So it was at this point when I realized that I can't use cuda with VMware Fusion since my gpu doesn't carry over

## working on mac

- following this guide https://towardsdatascience.com/real-time-eye-tracking-using-opencv-and-dlib-b504ca724ac6
- pip install opencv-python
- pip install dlib

## just gonna drop what I looked at if anyone wants to reapproach this later 

- pip install opencv-python
- pip install dlib
- good read https://towardsdatascience.com/real-time-eye-tracking-using-opencv-and-dlib-b504ca724ac6
- used https://github.com/antoinelame/GazeTracking
- the above is very human readable and can be easily modified to do whatever you might need it to do
- (that's what i did)
