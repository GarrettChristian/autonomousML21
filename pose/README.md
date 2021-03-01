# Tuesday Demo

planning:

- https://towardsdatascience.com/real-time-head-pose-estimation-in-python-e52db1bc606a
- 


## For installation following mac
- https://robinreni96.github.io/computervision/Python-Openpose-Installation/
- https://www.thomasvanhoey.com/post/installing-openpose-on-mac-october-2020-version/ (mac)
- https://medium.com/pixel-wise/real-time-pose-estimation-in-webcam-using-openpose-python-2-3-opencv-91af0372c31c seemed like a good tutorial for linux
- https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/1_prerequisites.md#mac-os-prerequisites
- git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
- cd openpose/
- git submodule update --init --recursive --remote
- brew install --cask cmake
- brew install caffe (this one was big...)
- move into openpose/scripts/osx/install_bre.sh bash scripts/osx/install_brew.sh
- bash scripts/osx/install_deps.sh

- cd build/
- make -j`nproc` or mac make -j`sysctl -n hw.logicalcpu`

- can't get around the free invalid pointer error thinking about switching to vm and trying to install tensorflow with cpu???

---

## trying on vm https://medium.com/@erica.z.zheng/installing-openpose-on-ubuntu-18-04-cuda-10-ebb371cf3442
- sudo apt-get install cmake-qt-gui
- sudo apt-get install autoconf automake libtool curl make g++ unzip -y 
- sudo apt-get install qtbase5-dev
- Go to OpenPose root folder sudo bash ./scripts/ubuntu/install_deps.sh 
- sudo apt install caffe-cuda
- git clone https://github.com/google/protobuf.git
- cd protobuf
- ./autogen.sh
- ./configure
- make
- make check
- sudo make install
- sudo ldconfig
- sudo apt-get install libopencv-dev
- cd openpose 
- cmake gui configure 
<!-- - BUILD_CAFFE set to false -->
- GPU_MODE set to CPU_ONLY (as recommended for MacOS)
<!-- - Caffe_INCLUDE_DIRS set to /usr/local/include/caffe
- Caffe_LIBS set to /usr/local/lib/libcaffe.dylib -->
- BUILD_PYTHON set to true
- cmake generate
<!-- - cd /build then make -->
- cd build then make -j`nproc`

## runing examples
- ./build/examples/openpose/openpose.bin
- ./build/examples/openpose/openpose.bin --image_dir ../input --write_images ../output
- python pose2.py --image_path chan.jpg


## restaring
- sudo apt-get install autoconf automake libtool curl make g++ unzip -y
- sudo apt-get install libgoogle-glog-dev

## trying to replicate pose tracking with just mriduls method
- https://github.com/stereolabs/zed-openpose a specific zed open pose implementation
- https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md explanation of key point output 2 & 5 are the {0: u'Nose', 1: u'Neck', 2: u'RShoulder', 3: u'RElbow', 4: u'RWrist', 5: u'LShoulder', 6: u'LElbow', 7: u'LWrist', 8: u'MidHip', 9: u'RHip', 10: u'RKnee', 11: u'RAnkle', 12: u'LHip', 13: u'LKnee', 14: u'LAnkle', 15: u'REye', 16: u'LEye', 17: u'REar', 18: u'LEar', 19: u'LBigToe', 20: u'LSmallToe', 21: u'LHeel', 22: u'RBigToe', 23: u'RSmallToe', 24: u'RHeel', 25: u'Background'}
- 

