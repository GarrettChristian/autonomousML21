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