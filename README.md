# autonomous ML Garrett Christian
repo for me to test things related to my autonomous vehicles class

## Steps taken to install and set up my VM
- Installed the Ubuntu Bionic Linux Mint 19.3 “Tricia” Cinnamon from here: https://linuxmint.com/edition.php?id=274 using the Harvard School of Engineering Mirror
- Added the image to my copy of VMWare Fusion.
- Booted the image clicked the disk to install
- Fixed the resolution to be
- Took a snapshot after installation
- installed visual studio code
- sudo apt-get update
- sudo apt install vim
- sudo apt install git
- Followed the steps here to add git password to my config: https://stackoverflow.com/questions/35942754/how-to-save-username-and-password-in-git NOTE THIS IS NOT SECURE (but it is much easier)
- git config --global user.email "you@example.com"
- git config --global user.name "Your Name"
- Followed the steps listed here: https://w3.cs.jmu.edu/molloykp/teaching/cs445/cs445_2020Fall/resources/workStationConfig.php from professor molloys Machine Learning class restated below
- sudo apt-get install python3-distutils python3-pip python3-dev python3-venv
- cd # places you in your home directory
- python3 -m venv csAuto_venv
- source $HOME/csAuto_venv/bin/activate
- pip install --upgrade pip
- pip install wheel
- pip install setuptools
- pip install numpy
- pip install matplotlib
- pip install notebook
- pip install pandas
- pip install PyQt5
- pip install seaborn
- pip install sklearn
- pip install tensorflow
- pip install Keras
- pip install tensorflow_datasets
- pip install nbgrader
- jupyter nbextension install --sys-prefix --py nbgrader
- jupyter nbextension enable --user validate_assignment/main --section=notebook
- jupyter serverextension enable --user nbgrader.server_extensions.validate_assignment
- added an alias to my .bash_aliases to start my virtual environment: alias auto="source $HOME/csAuto_venv/bin/activate"
- took another snapshot after performing set up

## steps to run Pose Tracking
- sudo apt install npm -g n
- sudo apt install node.js 
- sudo n stable
- npm i
- npm start
- got an error running react apps on ubuntu used: https://stackoverflow.com/questions/55763428/react-native-error-enospc-system-limit-for-number-of-file-watchers-reached to resolve
- echo fs.inotify.max_user_watches=524288 | sudo tee -a /etc/sysctl.conf
- sudo sysctl -p

## steps to run speech 
- pip install SpeechRecognition
- sudo apt-get install python3 python3-all-dev python3-pip build-essential swig git libpulse-dev libasound2-dev
- pip3 install pocketsphinx
- sudo apt-get install portaudio19-dev python-pyaudio
- pip install PyAudio

## setting up ros
- https://www.programmersought.com/article/79985355625/
- sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
- sudo vim /etc/apt/sources.list.d/ros-latest.list
- change tricia to bionic
- sudo apt-get update
- sudo aptitude install ros-melodic-desktop-full
- source /opt/ros/melodic/setup.bash
- mkdir -p ~/catkin_ws/src
- cd ~/catkin_ws/
- catkin_make
- source devel/setup.bash

## running bag files
- source /opt/ros/melodic/setup.bash
- roscore
- new terminal
- rosrun rviz rviz
- new terminal
- rosbag play <bagfile name>
- rosbag play -l <bagfile name> (loop)

## setting up a python 2.7 env
- sudo apt install virtualenv
- virtualenv -p /usr/bin/python auto_venv
- source auto_venv/bin/activate
- python -m pip install --upgrade pip
- pip install wheel
- pip install setuptools
- pip install numpy
- pip install matplotlib
- pip install notebook
- pip install pandas
- pip install seaborn
- pip install sklearn
- pip install tensorflow
- pip install Keras
- pip install tensorflow_datasets
- pip install SpeechRecognition
- pip install pocketsphinx
- pip install PyAudio
- pip2 install opencv-python==4.2.0.32
- pip install opencv-contrib-python==4.2.0.32

- sudo apt-get install cmake-qt-gui
- sudo apt-get install autoconf automake libtool curl make g++ unzip -y 
- sudo apt-get install qtbase5-de
- pip install catkin_pkg
- mkdir -p ~/catkin_ws/src
- cd ~/catkin_ws/
- catkin_make

## Changing to python2 open pose
- https://medium.com/@erica.z.zheng/installing-openpose-on-ubuntu-18-04-cuda-10-ebb371cf3442
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
- GPU_MODE set to CPU_ONLY (as recommended for MacOS)
- BUILD_PYTHON set to true
- PYTHON_EXECUTABLE=/usr/bin/python2.7
- PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7m.so”
- to find your executable python, import sys, print(sys.executable)
- to find library print(sys.path)
- cmake generate
- cd build then make -j`nproc`

## attempting to run old pose tracking again
- pip install rospkg
- pip install python-vlc==3.0.7110

## zed wrapper!!!
- http://wiki.ros.org/zed-ros-wrapper
- https://github.com/stereolabs/zed-ros-wrapper/
- https://www.stereolabs.com/developers/
- needs cuda :(



## installing cuda steps
- Following this guide https://mrprajesh.blogspot.com/2018/11/install-cuda-10-on-linux-mint-19-or.html
- Getting cuDNN (10.2) from: https://developer.nvidia.com/rdp/cudnn-download
- Cuda 10.2 from https://developer.nvidia.com/cuda-10.2-download-archive
- Cuda tool kit installation instructions https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
- sudo apt-get install build-essential dkms
- sudo apt-get install freeglut3 freeglut3-dev libxi-dev libxmu-dev
- Down load deb file from the above 
- sudo dpkg -i cuda-repo-ubuntu1804-10-0-local-10.0.130-410.48_1.0-1_amd64.deb
- sudo apt-key add /var/cuda-repo-10-0-local-10.0.130-410.48/7fa2af80.pub
- sudo apt-get update
- sudo apt-get install cuda
- "cuda-install-samples-10.0.sh ~/install" installs sample files
- cd into the folder "NVIDIA_CUDA-10.0_Samples"
- make -j2
- Add the "PATH=$PATH:/usr/local/cuda/bin"  to your "~/.bashrc" 
- run "deviceQuery" to verify the cuda installation.  And do the post installation formalities
- sudo apt-get install g++ freeglut3-dev build-essential libx11-dev \
    libxmu-dev libxi-dev libglu1-mesa libglu1-mesa-dev





x## installing cuda steps
- Following this guide https://mrprajesh.blogspot.com/2018/11/install-cuda-10-on-linux-mint-19-or.html
- Getting cuDNN (10.2) from: https://developer.nvidia.com/rdp/cudnn-download
- Cuda 10.2 from https://developer.nvidia.com/cuda-10.2-download-archive
- Cuda tool kit installation instructions https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
- wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
- sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
- wget https://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
- sudo dpkg -i cuda-repo-ubuntu1804-10-2-local-10.2.89-440.33.01_1.0-1_amd64.deb
- sudo apt-key add /var/cuda-repo-10-2-local-10.2.89-440.33.01/7fa2af80.pub
- sudo apt-get update
- sudo apt-get -y install cuda
- sudo vim /etc/profile.d/cuda.sh
- export PATH=$PATH:/usr/local/cuda/bin
- export CUDADIR=/usr/local/cuda
- sudo chmod +x /etc/profile.d/cuda.sh
- sudo vim /etc/ld.so.conf.d/cuda.conf
- /usr/local/cuda/lib64
- verify installation: nvcc --version

## Installing cudnn
- Following https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html
- Go to: https://developer.nvidia.com/rdp/cudnn-archive
- install the tar only!!!! Library for linux x86 8.0.5.39
- tar -xzvf cudnn-x.x-linux-x64-v8.x.x.x.tgz
- sudo cp cuda/include/cudnn*.h /usr/local/cuda/include 
- sudo cp -P cuda/lib64/libcudnn* /usr/local/cuda/lib64 
- sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*






## verifying cuda
- cd to /usr/local/cuda/samples
- sudo make -k
- cd /usr/local/cuda/samples/bin/x86_64/linux/release
- ./deviceQuery

## verifying cudnn
- cp -r /usr/src/cudnn_samples_v8/ $HOME
- cd  $HOME/cudnn_samples_v8/mnistCUDNN
- make clean && make
- ./mnistCUDNN
- should see: Test passed!

## zed wrapper
- install zed sdk for cuda 10.2
- Forbidden on school server :( 

## Fixing open pose 
- usual with new cmake version 3.15
- exec /usr/bin/python
- lib /usr/bin/python2.7
- build python yes 

## Installing zed wrapper
- Grabbed zed sdk install when at home brought it with ssd
