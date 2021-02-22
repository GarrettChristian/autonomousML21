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