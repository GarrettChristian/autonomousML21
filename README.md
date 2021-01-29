# autonomousMLrepo1
repo for me to test things related to my autonomous vehicles class

## Steps taken to set and install my VM
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
- Followed the steps listed here: https://w3.cs.jmu.edu/molloykp/teaching/cs445/cs445_2020Fall/resources/workStationConfig.php from professor molloys Machine Learning class restated below
- sudo apt-get install python3-distutils python3-pip python3-dev python3-venv
- cd # places you in your home directory
- python3 -m venv cs445_venv
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
- added an alias to start my virtual environment: alias auto="source $HOME/csAuto_venv/bin/activate"
- took another snapshot after performing set up
