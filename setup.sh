#!/bin/bash

# Only run once
python3 -m venv piper_env
source piper_env/bin/activate

pip3 install python-can
pip3 install scipy
pip3 install piper_sdk

sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager

git submodule init
git submodule update