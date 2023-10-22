#!/bin/bash

sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-pid
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev

cd scripts && pip3 install -r requirements.txt

cd ../..
git clone https://github.com/ANYbotics/kindr.git
git clone https://github.com/ethz-adrl/control-toolbox.git