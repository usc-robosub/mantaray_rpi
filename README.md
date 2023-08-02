# mantaray_rpi
ROS package that handles control on the Raspberry Pi.

## Installation

```
cd ~/catkin_ws/src
git clone git@github.com:USCAUVTeam/mantaray_rpi.git
cd ..
catkin_make
```

## Launching

```
roslaunch mantaray_rpi mantaray.launch
```

## Dependencies:
* [dvl-python](https://github.com/waterlinked/dvl-python/tree/master/serial) - A python library containing protocol for interacting with the Water Linked DVL A50

