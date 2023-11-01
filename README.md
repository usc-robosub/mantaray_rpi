# mantaray_rpi
ROS package that handles control on the Raspberry Pi.

## Installation

To install our mantaray library:

```
cd ~/catkin_ws/src
git clone git@github.com:USCAUVTeam/mantaray_rpi.git
cd mantaray_rpi
sudo bash install_deps.sh
cd ..

catkin build -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```


## Launching

```
roslaunch mantaray_rpi mantaray.launch
```

## Dependencies:
* [serialib](https://github.com/imabot2/serialib) - Cross-platform lightweight serial library for C++
* [NGIMU C++ Example](https://github.com/xioTechnologies/NGIMU-C-Cpp-Example) - A wrapper for decoding NGIMU messages
* [OSC99](https://github.com/xioTechnologies/OSC99) - Used for decoding OSC packets
* [robot-localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html) - ROS package for state estimation including EKF
* [pid](https://wiki.ros.org/pid) - Implementation of a Proportional-Integral-Derivative controller which includes many tuning tools
* [libeigen](https://gitlab.com/libeigen/eigen) - Fast C++ linear algebra library
* [boost](https://www.boost.org/) - Multipurpose C++ libraries
* [control-toolbox](https://github.com/ethz-adrl/control-toolbox) - Efficient C++ library for control, estimation, optimization and motion planning in robotics.