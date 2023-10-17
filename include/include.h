#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <chrono>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define THRUSTER_COUNT 8
#define THRUSTER_INIT_DELAY 0
const std::string SUB_NAME = "mantaray";