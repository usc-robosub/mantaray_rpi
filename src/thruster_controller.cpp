#include "ros/ros.h"
#include "mantaray_rpi/FloatStamped.h"
#include "std_msgs/Float64.h"

class Thruster
{
    private:
        // float curThrust(0);
        // float targetThrust(0);
        // bool initialized(false);
        float curThrust = 0;
        float targetThrust = 0;
        bool initialized = false;
        int thrusterNum;
    public:
        Thruster(int _thrusterNum) {
            thrusterNum = _thrusterNum;
        }
        void initThruster(ros::NodeHandle &n);
        void updateThruster(int num) {
            curThrust = num;
        }
};

void thrusterCallback(const std_msgs::Float64::ConstPtr& data) {
    targetThrust = data->data;
}

void Thruster::initThruster(ros::NodeHandle &n) {
    auto _thrusterCallback = [](const std_msgs::Float64::ConstPtr& data) {thrusterCallback(data)}
    ros::Subscriber _ = n.subscribe("thruster_0", 10, thrusterCallback);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "thruster_controller");
    ros::NodeHandle n;

    Thruster test(0);
    test.initThruster(n);
}