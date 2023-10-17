#include "include.h"
#include "fsm.h"

ros::Publisher* thruster_pubs;

void initializeThrusters() {
    ROS_INFO("Initializing thrusters");
    std::this_thread::sleep_for(std::chrono::milliseconds(THRUSTER_INIT_DELAY));
    ROS_INFO("Thrusters initialized");
}


int main(int argc, char **argv){
    ROS_INFO("Starting mantaray_control");
    initializeThrusters();

    ros::init(argc, argv, "mantaray_control");
    ros::NodeHandle nh;

    thruster_pubs = new ros::Publisher[THRUSTER_COUNT];
    for(int i = 0; i < THRUSTER_COUNT; i++){
        std::string topic_name = SUB_NAME + "/thruster/" + std::to_string(i) + "/input";
        thruster_pubs[i] = nh.advertise<std_msgs::Float64>(topic_name, 1);
    }

    ros::Rate loop_rate(10);

    FSM fsm;
    fsm.setState(1);

    while(ros::ok()){
        fsm.run(100);
        std_msgs::Float64 msg;
        msg.data = 0.0;
        for(int i = 0; i < THRUSTER_COUNT; i++){
            thruster_pubs[i].publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}