#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include "serialib.h"
#include <string>
#include "Osc99.h"
#include "NgimuReceive.h"
#include <thread>


#define PORT "/dev/ttyACM0"

ros::Publisher imu_pub;

float quat[4];
float gyro[3];
float accel[3];
float mag[3];
float baro = 0;

serialib serial;

void serial_imu_callback() {
    // Process each received byte
    while(true) {
        while (serial.available() > 0) {
            try
            {
                uint8_t* buff;
                serial.readBytes(buff, 1, 1000, 0);
                NgimuReceiveProcessSerialByte(*buff);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
        }
    }
}

// This function is called each time there is a receive error
extern void ngimuReceiveErrorCallback(const char* const errorMessage) {
    std::cout << (errorMessage)<< std::endl;
    std::cout << ("\r\n")<< std::endl;
}

// This function is called each time a "/sensors" message is received
void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {

    gyro[0] = ngimuSensors.gyroscopeX;
    gyro[1] = ngimuSensors.gyroscopeY;
    gyro[2] = ngimuSensors.gyroscopeZ;

    accel[0] = ngimuSensors.accelerometerX;
    accel[1] = ngimuSensors.accelerometerY;
    accel[2] = ngimuSensors.accelerometerZ;

    mag[0] = ngimuSensors.magnetometerX;
    mag[1] = ngimuSensors.magnetometerY;
    mag[2] = ngimuSensors.magnetometerZ;

    baro = ngimuSensors.barometer;
}

// This function is called each time a "/quaternion" message is received
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {
    quat[0] = ngimuQuaternion.w;
    quat[1] = ngimuQuaternion.x;
    quat[2] = ngimuQuaternion.y;
    quat[3] = ngimuQuaternion.z;

}

// This function is called each time a "/euler" message is received.
void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
    
}

int main(int argc, char **argv) {

    // Connection to serial port
    serial.openDevice(PORT, 115200);

    
    
    ros::init(argc, argv, "imu_publisher");

    std::thread t1 (serial_imu_callback);

    ros::NodeHandle n;

    imu_pub = n.advertise<sensor_msgs::Imu>("mantaray/imu", 1000);

    ros::Rate loop_rate(100);

    


    // Initialise NGIMU receive module
    NgimuReceiveInitialise();

    // Assign NGIMU receive callback functions
    NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
    NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
    NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
    NgimuReceiveSetEulerCallback(ngimuEulerCallback);

    while(ros::ok()) {
        
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();

        msg.orientation.w = quat[0];
        msg.orientation.x = quat[1];
        msg.orientation.y = quat[2];
        msg.orientation.z = quat[3];

        msg.angular_velocity.x = gyro[0];
        msg.angular_velocity.y = gyro[1];
        msg.angular_velocity.z = gyro[2];

        msg.linear_acceleration.x = accel[0];
        msg.linear_acceleration.y = accel[1];
        msg.linear_acceleration.z = accel[2];
        
        imu_pub.publish(msg);
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    serial.closeDevice();
}