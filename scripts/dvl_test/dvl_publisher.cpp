#include "dvl_publisher.h"
#include "serialib.h"
#include "vector"
#include "sstream"
#include "string"
// #include "ros/ros.h"
// #include <ros/console.h>
// #include "std_msgs/String.h"
#include <unistd.h>
#define PORT "/dev/ttyUSB0"

// constructor
DVLPUB::DVLPUB(){

}
// destructor
DVLPUB::~DVLPUB(){

}

// reads data from serial connection while open
int DVLPUB::readDevice(){
    // serialib serial;
    char dvl_test = this->serial.openDevice(PORT, 115200);
    if (dvl_test!=1) {
        return dvl_test;
    }
    std::cout << "Opened Device" << std::endl;
    while (true){
        std::cout << "Device Available" << std::endl;
        while (true){
            char* temp = new char[50];
            serial.readBytes(temp,49);
            std::cout << "temp: " << temp << std::endl;
            delete[] temp;
            // std::cout << "ur a failure" << std::endl;
            sleep(1);
        }
        while (serial.available() > 0){
            try{
                // char* buff = new char[256]; // how large can report be
                // does buff contain both WRZ and WRU reports???
                // int report = checkReport(buff);
                // serial.readBytes(buff,499,500);
                // parse when hit wrz or wrp
                // parseData(buff);
                std::cout << "reportType(): ";
                int report = reportType();
                std::cout << report << std::endl;
                if (report == 1){
                    parseWRZ();
                    std::cout << "test 1" << std::endl;
                    parseData();
                    setWRZ(); // need to rewrite
                    printWRZ();
                }
                else if (report == 2){
                    parseWRP();
                    std::cout << "test 2" << std::endl;
                    parseData();
                    setWRP(); // need to rewrite
                    printWRP();
                }
                ss.clear();
                // send data here before next loop overwrites
                wrp.clear();
                wrz.clear();
                // delete[] buff;
            } catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        sleep(1);
        }
    }
    return 1;
}

int DVLPUB::reportType(){
    // todo
    // readChar() until wrz or wrp
    // if wrz return 1
    // if wrp return 2
    while(true) {
        // char* letter = new char[4];
        char* letter = new char;
        while (*letter != 'w'){
            this->serial.readChar(letter); 
        }
        this->serial.readChar(letter); // next letter is gonna be the same bc r
        if (*letter != 'r') {
            continue;
        } 
        this->serial.readChar(letter); // z or p
        if (*letter == 'z') {
            return 1;
        } else if (*letter == 'p') {
            return 2;
        }
        delete letter;
    }
}

void DVLPUB::parseWRZ(){
    int count = 0;
    wrz.push_back("wrz");
    char* letter = new char;
    while (count < 11){
        if (this->serial.readChar(letter) == 1){
            if (strchr(letter,',') != nullptr){
                count++;
            }
            this->ss << letter;
        } else{
            std::cout << "Incorrect Data read from DVL" << std::endl;
        }
    }
    // read last char (status)
    if (this->serial.readChar(letter) == 1){
            if (strchr(letter,',') != nullptr){
                count++;
            }
            this->ss << letter;
            this->ss << ',';
        } else{
            std::cout << "Incorrect Data read from DVL" << std::endl;
    }
    // read last 3 chars for checksum
    for (int i=0;i<4;i++){
        if (this->serial.readChar(letter) == 1){
            this->ss << letter;
        } else{
            std::cout << "Incorrect Data read from DVL" << std::endl;
        }
    }
    delete letter;
}

void DVLPUB::parseWRP(){
    int count = 0;
    wrp.push_back("wrp");
    char* letter = new char;
    while (count < 9){
        if (this->serial.readChar(letter) == 1){
            if (strchr(letter,',') != nullptr){
                count++;
            }
            this->ss << letter;
        } else{
            std::cout << "Incorrect Data read from DVL" << std::endl;
        }
    }
    // read last char (status)
    if (this->serial.readChar(letter) == 1){
            if (strchr(letter,',') != nullptr){
                count++;
            }
            this->ss << letter;
            this->ss << ',';
        } else{
            std::cout << "Incorrect Data read from DVL" << std::endl;
    }
    // read 3 chars for checksum
    for (int i=0;i<4;i++){
        if (this->serial.readChar(letter) == 1){
            this->ss << letter;
        } else{
            std::cout << "Incorrect Data read from DVL" << std::endl;
        }
    }
    delete letter;
}

void DVLPUB::parseData(){
    // std::stringstream ss(buff);
    std::string val;
    int report = 0;
    while (getline(this->ss,val,',')){
        if (val == "wrz"){
            report = 1;
        }
        else if (val == "wru"){
            report = 2;
        }
        if (report == 1){
            wrz.push_back(val);
        }
        else if (report == 2){
            wrp.push_back(val);
        }
    }
}

// parse WZU Velocity report from data vector
void DVLPUB::setWRZ(){
    try{
        vx = std::stod(wrz.at(1));
        vy = std::stod(wrz.at(2));
        vz = std::stod(wrz.at(3));
        valid = wrz.at(4);
        altitude = std::stod(wrz.at(5));
        fom = std::stod(wrz.at(6));
        covariance = wrz.at(7);
        time_of_validity = std::stod(wrz.at(8));
        time_of_transmission = std::stod(wrz.at(9));
        time = std::stod(wrz.at(10));
        status = wrz.at(11);
        checksum = wrz.at(12);
    } catch (const std::out_of_range& e) {
        std::cerr << "Out of range exception in setWRZ(): " << e.what() << std::endl;
    }
}
// parse WRP Transducer report from data vector
void DVLPUB::setWRP(){
    try{
        x = std::stod(wrp.at(1));
        y = std::stod(wrp.at(2));
        z = std::stod(wrp.at(3));
        pos_std = std::stod(wrp.at(4));
        roll = std::stod(wrp.at(5));
        pitch = std::stod(wrp.at(6));
        yaw = std::stod(wrp.at(7));
        status = wrp.at(8);
        checksum = wrp.at(9);
    } catch (const std::out_of_range& e) {
        std::cerr << "Out of range exception in setWRP(): " << e.what() << std::endl;
    }
    
}

void DVLPUB::printWRZ(){
    std::cout << "WRZ Report: " << std::endl;
    std::cout << "vx: " << vx << std::endl;
    std::cout << "vy: " << vx << std::endl;
    std::cout << "vz: " << vz << std::endl;
    std::cout << "valid: " << valid << std::endl;
    std::cout << "altitude: " << altitude << std::endl;
    std::cout << "fom: " << fom << std::endl;
    std::cout << "covariance: " << covariance << std::endl;
    std::cout << "time_of_validity: " << time_of_validity << std::endl;
    std::cout << "time_of_transmission: " << time_of_transmission << std::endl;
    std::cout << "timetime: " << time << std::endl;
    std::cout << "End of WRZ Report" << std::endl;
}

void DVLPUB::printWRP(){
    std::cout << "WRP Report" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << x << std::endl;
    std::cout << "z: " << z << std::endl;
    std::cout << "pos_std: " << pos_std << std::endl;
    std::cout << "roll: " << x << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "yaw: " << yaw << std::endl;
    std::cout << "End of WRP Report" << std::endl;
}

int main(int argc, char* argv[]){
    // ros stuff here
    // ros::init(argc, argv, "dvl_publisher");
    // ros::NodeHandle n;
    // // first arg is topic, second arg is size of publisher queue/# of messages
    // ros::Publisher dvl_pub = n.advertise<std_msgs::String>("dvl", 1);
    // ros::Rate loop_rate(10);
    // DVLPUB dvl = new DVLPUB(); // ????? idk

    // while (ros::ok()) {
    //     // create msg
    //     dvl.readDevice();
    //     msg.header.stamp = ros::Time::now();
    //     msg.header.frame_id = "dvl_idk_wotdeufk";
        
    //     // velocity report
    //     msg.wrz.velocity.x = dvl.vx;
    //     msg.wrz.velocity.y = dvl.vy;
    //     msg.wrz.velocity.z = dvl.vz;

    //     msg.wrz.valid = dvl.valid;
    //     msg.wrz.altitude = dvl.altitude;
    //     msg.wrz.fom = dvl.fom;
    //     msg.wrz.covariance = dvl.covariance;

    //     msg.wrz.time.validity = dvl.time_of_validity;
    //     msg.wrz.time.transmission = dvl.time_of_transmission;
    //     msg.wrz.time.time = dvl.time;

    //     msg.wrz.status = dvl.status;
    //     msg.wrz.checksum = dvl.checksum;
        
    //     // dead reckoning report???
    //     msg.wrp.x = dvl.x;
    //     msg.wrp.y = dvl.y;
    //     msg.wrp.z = dvl.z;
        
    //     msg.wrp.pos_std = dvl.pos_std;
    //     msg.wrp.roll = dvl.roll;
    //     msg.wrp.pitch = dvl.pitch;
    //     msg.wrp.yaw = dvl.yaw;

    //     dvl_pub.publish(msg);

    //     ros::spinOnce();

    //     loop_rate.sleep();
    // }
    DVLPUB dvl;
    dvl.readDevice();
}


