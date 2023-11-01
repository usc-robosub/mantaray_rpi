#include "dvl_publisher.h"
#include "serialib.h"
#include "vector"
#include "sstream"
#include "string"

#define PORT "/dev/ttyUSB0"

// constructor
DVLPUB::DVLPUB(){

}
// destructor
DVLPUB::~DVLPUB(){

}

// reads data from serial connection while open
int DVLPUB::readDevice(){
    serialib serial;
    char dvl_test = serial.openDevice(PORT, 115200);
    if (dvl_test!=1) {
        return dvl_test;
    }
    std::cout << "Opened Device" << std::endl;
    while (true){
        while (serial.available() > 0){
            try{
                char* buff = new char[10000]; // how large can report be
                // does buff contain both WRZ and WRU reports???
                // int report = checkReport(buff);
                serial.readBytes(buff,9999,1000);
                parseData(buff);
                // if (report == 1){
                //     setWRZ(buff);
                // }
                // else if (report == 2){
                //     setWRU(buff);
                // }
                // regroup with matt on how to send before next loop overwrites values
                if (wrz.size() != 0){
                    setWRZ();
                }
                if (wru.size() != 0){
                    setWRU();
                }
                wru.clear();
                wrz.clear();
                delete[] buff;
            } catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }
    }
    return 1;
}

// function to check with report was sent
// int DVLPUB::checkReport(char* buff){
//     if (strncmp(buff,"wrz,",4) == 0){
//         return 1;
//     }
//     else if (strncmp(buff,"wru,",4) == 0) {
//         return 2;
//     }
//     return 0;
// }

void DVLPUB::parseData(char* buff){
    std::stringstream ss(buff);
    std::string val;
    int report = 0;
    while (getline(ss,val,',')){
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
            wru.push_back(val);
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
    } catch (const std::out_of_range& e) {
        std::cerr << "Out of range exception: " << e.what() << std::endl;
    }
}
// parse WRU Transducer report from data vector
void DVLPUB::setWRU(){
    // sends 4, could store in array of size 4 for each value
    // check if parse works with sample input with newline
    // how tf does it send this data
}

int main(int argc, char* argv[]){
    // ros stuff here
}


