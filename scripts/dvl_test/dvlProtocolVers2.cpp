#include "serialib.cpp"
#include <vector>
#include <iostream>
#include <string.h>
#include <unordered_map>

//using namespace std; ??
//look into serial library
//open device

//todo make the buffer a char[] ?? rather than a string

char readDevice(const char device, int baudrate, std::string* buffer) {
    serialib serial;
    char dvl_test = serial.openDevice(device, baudrate);
    if (dvl_test!=1) {
        return dvl_test;
    }
    printf("device opened successfully");
    //dont need a loop bc the readDevice thing will be loop called somewhere else... i think ;p;
    // while(serial.available() > 0) {
    //     try {
    //         int stringRead = serial.readString(buffer, '\n', 200/*idk so random number for now*/);
    //         if (stringRead!=1) {
    //             return stringRead;
    //         }
    //         printf("device read successfully");

    //         //todo add smth to store the buffer somewhere, get it parsed n shit
    //         delete[] buffer;
    //     }
    //     catch (const std::exception& e){
    //         std::cerr << e.what() << '\n';

    //     }
    // }
    if (serial.available() > 0) {
        try {
            int stringRead = serial.readString(buffer, '\n', 200/*idk so random number for now*/);
            if (stringRead!=1) {
                return stringRead;
            }
            printf("device read successfully");

            //todo add smth to store the buffer somewhere, get it parsed n shit
        }
        catch (const std::exception& e){
            std::cerr << e.what() << '\n';

        }
    serial.closeDevice();

    return 1;
}


//if isDeviceOpen = true then do stuff
// writeString to put stuff into a buffer
// wrz command to get velocity report
// get time, vx, vy, vz, covariance, fom, altitude, valid
//readString 


//parser class to parse thru stuff
class ProtocolParser() {
    public:
        ProtocolParser();
        bool parse(std::string* buffer) {
            vector<float> dataParts;
            std::string option;
            float optionFloat;
            //split into parts by the ,'s, then add each thing into parts
            int index = 0;
            while (buffer.find(",") != std::string::npos) {
                index = buffer.find(",");
                option = buffer.substr(0,index);
                //change option from string to float here
                dataParts.push_back(optionFloat);
                buffer.erase(0, index);
            }

            //check type of report
            if (dataParts[0] /*!= the velocity report*/) {
                printf("Not the velocity report");
                return 0;
            }
            /*
            tf is this lol,,, from the example py code
            # Split covariance by ';' and convert to list of float values
            covariance = list(map(str2float, cov.split(b";")))
            */
           std::unordered_map<std::string, float> parts = {
            //dataparts[0] is "wrz" i think
                {"vx", dataParts[1]},
                {"vy", dataParts[2]},
                {"vz", dataParts[3]},
                {"valid", dataParts[5]},
                {"altitude", dataParts[6]},
                {"fom", dataParts[7]},
                {"covariance", dataParts[8]},
                {"time_of_validity", dataParts[9]},
                {"time_of_transmission", dataParts[10]},
                {"time", dataParts[11]},
                {"status",dataParts[12]}
                 //etc.. finish later
            }
        }
        // creates a hashmap then puts it into some thing to spit out some variable like covariance

    //stuff
};
//the writing string stuff shoudl be here?
//parse through the velocity report

//dvlbase object
class DVLBase {
    public:
        std::string buffer;
        bool debug;
        string oldString;
        DVLBase(debug=false) {
            this.debug = debug;
            buffer = some array idk;
            oldString = "";
            ProtocolParser parser = ProtocolParser(); 
        }
        //add in a function to split stuff??
        // should spit out the buffer thing as a string rawData
        std::string getData(DVLBase dvl) {
            bool contain = false;
            string rawData = "";
            while (contain) {
                rawData = rawData + //read the data from dvl;
                if ("\r\n" in rawData) { // from the python version
                    contain = 1;
                }
            }
            /*
            // get data from dvl
            rawData = oldString _ rawData;
            oldString = "";
            rawData = rawData.split("\r\n");
            oldString = rawData[1];
            rawData = rawData[0] + "\r\n"
            self.oldString = oldString*/
            return rawData;
        }
        std::string getPack() {
            std:string rawData = "false";
            //smth
            return rawData;
        }

        vector<float> read() {
            readDevice(device, bauds, buffer);
            part = parser.parse(buffer);
            //get dataParts from that yo
            return part;

        }

};
//create a parser
// buffer = some vector? -> is bytearray() in dvl-python
//initialize it
//getData
//getPack?/
//read data
//the data we get is the packet, then put through parser to get specific data


//create dvl object
//class DVL
class DVL : DVLBase
{
    public:
        char dvl;
        const unsigned int bauds;
        DVL(const char *device, const unsigned int bauds) {
            try {
                //open device
                dvl = readDevice(device, bauds);
                //get the buffer from it and read hUH

            }
            catch {
                throw WlDVLGenericError("Error opening serila device"); //i'll add later
            }
        }
};