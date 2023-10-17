#include "serialib.cpp"
#include <vector>
#include <iostream>
#include <string.h>
#include <unordered_map>

//using namespace std; ??

//look into serial library
//open device
//if isDeviceOpen = true then do stuff
// writeString to put stuff into a buffer
// wrz command to get velocity report
// get time, vx, vy, vz, covariance, fom, altitude, valid
//readString 


//parser class to parse thru stuff
class ProtocolParser() {
    public:
        ProtocolParser();
        bool parse(std::string buffer) {
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
           std::unordered_map<std::string, float> parts {
            //dataparts[0] is "wrz" i think
                {"vx", dataParts[1]},
                {"vy", dataParts[2]},
                {"vz", dataParts[3]} //etc.. finish later
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
        vector<float> buffer;
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
            //some bs todo idk
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
class DVL //: smth that i will add later
{
    public:
        char dvl;
        const unsigned int baudRate;
        DVL(const char *device, const unsigned int bauds) {
            try {
                dvl = serialib.openDevice(device, bauds);

            }
            catch {
                throw WlDVLGenericError("Error opening serila device"); //i'll add later
            }
        }
};