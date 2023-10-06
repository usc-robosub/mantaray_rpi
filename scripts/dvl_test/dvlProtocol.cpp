// opening up the DVL with serial?
#include "serialib.cpp"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <cstring>
#include <unistd.h>
//idk
#include <crcmod>

using namespace std;

class DVLGenericError : public std::exception {
    public:
        DVLGenericError(const char* message) : message_(message) {}
        const char* what() const noexcept override {
            return message_.c_str();
        }
    private:
        std::string message_;
};

class ProtocolParseError : public DVLGenericError {
    public:
        ProtocolParseError(const char* message) : DVLGenericError(message) {}
};

class ProtocolChecksumError : public ProtocolParseError {
    public:
        ProtocolChecksumError(const char* message) : ProtocolParseError(message) {}
};

class WlProtocolParser {
public:
    WlProtocolParser() {
        //wtf is crc_func
        //crc_func = None;
        //crc_func = crcmod.predefined.mkPredefinedCrcFun("crc-8");
        //from chatGPT:
        crc_func = crcmod::crcmod::Crc(CRC8_POLY);
    }

    static const char* do_format_checksum(uint8_t checksum) {
        char checksum_str[4];
        snprintf(checksum_str, sizeof(checksum_str), "*%02X", checksum);
        return checksum_str;
    }

    uint8_t checksum_for_buffer(const std::vector<uint8_t>& data) {
        crc_func.reset();
        crc_func.update(data.data(), data.size());
        uint8_t checksum = crc_func.checksum();
        return checksum;
    }

    std::vector<uint8_t> parse(const std::vector<uint8_t>& sentence) {
        std::vector<uint8_t> result;
        char sop = sentence[0];
        /*
        if isinstance(sop, bytes) {
            sop = ord(sop);
        }*/
        if (sop != SOP) {
            return result; // Return an empty result for invalid SOP
        }
        if (sentence.size() < 3) {
            return result; // Return an empty result for a too short sentence
        }

        char direction = sentence[1];
        //missing stuff...
        if (direction != DIR_CMD && direction != DIR_RESP) {
            return result; // Return an empty result for an invalid direction
        }

        bool got_checksum = is_checksum(sentence[sentence.size() - 3]);
        std::vector<uint8_t> csum;
        if (got_checksum) {
            csum = {sentence.end() - 3, sentence.end()};
            sentence.resize(sentence.size() - 3); // Remove checksum to ease further processing
            if (csum != checksum_for_buffer(sentence)) {
                return result; // Return an empty result for a checksum error
            }
        }

        char cmd = sentence[2];
        if (isValidCommand(cmd)) {
            // Parse the sentence and populate the result vector
            // Implement your parsing logic here
        }

        return null;
    }

    bool isValidCommand(char cmd) {
        // Implement your command validation logic here
        return true;
    }

    unordered_map<string, float> doDict(cmd, direction, options) {
        unordered_map<string, float> result = {
            "time": float(options[0].decode('utf-8')),
            "vx": float(options[1].decode('utf-8')),
            "vy": float(options[2].decode('utf-8')),
            "vz": float(options[3].decode('utf-8')),
            "fom": float(options[4].decode('utf-8')),
            "covariance": float(options[0].decode('utf-8')),
            "altitude": float(options[5].decode('utf-8')),
            "valid": true if options[6].decode('utf-8') == 'y' else false,
        }
        return result;
    }

private:
    crcmod::crcmod::Crc crc_func;
    static const uint8_t CRC8_POLY = 0xD5; // CRC-8 polynomial

};

//figure out wtf all these variables are
class DVLBase {
    public: 
        DVLBase(IoDevice& iodev, bool debug=false) 
            : _iodev(iodev),
            parser(WlProtocolParser()),
            payload_size(-1),
            holdoff(0),
            buffer(), // = bytearray() ???
            debug(debug),
            rx_queue(),
            oldString("") {}

        vector<uint8_t> getData(DVLBase self) {
            oldString = self.oldString;
            contain = 0;
            raw_data = "";
            while (contain == 0) {
                raw_data = raw_data + self._iodev.read(1).decode("utf-8"); // i dont think this is legal in c++ but put here temprarily')
                if ("\r\n" in raw_data) {
                    contain = 1;
                }
            }
            raw_data = oldString + raw_data;
            oldString = "";
            raw_data = raw_data.split("\r\n");
            oldString = raw_data[1];
            raw_data = raw_data[0] + "\r\n";
            self.oldString = oldString;

            return raw_data;
        }

    private:
        IoDevice& _iodev;
        WlProtocolParser parser;
        int payload_size;
        int holdoff;
        std::vector<uint8_t> buffer;
        bool debug;
        std::vector<std::string> rx_queue;
};

class DVL : public DVLBase 
{
    public:
        DVL(const std::string& device, int baudrate=115200, bool debug=false) {
            try {
                DVL._serial = serial.Serial(device, baudrate);
            }
            catch {
                throw {
                    DVLGenericError("Error opening serial port");
                }
            }
        }
};