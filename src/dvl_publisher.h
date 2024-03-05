#include "serialib.h"
#include "vector"
#include "sstream"
#include "string"

class DVLPUB{
    public:
        // serial connection
        serialib serial;
        // string stream to hold data
        std::stringstream ss;
        // vector of data for wrz
        std::vector<std::string> wrz;
        // vector of data wrp
        std::vector<std::string> wrp;
        // https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol
        // WRZ data members
        // ex: wrz,0.120,-0.400,2.000,y,1.30,1.855,1e-07;0;1.4;0;1.2;0;0.2;0;1e+09,7,14,123.00,1*50
        // Velocity in x direction (m/s)
        double vx;
        // Velocity in y direction (m/s)
        double vy;
        // Velocity in z direction (m/s)
        double vz;
        // If y, the DVL has a lock on the reflecting surface,
        // and the altitude and velocities are valid (y/n)
        std::string valid;
        // Measured altitude to the bottom (m)
        double altitude;
        // Figure of merit, a measure of the accuracy of the velocities (m/s)
        double fom;
        // Covariance matrix for the velocities. The figure of merit is calculated from this.
        // 9 entries ((m/s)^2) separated by ;
        std::string covariance;
        // Timestamp of the surface reflection, aka 'center of ping' (Unix timestamp in microseconds)
        double time_of_validity;
        // Timestamp from immediately before sending of the report over TCP (Unix timestamp in microseconds)
        double time_of_transmission;
        // Milliseconds since last velocity report (ms)
        double time;

        // WRP data members
        // wrp,[time_stamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],[status]
        // wrp,49056.809,0.41,0.15,1.23,0.4,53.9,13.0,19.3,0*de

        // Time stamp of report (Unix timestamp in seconds)
        double time_stamp;
        // Distance in X direction (m)
        double x;
        // Distance in Y direction (m)
        double y;
        // Distance in downward direction (m)
        double z;
        // Standard deviation (Figure of merit) for position (m)
        double pos_std;
        // Rotation around X axis (degrees)
        double roll;
        // Rotation around Y axis (degrees)
        double pitch;
        // Rotation around Z axis, i.e. heading (degrees)
        double yaw;
        
        // used for wrz and wrp:
        // 0 for normal operation, 1 for operational issues such as high temperature
        std::string status;
        // checksum format: '*' followed by 2 hex digits
        std::string checksum;
        
        DVLPUB();

        // destructor
        ~DVLPUB();

        int readDevice();

        void parseData();

        void parseWRZ();

        void parseWRP();

        // function to parse data read into vector of string for WRZ Velocity report
        void setWRZ();

        // function to parse data read into vector of string for WRP dead reckoning report
        void setWRP();

        void printWRZ();

        void printWRP();

        int reportType();
};