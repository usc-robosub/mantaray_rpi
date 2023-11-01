class DVLPUB{
    public:
        // vector of data for wrz
        std::vector<std::string> wrz;
        // vector of data wru
        std::vector<std::string> wru;
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
        // 0 for normal operation, 1 for operational issues such as high temperature
        std::string status;

        // WRU data members
        /* ex:
            wru,0,0.070,1.10,-40,-95*9c
            wru,1,-0.500,1.25,-62,-104*f0
            wru,2,2.200,1.40,-56,-98*18
            wru,3,1.800,1.35,-58,-96*a3
        */
        // Transducer number
        int id;
        // Velocity in the direction of the transducer (m/s)
        double velocity;
        // Distance (parallel to the transducer beam, i.e. not the vertical distance) to the reflecting surface from this transducer (m)
        double distance;
        // Received signal strength indicator: strength of the signal received by this transducer (dBm)
        double rssi;
        // Noise spectral density: strength of the background noise received by this transducer (dBm)
        std::string nsd;
        // constructor
        DVLPUB();

        // destructor
        ~DVLPUB();

        // function to make serial connection and read data from device
        int readDevice();

        // function to differentiate reports
        int checkReport(char* buff);

        // function to parse comma separated report (buff) into data vector
        // void checkReport(char* buff);

        //
        void parseData(char* buff);

        // function to parse data read into vector of string for WRZ Velocity report
        void setWRZ();

        // function to parse data read into vector of string for WRU Transducer report
        void setWRU();

        
};