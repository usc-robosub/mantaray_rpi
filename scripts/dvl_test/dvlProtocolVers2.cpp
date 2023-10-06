#include "serialib.cpp"
#include <vector>

//look into serial library
//open device
//if isDeviceOpen = true then do stuff
// writeString to put stuff into a buffer
// wrz command to get velocity report
// get time, vx, vy, vz, covariance, fom, altitude, valid
//readString 


//parser class to parse thru stuff
//the writing string stuff shoudl be here?
//parse through the velocity report
// creates a hashmap then puts it into some thing to spit out some variable like covariance

//dvlbase object
//create a parser
// buffer = some vector? -> is bytearray() in dvl-python
//initialize it
//getData
//getPack?/
//read data
//the data we get is the packet, then put through parser to get specific data


//create dvl object
//class DVL