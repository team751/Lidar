#ifndef LIDAR_LIDARDATACOMMUNICATION_ROBORIOCONNECTION_H_
#define LIDAR_LIDARDATACOMMUNICATION_ROBORIOCONNECTION_H_

#include "../../LidarCommonTypes/tote_pose.h"

#include <iostream>

// Handles the communication between the RoboRIO and the processor running the
// lidar code by assembling and sending requests with tote pose information
//
// Example:
//    RoboRIOConnection *roboRIOConnection;
//    if (!roboRIOConnection.start("127.0.0.1", "9999") {
//      cerr << "Unable to start connection with RoboRIO" << "\n";
//    }
//
//    if (!roboRIOConnection.send(TotePosePacket::ToteEndpoint(x, y))) {
//      cerr << "Failed to send packet to RoboRIO" << "\n";
//    }
//
//    roboRIOConnection.stop()
class RoboRIOConnection {
public:
    // Opens a connection to the server if one doesn't already exist
    // Returns (boolean): returns true if the connection was successfully
    // established or false if an error occurred
    bool start(std::string host, std::string port);

    // Closes the connection to the server
    // Returns (boolean): returns true if the connection was successfully
    // closed
    void stop();

    // Returns (boolean): returns true if the a connection to the server is
    // already open
    bool isOpen();

    // Sends a datapoint to the RoboRIO
    // Returns (boolean): returns true if the message successfully sent
    bool send(TotePose totePacket);

private:
    // Stores the state of the connection
    bool connectionOpen;

    // Stores the current socket
    int currentSocket;

    // The hostname of the RoboRIO
    const char *host;

    // The port the RoboRIO is listening on
    const char *port;
};

#endif // LIDAR_LIDARDATACOMMUNICATION_ROBORIOCONNECTION_H_