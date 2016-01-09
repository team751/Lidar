#ifndef LIDAR_LIDARSERIALDATACOLLECTOR_LIDARSERIALDATACOLLECTOR_H_
#define LIDAR_LIDARSERIALDATACOLLECTOR_LIDARSERIALDATACOLLECTOR_H_

#include <array>
#include <iostream>
#include <thread>

// #include <serial/serial.h>

#include "lidar_serial_packet.h"

class LidarSerialDataCollector {
public:
    // Constructor (name and baud rate)
    LidarSerialDataCollector(std::string serialPortName, uint32_t baudRate);

    // Call to start running the data collector
    // Returns (boolean): true if successful
    bool start();

    // Call to stop running the data collector
    void stop();

    // Return (boolean): true if serial collector is running
    bool isRunning();

    // Stores the current serial packet
    LidarSerialPacket *serialPacket = NULL;

    // Returns a constant copy of the output array
    std::array<double, 360> getOutput() const { return this->output; };

private:
    // A private method run on a separate thread to read data from the lidar via the serial port and parse it.
    void runMachine();

    // A private method to save the data of a packet to the output array
    void saveData(LidarSerialPacket packet);

    // Stores the output (angle = key) (distance = value)
    std::array<double, 360> output;

    // The possible states of the finite state machine
    enum MachineState {
        NOT_RUNNING, WAITING_FOR_INDEX, WAITING_FOR_HEADER, WAITING_FOR_SPEED, WAITING_FOR_DATA, WAITING_FOR_CHECKSUM
    };

    // Stores the current state of the finite state machine
    MachineState machineState;

    // Stores the serial port name the Lidar is on
    std::string serialPortName;

    // Stores the baud rate the lidar is using
    uint32_t baudRate;

    // Stores if the processor is running
    bool running;

    // The thread that's running the data processor
    std::thread processorThread;
};

#endif // LIDAR_LIDARSERIALDATACOLLECTOR_LIDARSERIALDATACOLLECTOR_H_
