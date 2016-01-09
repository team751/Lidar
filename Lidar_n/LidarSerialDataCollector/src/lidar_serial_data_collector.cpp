#include "../inc/lidar_serial_data_collector.h"

#include "../inc/constants.h"
#include "../inc/lidar_serial_packet.h"

#include <serial/serial.h>

// Constructor
LidarSerialDataCollector::LidarSerialDataCollector(std::string serialPortName, uint32_t baudRate) {
    this->serialPortName = serialPortName;
    this->baudRate = baudRate;
}

bool LidarSerialDataCollector::start() {
    // Check preconditions
    if (isRunning()) return false;

    // Set state variables
    this->running = true;
    this->machineState = WAITING_FOR_HEADER;

    // Start the data collector on a new thread
    processorThread = std::thread(&LidarSerialDataCollector::runMachine, this);
    processorThread.detach();

    return true;
}

void LidarSerialDataCollector::stop() {
    // Check preconditions
    if (!isRunning()) return;

    // Set state variables
    this->machineState = NOT_RUNNING;
    this->running = false;
}

bool LidarSerialDataCollector::isRunning() {
    return this->running;
}

void LidarSerialDataCollector::saveData(LidarSerialPacket packet) {
    int startAngle = packet.index * 4;

    for (int angle = startAngle; angle < startAngle + 4; angle++) {
        this->output[angle] = packet.getDistance(angle - startAngle);
    }
}

void LidarSerialDataCollector::runMachine() {
    if (this->serialPortName == "") return;
    if (this->baudRate == 0) return;

    // Setup Serial Port
    serial::Serial serialPort(this->serialPortName, this->baudRate);

    // An integer used to store the data packet being read
    int currentDataReadIndex;

    // An integer that stores the current byte of the data packet being read (4 bytes per data packet, 2 for speed)
    int currentPacketIndex;

    while (this->running && serialPort.isOpen()) {
        // Read 1 byte into a buffer
        std::vector<uint8_t> buffer;
        serialPort.read(buffer);

        // Ensure data was received
        if (buffer.size() == 0) continue;

        // Grab the first byte (of 1 byte array)
        uint8_t inputByte = buffer[0];


        switch (this->machineState) {
            case NOT_RUNNING:
                break;
            case WAITING_FOR_HEADER:
                // Check if we received a header bit
                if (inputByte == Constants::kHeaderBit) {
                    this->machineState = WAITING_FOR_INDEX;
                }

                if (serialPacket != NULL) this->saveData(*serialPacket);

                // Generate a new packet
                LidarSerialPacket newPacket;
                serialPacket = &newPacket;

                break;
            case WAITING_FOR_INDEX:
                // Store the packet index
                serialPacket->index = inputByte - Constants::kIndexLowerBound;

                // Reset variables
                currentPacketIndex = 0;

                this->machineState = WAITING_FOR_SPEED;

                break;
            case WAITING_FOR_SPEED:
                // If it's byte 1...
                if (currentPacketIndex == 0) {
                    // Implicitly cast the int to a double
                    serialPacket->speedData[0] = inputByte;

                    // Specify that the first byte has been stored
                    currentPacketIndex = 1;
                } else {
                    serialPacket->speedData[1] = inputByte;

                    // Reset variables
                    currentPacketIndex = 0;
                    currentDataReadIndex = 0;


                    this->machineState = WAITING_FOR_DATA;
                }

                break;
            case WAITING_FOR_DATA:
                // Set the current serial byte
                serialPacket->data[currentPacketIndex][currentDataReadIndex] = inputByte;

                // Increment the read index (4 per data set)
                currentDataReadIndex++;

                // If all 4 bytes have been delivered...
                if (currentDataReadIndex >= 4) {
                    // Goto the next data set
                    currentPacketIndex++;

                    // Reset the read index to 0
                    currentDataReadIndex = 0;
                }

                // If all 4 sets are delivered...
                if (currentPacketIndex >= 4) {
                    // Reset variables
                    currentPacketIndex = 0;
                    currentDataReadIndex = 0;

                    this->machineState = WAITING_FOR_CHECKSUM;
                }
                break;
            case WAITING_FOR_CHECKSUM:
                newPacket.checksum[currentPacketIndex] = inputByte;

                currentPacketIndex++;

                if (currentPacketIndex == 2) {
                    this->machineState = WAITING_FOR_HEADER;
                }
                break;
            default:
                break;
        }
    }
}
