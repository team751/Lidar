//
// Created by Sam Baumgarten on 10/3/15.
//

#ifndef LIDAR_LIDARSERIALDATACOLLECTOR_LIDARSERIALPACKET_H
#define LIDAR_LIDARSERIALDATACOLLECTOR_LIDARSERIALPACKET_H

struct LidarSerialPacket {
    uint8_t index;
    int speedData[2];
    uint8_t data[4][4];
    uint8_t checksum[2];

    double getDistance(int i) {
        return (this->data[i][0] | ((this->data[i][1] & 0x3f) << 8));
    }

    double getSpeed() {
        // Combine the bytes and divide by 64 (returned in 1/64ths of a rpm)
        return (this->speedData[0] | (this->speedData[1] << 8)) / 64.0;
    }
};

#endif // LIDAR_LIDARSERIALDATACOLLECTOR_LIDARSERIALPACKET_H