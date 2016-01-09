#ifndef LIDAR_LIDARSERIALDATACOLLECTOR_CONSTANTS_H
#define LIDAR_LIDARSERIALDATACOLLECTOR_CONSTANTS_H

namespace Constants {
    // The bit sent by the lidar whenever it's about to send a new packet
    const uint8_t kHeaderBit       = 0xFA;
    const uint8_t kIndexLowerBound = 0xA0;
}

#endif //LIDAR_LIDARSERIALDATACOLLECTOR_CONSTANTS_H
