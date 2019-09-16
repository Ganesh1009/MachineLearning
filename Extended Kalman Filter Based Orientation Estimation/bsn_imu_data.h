/*
 * bsn_imu_data.h
 *
 *  Created on: 5 Dec 2018
 *      Author: musahl
 */

#ifndef BRIDGE_NODE_DONGLE_BSN_IMU_DATA_H_
#define BRIDGE_NODE_DONGLE_BSN_IMU_DATA_H_

#include <stdint.h>

#pragma pack(push, 1)
typedef struct data_layout_v00
{
    uint8_t name[2];
    uint8_t date[6];
    uint32_t id;
    uint16_t version;
    float ba[3];
    float bg[3];
    float bm[3];
    float krainv[9];
    float krginv[9];
    float krminv[9];
    uint8_t free[94];
    uint32_t crc;
} imu_eeprom_layout_v00_t;
#pragma pack(pop)


typedef struct imu_s {
    uint32_t time;
    uint8_t id;
    int16_t temp;
    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];
} imu_t;

typedef struct imu_calibrated_s {
  uint32_t time;
  uint8_t id;
  float temp;
  float acc[3];
  float gyr[3];
  float mag[3];
} imu_calibrated_t;

typedef struct imu_orientation_s {
  uint32_t time;
  uint8_t id;
  float x;
  float y;
  float z;
  float w;
} imu_orientation_t;

#endif /* BRIDGE_NODE_DONGLE_BSN_IMU_DATA_H_ */
