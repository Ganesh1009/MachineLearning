/*
 * bsn_imu_calib.h
 *
 *  Created on: 5 Dec 2018
 *      Author: musahl
 */

#ifndef BRIDGE_NODE_DONGLE_BSN_IMU_CALIB_H_
#define BRIDGE_NODE_DONGLE_BSN_IMU_CALIB_H_

#include "bsn_imu_data.h"

void calibration_matrix_initialization(imu_eeprom_layout_v00_t *);
imu_calibrated_t bsn_imu_calib_applycalibration(imu_eeprom_layout_v00_t * calib_data, imu_t * imu_raw_data);

#endif /* BRIDGE_NODE_DONGLE_BSN_IMU_CALIB_H_ */
