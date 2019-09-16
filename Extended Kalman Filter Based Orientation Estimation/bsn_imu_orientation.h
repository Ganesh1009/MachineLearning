/*
 * bsn_imu_orientation.h
 *
 *  Created on: 5 Dec 2018
 *      Author: musahl
 */

#ifndef BRIDGE_NODE_DONGLE_BSN_IMU_ORIENTATION_H_
#define BRIDGE_NODE_DONGLE_BSN_IMU_ORIENTATION_H_

void bsn_imu_orientation_initialize();

void bsn_imu_orientation_add_sample(imu_calibrated_t * calibrated_imu_data);

imu_orientation_t bsn_imu_orientation_get_quaternion();

void bsn_imu_orientation_free(void);
#endif /* BRIDGE_NODE_DONGLE_BSN_IMU_ORIENTATION_H_ */
