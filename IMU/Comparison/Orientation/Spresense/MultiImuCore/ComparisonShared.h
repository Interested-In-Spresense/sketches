/*
 *  ComparisonShared.h - Shared definitions for Comparison multi-core sketch.
 *  Author Interested-In-Spresense
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef COMPARISON_SHARED_H
#define COMPARISON_SHARED_H

#define BMI160CORE_ID      1
#define M_IMUCORE_ID       2

#define MSG_SET_SHARED_BMI 11
#define MSG_SET_SHARED_IMU 12
#define MSG_SET_LOCK       13
#define MSG_SENSOR_STOP    20
#define MSG_SENSOR_START   21
#define MSG_SENSOR_READY   22

#define RINGBUFFER_CAPACITY 256

enum SensorState {
	SENSOR_STATE_READY = 0,
	SENSOR_STATE_RUN   = 1,
};

#define SENSOR_ERR_BMI_UNEXPECTED_MSG    (-103)
#define SENSOR_ERR_BMI_LOCK_TIMEOUT      (-107)

#define SENSOR_ERR_MIMU_BEGIN_FAILED      (-201)
#define SENSOR_ERR_MIMU_INIT_FAILED       (-202)
#define SENSOR_ERR_MIMU_START_FAILED      (-203)

struct BMIQuaternion {
	float timestamp;
	float q0, q1, q2, q3;
};

struct MIMUQuaternion {
	float timestamp;
	float temperature;
	float q0, q1, q2, q3;
};

#endif