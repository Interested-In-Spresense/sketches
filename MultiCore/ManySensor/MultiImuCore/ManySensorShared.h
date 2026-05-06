/*
 *  ManySensorShared.h - Shared definitions for ManySensor multi-core sketch.
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

#ifndef MANY_SENSOR_SHARED_H
#define MANY_SENSOR_SHARED_H

#define BMP280CORE_ID      1
#define BMI160CORE_ID      2
#define M_IMUCORE_ID       3

#define MSG_SET_SHARED_BMP 10
#define MSG_SET_SHARED_BMI 11
#define MSG_SET_SHARED_IMU 12
#define MSG_SET_LOCK       13
#define MSG_SENSOR_STOP    20
#define MSG_SENSOR_START   21

#define RINGBUFFER_CAPACITY 64

enum SensorState {
	SENSOR_STATE_READY = 0,
	SENSOR_STATE_RUN   = 1,
};

/* BMP280 measurement modes */
enum BMP280Mode {
	BMP280_MODE_FULL  = 0,  /* Temperature + Pressure + Altitude */
	BMP280_MODE_PRESS = 1,  /* Pressure only */
};

/* BMI160 measurement modes */
enum BMI160Mode {
	BMI160_MODE_ACC  = 0,  /* Acceleration only: ax, ay, az */
	BMI160_MODE_GYRO = 1,  /* Gyroscope only: gx, gy, gz */
	BMI160_MODE_FULL = 2,  /* Acceleration + Gyroscope: ax, ay, az, gx, gy, gz */
};

/* MultiIMU measurement modes */
enum MIMUMode {
	MIMU_MODE_DATA = 0,  /* ax, ay, az, gx, gy, gz */
	MIMU_MODE_FULL = 1,  /* ax, ay, az, gx, gy, gz, timestamp, temperature */
};

#define SENSOR_ERROR_OK        1u
#define SENSOR_STATUS_READY    2u

#define SENSOR_ERR_BMP_BEGIN_FAILED      (-1)
#define SENSOR_ERR_BMP_RECV_FAILED       (-3)
#define SENSOR_ERR_BMP_UNEXPECTED_MSG    (-4)
#define SENSOR_ERR_BMP_LOCK_TIMEOUT      (-7)

#define SENSOR_ERR_BMI_BEGIN_FAILED      (-101)
#define SENSOR_ERR_BMI_RECV_FAILED       (-102)
#define SENSOR_ERR_BMI_UNEXPECTED_MSG    (-103)
#define SENSOR_ERR_BMI_LOCK_TIMEOUT      (-107)

#define SENSOR_ERR_MIMU_BEGIN_FAILED      (-201)
#define SENSOR_ERR_MIMU_INIT_FAILED       (-202)
#define SENSOR_ERR_MIMU_START_FAILED      (-203)
#define SENSOR_ERR_MIMU_RECV_FAILED       (-207)
#define SENSOR_ERR_MIMU_UNEXPECTED_MSG    (-208)
#define SENSOR_ERR_MIMU_LOCK_TIMEOUT      (-209)

/* BMP280 data: [temp_c, pressure_pa, altitude_m] */
#define BMP280_DATA_LEN    3

/* BMI160 data: [ax, ay, az] */
#define BMI160_DATA_LEN    3

/* MultiIMU data: [ax, ay, az, gx, gy, gz] */
#define MULTIMU_DATA_LEN   6

/* MultiIMU full data: [timestamp_s, temp_c, ax, ay, az, gx, gy, gz] */
#define MULTIMU_FULL_LEN   8

/* --- Mode-specific payload structs --- */

struct BMP280Full {
	float temperature;
	float pressure;
	float altitude;
};

struct BMP280Press {
	float pressure;
};

struct BMI160Acc {
	float ax;
	float ay;
	float az;
};

struct BMI160Gyro {
	float gx;
	float gy;
	float gz;
};

struct BMI160Imu {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
};

struct MIMURaw {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
};

struct MIMUFull {
	float timestamp;
	float temperature;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
};

#endif