//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 22/05/2025	Sony Semiconductor Solutions	Add function to set roll/pitch posture by using Accel
//           	                            	Unified the unit of euler angles to radian
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

struct ahrs_workmem_s
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3;
  float _4q0, _4q1, _4q2;
  float _8q1, _8q2;
  float q0q0, q1q1, q2q2, q3q3;
};

static struct ahrs_workmem_s g_work;

//---------------------------------------------------------------------------------------------------
// Function declarations

static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;

	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(struct ahrs_out_s *inst,
                           float gx, float gy, float gz,
                           float ax, float ay, float az, float dtsec)
{
	// Rate of change of quaternion from gyroscope
	g_work.qDot1 = 0.5f * (-inst->q[1] * gx - inst->q[2] * gy - inst->q[3] * gz);
	g_work.qDot2 = 0.5f * ( inst->q[0] * gx + inst->q[2] * gz - inst->q[3] * gy);
	g_work.qDot3 = 0.5f * ( inst->q[0] * gy - inst->q[1] * gz + inst->q[3] * gx);
	g_work.qDot4 = 0.5f * ( inst->q[0] * gz + inst->q[1] * gy - inst->q[2] * gx);

	// Compute feedback only if accelerometer measurement valid
  //  (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		g_work.recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= g_work.recipNorm;
		ay *= g_work.recipNorm;
		az *= g_work.recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		g_work._2q0 = 2.0f * inst->q[0];
		g_work._2q1 = 2.0f * inst->q[1];
		g_work._2q2 = 2.0f * inst->q[2];
		g_work._2q3 = 2.0f * inst->q[3];
		g_work._4q0 = 4.0f * inst->q[0];
		g_work._4q1 = 4.0f * inst->q[1];
		g_work._4q2 = 4.0f * inst->q[2];
		g_work._8q1 = 8.0f * inst->q[1];
		g_work._8q2 = 8.0f * inst->q[2];
		g_work.q0q0 = inst->q[0] * inst->q[0];
		g_work.q1q1 = inst->q[1] * inst->q[1];
		g_work.q2q2 = inst->q[2] * inst->q[2];
		g_work.q3q3 = inst->q[3] * inst->q[3];

		// Gradient decent algorithm corrective step
		g_work.s0 = g_work._4q0 * g_work.q2q2 +
               g_work._2q2 * ax           +
               g_work._4q0 * g_work.q1q1  -
               g_work._2q1 * ay;
		g_work.s1 = g_work._4q1 * g_work.q3q3
              - g_work._2q3 * ax
              + 4.0f * g_work.q0q0 * inst->q[1]
              - g_work._2q0 * ay
              - g_work._4q1
              + g_work._8q1 * g_work.q1q1
              + g_work._8q1 * g_work.q2q2
              + g_work._4q1 * az;
		g_work.s2 = 4.0f * g_work.q0q0 * inst->q[2]
              + g_work._2q0 * ax
              + g_work._4q2 * g_work.q3q3
              - g_work._2q3 * ay
              - g_work._4q2
              + g_work._8q2 * g_work.q1q1
              + g_work._8q2 * g_work.q2q2
              + g_work._4q2 * az;
		g_work.s3 = 4.0f * g_work.q1q1 * inst->q[3]
              - g_work._2q1 * ax
              + 4.0f * g_work.q2q2 * inst->q[3]
              - g_work._2q2 * ay;

    // normalise step magnitude
		g_work.recipNorm = invSqrt(g_work.s0 * g_work.s0 +
                               g_work.s1 * g_work.s1 +
                               g_work.s2 * g_work.s2 +
                               g_work.s3 * g_work.s3);

		g_work.s0 *= g_work.recipNorm;
		g_work.s1 *= g_work.recipNorm;
		g_work.s2 *= g_work.recipNorm;
		g_work.s3 *= g_work.recipNorm;

		// Apply feedback step
		g_work.qDot1 -= inst->beta * g_work.s0;
		g_work.qDot2 -= inst->beta * g_work.s1;
		g_work.qDot3 -= inst->beta * g_work.s2;
		g_work.qDot4 -= inst->beta * g_work.s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	inst->q[0] += g_work.qDot1 * dtsec;
	inst->q[1] += g_work.qDot2 * dtsec;
	inst->q[2] += g_work.qDot3 * dtsec;
	inst->q[3] += g_work.qDot4 * dtsec;

	// Normalise quaternion
	g_work.recipNorm = invSqrt(inst->q[0] * inst->q[0] +
                             inst->q[1] * inst->q[1] +
                             inst->q[2] * inst->q[2] +
                             inst->q[3] * inst->q[3]);
	inst->q[0] *= g_work.recipNorm;
	inst->q[1] *= g_work.recipNorm;
	inst->q[2] *= g_work.recipNorm;
	inst->q[3] *= g_work.recipNorm;
}

static inline float rad2deg(float rad)
{
    return rad * (180.0f / (float) M_PI);
}

static inline float arc_sinf(const float value)
{
    if (value <= -1.0f) {
        return (float) M_PI / -2.0f;
    }
    if (value >= 1.0f) {
        return (float) M_PI / 2.0f;
    }
    return asinf(value);
}

void quaternion2euler(const float q[4], float e[3])
{
  const float hmq = 0.5f - q[2] * q[2];
  e[0] = atan2f(q[0] * q[1] + q[2] * q[3], hmq - q[1] * q[1]);
  e[1] = arc_sinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
  e[2] = atan2f(q[0] * q[3] + q[1] * q[2], hmq - q[3] * q[3]);
}

void euler2quaternion(const float e[3], float q[4])
{
	float sin_rx_2, sin_ry_2, sin_rz_2;
	float cos_rx_2, cos_ry_2, cos_rz_2;

	sin_rx_2 = sinf(e[0] / 2.0);
	sin_ry_2 = sinf(e[1] / 2.0);
	sin_rz_2 = sinf(e[2] / 2.0);

	cos_rx_2 = cosf(e[0] / 2.0);
	cos_ry_2 = cosf(e[1] / 2.0);
	cos_rz_2 = cosf(e[2] / 2.0);

	q[0] =  cos_rx_2 * cos_ry_2 * cos_rz_2 + sin_rx_2 * sin_ry_2 * sin_rz_2;
	q[1] =  sin_rx_2 * cos_ry_2 * cos_rz_2 - cos_rx_2 * sin_ry_2 * sin_rz_2;
	q[2] =  sin_rx_2 * cos_ry_2 * sin_rz_2 + cos_rx_2 * sin_ry_2 * cos_rz_2;
	q[3] =  cos_rx_2 * cos_ry_2 * sin_rz_2 - sin_rx_2 * sin_ry_2 * cos_rz_2;
}

void setPostureByAccel(struct ahrs_out_s *inst,
					   float ax, float ay, float az, float yaw)
{
	float a[3] = {ax, ay, az};
	float norm;
	float euler[3] = {0.0};

	norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	if (norm == 0){
		return;
	}
	else{
		g_work.recipNorm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		a[0] *= g_work.recipNorm;
		a[1] *= g_work.recipNorm;
		a[2] *= g_work.recipNorm;
	}
	
	euler[0] = atan2f(a[1], a[2]);
	euler[1] = -1.0 * asinf(a[0]);
	euler[2] = yaw;
	
	euler2quaternion(euler, inst->q);

	g_work.recipNorm = invSqrt(inst->q[0] * inst->q[0] + 
							   inst->q[1] * inst->q[1] +
							   inst->q[2] * inst->q[2] +
							   inst->q[3] * inst->q[3]);
	inst->q[0] *= g_work.recipNorm;
	inst->q[1] *= g_work.recipNorm;
	inst->q[2] *= g_work.recipNorm;
	inst->q[3] *= g_work.recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//====================================================================================================
// END OF CODE
//====================================================================================================
