//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

struct ahrs_out_s
{
  float beta;
  float q[4];
  float samplerate;
};

#define INIT_AHRS(i, b, s)  do { (i)->beta = (b); \
                                 (i)->q[0] = 1.0f; (i)->q[1] = 0.0f; \
                                 (i)->q[2] = 0.0f; (i)->q[3] = 0.0f; \
                                 (i)->samplerate = (s); }while(0)

void MadgwickAHRSupdateIMU(struct ahrs_out_s *out,
                           float gx, float gy, float gz,
                           float ax, float ay, float az, float dtsec);
void quaternion2euler(const float q[4], float e[3]);
void euler2quaternion(const float e[3], float q[4]);
void setPostureByAccel(struct ahrs_out_s *inst,
                       float ax, float ay, float az, float yaw);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
