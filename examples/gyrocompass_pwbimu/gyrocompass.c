/****************************************************************************
 * examples/gyrocompass_pwbimu/gyrocompass.c
 *
 *   Copyright 2025 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <math.h>

#include <nuttx/sensors/cxd5602pwbimu.h>
#include "gyrocompass.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EARTH_RATE   (2.0 * M_PI / 86164.098903691)
#define RAD2DEG      (180.0 / M_PI)

#define SQUARED(v)  ((v) * (v))
#define CUBE(v)     ((v) * (v) * (v))
#define DET_MATRIX(m) m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - \
                      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + \
                      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])

/*****************************************************************************
 * Private Types
 *****************************************************************************/

struct circlefit_workmem_s
{
  double workA;
  double workB;

  double f_mat[3][3];
  double g_vec[3];
  double t_vec[3];
  double tmp_mat[3][3];
};

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static struct circlefit_workmem_s g_work;

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

int calc_bias_circlefitting(struct dvec3_s *gavgs, int num,
                            struct dvec3_s *bias_out)
{
  int i;
  int r;
  int c;

  memset(g_work.f_mat, 0, sizeof(g_work.f_mat));
  memset(g_work.g_vec, 0, sizeof(g_work.g_vec));

  for (i = 0; i < num; i++)
    {
      g_work.workA = (double)gavgs[i].x;
      g_work.workB = (double)gavgs[i].y;

      g_work.f_mat[0][2] += g_work.workA;
      g_work.f_mat[1][2] += g_work.workB;
      g_work.f_mat[0][0] += SQUARED(g_work.workA);
      g_work.f_mat[1][1] += SQUARED(g_work.workB);
      g_work.f_mat[0][1] += g_work.workA * g_work.workB;
      g_work.g_vec[0]    += CUBE(g_work.workA) + g_work.workA * SQUARED(g_work.workB);
      g_work.g_vec[1]    += CUBE(g_work.workB) + g_work.workB * SQUARED(g_work.workA);
      g_work.g_vec[2]    += SQUARED(g_work.workA) + SQUARED(g_work.workB);
    }

  g_work.f_mat[1][0] = g_work.f_mat[0][1];
  g_work.f_mat[2][0] = g_work.f_mat[0][2];
  g_work.f_mat[2][1] = g_work.f_mat[1][2];
  g_work.f_mat[2][2] = (double)num;

  g_work.g_vec[0] = -g_work.g_vec[0];
  g_work.g_vec[1] = -g_work.g_vec[1];
  g_work.g_vec[2] = -g_work.g_vec[2];

  g_work.workB = DET_MATRIX(g_work.f_mat);

  for (i = 0; i < 3; i++)
    {
      for (r = 0; r < 3; r++)
        {
          for (c = 0; c < 3; c++)
            {
              g_work.tmp_mat[r][c] = (c == i) ? g_work.g_vec[r] : g_work.f_mat[r][c];
            }
        }

      g_work.workA = DET_MATRIX(g_work.tmp_mat);

      g_work.t_vec[i] = g_work.workA / g_work.workB;
    }

  g_work.workA = -g_work.t_vec[0] / 2;
  g_work.workB = -g_work.t_vec[1] / 2;

  bias_out->x = g_work.workA;
  bias_out->y = g_work.workB;

  g_work.workB = SQUARED(g_work.workA) + SQUARED(g_work.workB) - g_work.t_vec[2];
  g_work.workA = 0;

  for (i = 0; i < num; i++)
    {
      g_work.workA += gavgs[i].z;
    }

  bias_out->z = g_work.workA / num;

  return (g_work.workB > 0.0) ? 0 : -1;
}

double calc_device_heading2d(float x, float y)
{
  double deg = atan2(-x, y) * RAD2DEG;

  if (deg < 0)
    {
      deg += 360.0;
    }

  return deg;
}
