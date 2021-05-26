/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
// #include "math.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}
// static inline float max(float x, float y){
//   if(x>y) return x;
//   return y;
// }
// static inline float min(float x, float y){
//   if(x<y) return x;
//   return y;
// }
void powerDistribution(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;

    // float m1 = control->thrust - r + p + control->yaw;
    // float m2 = control->thrust - r - p - control->yaw; 
    // float m3 = control->thrust - r - p - control->yaw;
    // float m4 = control->thrust + r + p - control->yaw;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
    // float ma = max(max(m1, m2), max(m3, m4));
    // float mi = min(min(m1, m2), min(m3, m4));
    // if(ma <= 65535 && mi >= 0){
    //   motorPower.m1 = m1;
    //   motorPower.m2 = m2;
    //   motorPower.m3 = m3;
    //   motorPower.m4 = m4;
    // }else if(mi > 0){
    //   motorPower.m1 = m1 * 65535 / ma;
    //   motorPower.m2 = m2 * 65535 / ma;
    //   motorPower.m3 = m3 * 65535 / ma;
    //   motorPower.m4 = m4 * 65535 / ma;
    // }else{ 
    //   if(ma < 0){
    //     motorPower.m1=0;
    //     motorPower.m2=0;
    //     motorPower.m3=0;
    //     motorPower.m4=0;
    //   }else if(ma<=65535){
    //     motorPower.m1 = ma - (ma-m1) / (ma-mi) * ma;
    //     motorPower.m2 = ma - (ma-m2) / (ma-mi) * ma;
    //     motorPower.m3 = ma - (ma-m3) / (ma-mi) * ma;
    //     motorPower.m4 = ma - (ma-m4) / (ma-mi) * ma;
    //   }else{
    //     motorPower.m1 = (m1 - mi) * (65535) / (ma -mi);
    //     motorPower.m2 = (m2 - mi) * (65535) / (ma -mi);
    //     motorPower.m3 = (m3 - mi) * (65535) / (ma -mi);
    //     motorPower.m4 = (m4 - mi) * (65535) / (ma -mi);

    //   }
    //   // else if(ma > 65535)
    // }
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
