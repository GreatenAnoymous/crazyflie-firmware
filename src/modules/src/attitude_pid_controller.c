/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * attitude_pid_controller.c: Attitude controler using PID correctors
 */
#include <stdbool.h>

#include "FreeRTOS.h"

#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "math3d.h"

#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 40.0f
#define ATTITUDE_RATE_LPF_ENABLE false

#define HOVER_MODE_MAX_ATTITUDE_RATE 150.0f

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static float dt;

static bool isInit;
static bool agg_mode;
static bool pos_mode;

void attitudeControllerInit(const float updateDt)
{
  if(isInit)
    return;

  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

  pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  updateDt,
      ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, updateDt,
      ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  pidInit(&pidYaw,   0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   updateDt,
      ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

  pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);

  dt=updateDt;
  isInit = true;
}

bool attitudeControllerTest()
{
  return isInit;
}

void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

  pidSetDesired(&pidYawRate, yawRateDesired);
  yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
}

float clampf(float u,float minf,float maxf){
  if(u>maxf) return maxf;
  if(u<minf) return minf;
  return u;

}

void attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
  // if(pos_mode){
  //   //*yawRateDesired=clampf(*yawRateDesired,-HOVER_MODE_MAX_ATTITUDE_RATE,HOVER_MODE_MAX_ATTITUDE_RATE);
  //   *rollRateDesired=clampf(*rollRateDesired,-HOVER_MODE_MAX_ATTITUDE_RATE,HOVER_MODE_MAX_ATTITUDE_RATE);
  //   *pitchRateDesired=clampf(*pitchRateDesired,-HOVER_MODE_MAX_ATTITUDE_RATE,HOVER_MODE_MAX_ATTITUDE_RATE);
  // }
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidRoll);

}

void attitudeControllerAgressiveMode(void){
  if(agg_mode) return;
    pidReset(&pidPitch);
    pidReset(&pidRoll);
    pidReset(&pidYaw);
    pidInit(&pidPitchRate,  0, 500,  0,  5,
      dt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
    pidInit(&pidRollRate,  0, 250,  0,  10,
      dt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
    pidInit(&pidYawRate,  0, 250,  0,  10,
      dt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

  agg_mode=true;
  pos_mode=false;
}

void attitudeControllerPositionMode(void){
if(pos_mode) return;
  pidReset(&pidPitch);
  pidReset(&pidRoll);
  pidReset(&pidYaw);
  pidInit(&pidRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
      dt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
      dt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
      dt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pos_mode=true;
  agg_mode=false;
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidPitch);
    
}

void attitudeControllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  if(agg_mode){
    // *roll = clamp(rollOutput,-10920,10920);
    // *pitch = clamp(pitchOutput,-32767,32767);
    // *yaw = clamp(yawOutput,-10920,10920);
    *roll = rollOutput;
    *pitch = pitchOutput;
    *yaw = yawOutput;
  }
  else{
    *roll = rollOutput;
    *pitch = pitchOutput;
    *yaw = yawOutput;
  }
 
}

LOG_GROUP_START(pid_attitude)
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.outP)
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.outI)
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.outD)
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.outP)
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.outI)
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.outD)
LOG_GROUP_STOP(pid_attitude)

LOG_GROUP_START(pid_rate)
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.outP)
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.outI)
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.outD)
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.outP)
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.outI)
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.outD)
LOG_GROUP_STOP(pid_rate)

PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)
