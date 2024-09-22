/**
 *
 * Crazyflie Gimbal control firmware
 *
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "controller.h"
#include "controller_pid.h"

#define DEBUG_MODULE "GIMBAL"
#include "debug.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

bool gimbal = false;
bool rate = false;
bool relay = false;
bool relay_level = false;
static float relay_error = 0.0f;
static float relay_threshold = 7.0f;
static float relay_cmd = 30.0f;
static float rate_relay_threshold = 25.0f;
static float rate_relay_cmd = 25000.0f;
static float rroll_cmd = 0.0f;
static float rpitch_cmd = 0.0f;
static float ryaw_cmd = 0.0f;

static setpoint_t setpoint_cmd;
static state_t state_process;

void target_pose(float pose[]){}
void enable_formation(){}
void add_new_agent(float name, float pose[], float d, float k) {}
void remove_neighbour(float name){}
void update_distance(float name, float d){}
void update_agent_pose(float name, float x, float y, float z) {}
void controller_update(setpoint_t setpoint, const state_t *state) {}

void attitud_cmd(float roll, float pitch, float yaw, float thrust){
  if(rate){
    rate = false;
    setpoint_cmd.mode.roll = modeAbs;
    setpoint_cmd.mode.pitch = modeAbs;
    setpoint_cmd.mode.yaw = modeAbs;
  }
  setpoint_cmd.attitude.roll  = roll;
  setpoint_cmd.attitude.pitch = pitch;
  setpoint_cmd.attitude.yaw = yaw;
  setpoint_cmd.thrust = thrust;
  commanderSetSetpoint(&setpoint_cmd, 3);
}

void attituderate_cmd(float roll, float pitch, float yaw, float thrust){
  if(!rate){
    rate = true;
    setpoint_cmd.mode.roll = modeVelocity;
    setpoint_cmd.mode.pitch = modeVelocity;
    setpoint_cmd.mode.yaw = modeVelocity;
  }
  setpoint_cmd.attitudeRate.roll  = roll;
  setpoint_cmd.attitudeRate.pitch = pitch;
  setpoint_cmd.attitudeRate.yaw = yaw;
  setpoint_cmd.thrust = thrust;
  commanderSetSetpoint(&setpoint_cmd, 3);
}

float update_attitude_relay(float setpoint, float value){
  relay_error = setpoint - value;
  if(!relay_level){
    if(relay_error>relay_threshold){
      relay_level = true;
      return relay_cmd;
    }else{
      return -relay_cmd;
    }
  }else{
    if(relay_error<-relay_threshold){
      relay_level = false;
      return -relay_cmd;
    }else{
      return relay_cmd;
    }
  }
}

float update_rate_relay(float setpoint, float value){
  relay_error = setpoint - value;
  if(!relay_level){
    if(relay_error>rate_relay_threshold){
      relay_level = true;
      return rate_relay_cmd;
    }else{
      return -rate_relay_cmd;
    }
  }else{
    if(relay_error<-rate_relay_threshold){
      relay_level = false;
      return -rate_relay_cmd;
    }else{
      return rate_relay_cmd;
    }
  }
}

void enable_relay(){
  relay_error = 0.0f;
  if(!relay){
    relay = true;
  }else{
    relay = false;
  }
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  
  DEBUG_PRINT("In progress ...\n");
  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit() {
  // Call the PID controller instead in this example to make it possible to fly
  setpoint_cmd.mode.x = modeDisable;
  setpoint_cmd.mode.y = modeDisable;
  setpoint_cmd.mode.z = modeDisable;
  setpoint_cmd.mode.roll = modeAbs;
  setpoint_cmd.mode.pitch = modeDisable;
  setpoint_cmd.mode.yaw = modeDisable;
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t* setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t stabilizerStep) {
  // Implement your controller here...
  // controller_update(setpoint, state);
  // Call the PID controller instead in this example to make it possible to fly
  // DEBUG_PRINT("State: roll:%.3f, pitch:%.3f, yaw:%.3f\n", (double)state->attitude.roll, (double)state->attitude.pitch, (double)state->attitude.yaw);

  
  // DEBUG_PRINT("Test: ref:%.3f, roll:%.3f\n", (double)setpoint->attitude.roll, (double)state->attitude.pitch);
  // Call the PID controller instead in this example to make it possible to fly
  state_process.attitude.roll = state->attitude.roll;
  state_process.attitude.pitch = state->attitude.pitch;
  state_process.attitude.yaw = state->attitude.yaw;
  actuatorThrust = setpoint_cmd.thrust+10000;
  // controllerPid(control, setpoint, sensors, state, tick);
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    attitudeDesired.roll = setpoint_cmd.attitude.roll;
    attitudeDesired.pitch = setpoint_cmd.attitude.pitch;
    //attitudeDesired.yaw = setpoint_cmd.attitude.yaw;

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);
    // Attitude Relay

    if(false){
      // Roll
      //rateDesired.roll = update_attitude_relay(attitudeDesired.roll, state->attitude.roll);
      //attitudeControllerResetRollAttitudePID();
      
      // Pitch
      //rateDesired.pitch = update_attitude_relay(attitudeDesired.pitch, state->attitude.pitch);
      //attitudeControllerResetPitchAttitudePID();

      // Yaw
      //rateDesired.yaw = update_attitude_relay(attitudeDesired.roll, state->attitude.roll);
      //attitudeControllerResetYawAttitudePID();
    }

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    /*
    
    */
    
    if(relay){
      attitudeControllerCorrectRatePID( sensors->gyro.x, 0.0f, sensors->gyro.z,
                                        rateDesired.roll, 0.0f, rateDesired.yaw);
    
      attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

      control->pitch = update_rate_relay(0.0f, -sensors->gyro.y);
    }else{
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    
      attitudeControllerGetActuatorOutput(&control->roll,
                                          &control->pitch,
                                          &control->yaw);
    }
    
    /*
    pidSetDesired(&pidRollRate, rateDesired.roll);
    control->roll = saturateSignedInt16(pidUpdate(&pidRollRate, sensors->gyro.x, true));

    pidSetDesired(&pidPitchRate, rateDesired.pitch);
    control->pitch = saturateSignedInt16(pidUpdate(&pidPitchRate, -sensors->gyro.y, true));

    pidSetDesired(&pidYawRate, rateDesired.yaw);

    control->yaw = saturateSignedInt16(pidUpdate(&pidYawRate, sensors->gyro.z, true));
    */
    
    
    control->yaw = -control->yaw;
  }

  control->thrust = actuatorThrust;
  rroll_cmd = control->roll;
  rpitch_cmd = control->pitch;
  ryaw_cmd = control->yaw;

  
}

/**
 */
LOG_GROUP_START(rele)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, roll_sp, &setpoint_cmd.attitude.roll)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, pitch_sp, &setpoint_cmd.attitude.pitch)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, yaw_sp, &setpoint_cmd.attitude.yaw)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, roll, &state_process.attitude.roll)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, pitch, &state_process.attitude.pitch)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, yaw, &state_process.attitude.yaw)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, rate_roll_sp, &rateDesired.roll)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, rate_pitch_sp, &rateDesired.pitch)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_yaw_sp, &relay_error)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, rate_roll_cmd, &rroll_cmd)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, rate_pitch_cmd, &rpitch_cmd)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_yaw_cmd, &ryaw_cmd)

LOG_GROUP_STOP(rele)

