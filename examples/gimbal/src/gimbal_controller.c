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
#include <stdlib.h>

#include "app.h"
#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "platform_defaults.h"

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
struct threshold roll_rate_threshold, pitch_rate_threshold;


bool gimbal = false;
bool rate = false;
bool relay = false;
bool relay_level = false;
static int idx = 0;
static float relay_error = 0.0f;
static float relay_threshold = 11.0f;
static float relay_cmd = 10.0f;
static float rate_relay_threshold = 2.0f;
static float rate_relay_cmd = 10000.0f;
static float rate_cmd_eq[200];
static float cmd_eq = 0.0f;
static float rroll_cmd = 0.0f;
static float rpitch_cmd = 0.0f;
static float ryaw_cmd = 0.0f;
static float rroll_count = 0.0f;
static float rpitch_count = 0.0f;

static setpoint_t setpoint_cmd;
static state_t state_process;

void target_pose(float pose[]){}
void enable_formation(){}
void add_new_agent(float name, float pose[], float d, float k) {}
void remove_neighbour(float name){}
void update_distance(float name, float d){}
void update_agent_pose(float name, float x, float y, float z) {}
void controller_update(setpoint_t setpoint, const state_t *state) {}


static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;

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

PidObject pidRollRate_rp = {
  .kp = PID_ROLL_RATE_KP,
  .ki = PID_ROLL_RATE_KI,
  .kd = PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
};

PidObject pidPitchRate_rp = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
};

PidObject pidYawRate_rp = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};

struct threshold init_triggering(float co, float ai)
{
  struct threshold  trigger;

  trigger.co = co;
  trigger.ai = ai;
  trigger.count = 1;
  trigger.last_signal = 0.0f;
  trigger.dt = 0.0f;

  return trigger;
}

bool eval_threshold(struct threshold *trigger, float signal, float ref)
{
  // Noise - Cn
	float mean = signal/20.0f;
	for(int i = 0; i<19; i++){
		trigger->noise[i+1] = trigger->noise[i];
		mean += trigger->noise[i]/20.0f;
	}
	trigger->noise[0] = signal;
	trigger->cn = 0.0;
	for(int i = 0; i<20; i++){
		if(abs(trigger->noise[i]-mean) > trigger->cn)
			trigger->cn = trigger->noise[i]-mean;
	}
	// a
	float a = trigger->ai * abs(signal - ref);
	if (a>trigger->ai)
		a = trigger->ai;
	// Threshold
	float th = trigger->co + a + trigger->cn;
	float inc = abs(abs(ref-signal) - trigger->last_signal);

	// Delta Error
	if (inc >= th){
		trigger->last_signal = abs(ref-signal);
    // trigger->count = 1;
		return true;
	}
  trigger->count += 1;
	return false;
}


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
      return rate_relay_cmd+cmd_eq;
    }else{
      return -rate_relay_cmd+cmd_eq;
    }
  }else{
    if(relay_error<-rate_relay_threshold){
      relay_level = false;
      return -rate_relay_cmd+cmd_eq;
    }else{
      return rate_relay_cmd+cmd_eq;
    }
  }
}

void enable_relay(){
  relay_error = 0.0f;
  if(!relay){
    relay = true;
    for (size_t i = 0; i < 200; i++)
    {
      cmd_eq = cmd_eq + rate_cmd_eq[i];
    }
    
  }else{
    relay = false;
    cmd_eq = 0.0f;
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
  roll_rate_threshold = init_triggering(0.5f, 0.1);
  pitch_rate_threshold = init_triggering(0.5f, 0.1);
  pidInit(&pidRollRate_rp,  0, pidRollRate_rp.kp,  pidRollRate_rp.ki,  pidRollRate_rp.kd,
       pidRollRate_rp.kff,  0.002f, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&pidPitchRate_rp, 0, pidPitchRate_rp.kp, pidPitchRate_rp.ki, pidPitchRate_rp.kd,
       pidPitchRate_rp.kff, 0.002f, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&pidYawRate_rp,   0, pidYawRate_rp.kp,   pidYawRate_rp.ki,   pidYawRate_rp.kd,
       pidYawRate_rp.kff,   0.002f, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&pidRollRate_rp,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate_rp, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate_rp,   PID_YAW_RATE_INTEGRATION_LIMIT);

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
    // Cambiar "false" por "relay" para ejecutar el relé en el Attitude controller
    if(false){
      // Roll
      rateDesired.roll = update_attitude_relay(attitudeDesired.roll, state->attitude.roll);
      attitudeControllerResetRollAttitudePID();
      
      // Pitch
      // rateDesired.pitch = update_attitude_relay(attitudeDesired.pitch, state->attitude.pitch);
      // attitudeControllerResetPitchAttitudePID();

      // Yaw
      //rateDesired.yaw = update_attitude_relay(attitudeDesired.roll, state->attitude.roll);
      //attitudeControllerResetYawAttitudePID();
    }

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity || rate) {
      rateDesired.roll = setpoint_cmd.attitudeRate.roll; //setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity || rate) {
      rateDesired.pitch = setpoint_cmd.attitudeRate.pitch; // setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }
    // rateDesired.pitch = setpoint_cmd.attitudeRate.pitch;
    // rateDesired.roll = setpoint_cmd.attitudeRate.roll;


    // Cambiar "false" por "relay" para ejecutar el relé en el Rate controller
    if(relay){
      attitudeControllerCorrectRatePID( sensors->gyro.x, 0.0f, sensors->gyro.z, rateDesired.roll, 0.0f, rateDesired.yaw);
      // attitudeControllerCorrectRatePID( 0.0f, -sensors->gyro.y, sensors->gyro.z, 0.0f, rateDesired.pitch, rateDesired.yaw);    
      attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

      control->pitch = update_rate_relay(0.0f, -sensors->gyro.y);
      //control->roll = update_rate_relay(0.0f, sensors->gyro.x);
    }else{
      /*
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    
      attitudeControllerGetActuatorOutput(&control->roll,
                                          &control->pitch,
                                          &control->yaw);
      */
      if(eval_threshold(&roll_rate_threshold, sensors->gyro.x, rateDesired.roll))
      {
        pidSetDesired(&pidRollRate_rp, rateDesired.roll);
        pidRollRate_rp.dt = 0.002f * roll_rate_threshold.count;
        control->roll = saturateSignedInt16(pidUpdate(&pidRollRate_rp, sensors->gyro.x, true));
        roll_rate_threshold.count = 1;
      }
      else
      {
        control->roll = rroll_cmd;
      }
      if(eval_threshold(&pitch_rate_threshold, -sensors->gyro.y, rateDesired.pitch))
      {
        pidSetDesired(&pidPitchRate_rp, rateDesired.pitch);
        pidPitchRate_rp.dt = 0.002f * pitch_rate_threshold.count;
        control->pitch = saturateSignedInt16(pidUpdate(&pidPitchRate_rp, -sensors->gyro.y, true));
        pitch_rate_threshold.count = 1;
      }
      else
      {
        control->pitch = rpitch_cmd;
      }
    }    
    control->yaw = 0.0f; //-control->yaw;
  }

  control->thrust = actuatorThrust;
  rroll_cmd = control->roll;
  rpitch_cmd = control->pitch;
  ryaw_cmd = control->yaw;
  rroll_count = roll_rate_threshold.count;
  rpitch_count = pitch_rate_threshold.count;

  rate_cmd_eq[idx] = rpitch_cmd/200.0f;
  if(idx == 199){
    idx = 0;
  }else{
    idx = idx + 1;
  }  
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
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_roll_count, &rroll_count)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, rate_pitch_count, &rpitch_count)

LOG_GROUP_STOP(rele)

