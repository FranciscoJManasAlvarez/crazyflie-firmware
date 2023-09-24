/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "log.h"
#include "param.h"

#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"

#include "controller.h"
#include "controller_pid.h"

#define DEBUG_MODULE "MULTIROBOTSYSTEM"
#include "debug.h"

#define MAX_AGENTS 16

typedef struct {
    char id[20];
    float idn;
    float k;
    float d;
    float pose[3];
} agent;

agent agent_list[MAX_AGENTS];
int num_agents = 0;

float ref_pose[3];
bool goal_pose = false;
bool formation = false;
static setpoint_t multi_agent_setpoint;

void target_pose(float pose[]){
  setpoint_t setpoint;

  setpoint.mode.z = modeAbs;
  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.position.x = pose[0];
  setpoint.position.y = pose[1];
  setpoint.position.z = pose[2];
  memcpy(agent_list[num_agents].pose, pose, sizeof(float)*3);
  formation = false;
  ref_pose[0] = pose[0];
  ref_pose[1] = pose[1];
  ref_pose[2] = pose[2];
  goal_pose = true;
  commanderSetSetpoint(&setpoint, 3);
}
void enable_formation(){
  formation = true;
}

void add_new_agent(float name, float pose[], float d, float k) {
    if (num_agents >= MAX_AGENTS) {
        DEBUG_PRINT("Cannot add new agent. Maximum number of agents reached.\n");
        return;
    }

    // strcpy(agent_list[num_agents].id, name);
    agent_list[num_agents].idn = name;
    memcpy(agent_list[num_agents].pose, pose, sizeof(float)*3);
    agent_list[num_agents].d = d;
    agent_list[num_agents].k = k;

    num_agents++;
}

void update_agent_pose(float name, float x, float y, float z) {
    int i;
    float new_pose[3] = {x, y, z};
    for (i = 0; i < num_agents; i++) {
        if (agent_list[i].idn == name) {
            memcpy(agent_list[i].pose, new_pose, sizeof(float)*3);
            return;
        }
    }

    // DEBUG_PRINT("Could not find agent with id %.0f\n", name);
}

void controller_update(setpoint_t setpoint, const state_t *state) {
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;
    float error_x = 0.0;
    float error_y = 0.0;
    float error_z = 0.0;
    float distance = 0.0;
    
    for (int i = 0; i < num_agents; i++) {
      error_x = state->position.x - agent_list[i].pose[0];
      error_y = state->position.y - agent_list[i].pose[1];
      error_z = state->position.z - agent_list[i].pose[2];
      distance = powf(error_x,2)+powf(error_y,2)+powf(error_z,2);
      dx += agent_list[i].k * (powf(agent_list[i].d,2) - distance) * error_x;
      dy += agent_list[i].k * (powf(agent_list[i].d,2) - distance) * error_y;
      dz += agent_list[i].k * (powf(agent_list[i].d,2) - distance) * error_z;
    }

    if(dx >  0.32f) 
      dx = 0.32f;
    if(dx < -0.32f) 
      dx = -0.32f;
    if(dy >  0.32f) 
      dy = 0.32f;
    if(dy < -0.32f) 
      dy = -0.32f;
    if(dz >  0.32f) 
      dz = 0.32f;
    if(dz < -0.32f) 
      dz = -0.32f;

    setpoint.mode.z = modeAbs;
    setpoint.mode.x = modeAbs;
    setpoint.mode.y = modeAbs;
    setpoint.position.x = state->position.x + dx/4.0f;
    setpoint.position.y = state->position.y + dy/4.0f;
    setpoint.position.z = state->position.z + dz/4.0f;

    // setpoint.position.z = 1.0;

    if(setpoint.position.z > 2.0f){
      setpoint.position.z = 2.0f;
    }
    
    commanderSetSetpoint(&setpoint, 3);
}


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  while(1) {
    vTaskDelay(M2T(1000));
    // DEBUG_PRINT("Test MRS!\n");
  }
}

void controllerOutOfTreeInit() {
  // Initialize your controller data here...

  // Call the PID controller instead in this example to make it possible to fly
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...
  if (RATE_DO_EXECUTE(20, tick)) {
    if(formation){
      controller_update(multi_agent_setpoint, state);
      goal_pose = false;
    }else{
      if(goal_pose){
        target_pose(ref_pose);
      }
    }
    
    multi_agent_setpoint.position.x = setpoint->position.x;
    multi_agent_setpoint.position.y = setpoint->position.y;
    multi_agent_setpoint.position.z = setpoint->position.z;

    // DEBUG_PRINT("state: (X:%.3f, Y:%.3f, Z:%.3f)\n", (double)state->position.x, (double)state->position.y, (double)state->position.z);
    // DEBUG_PRINT("SETPOINT: (X:%.3f, Y:%.3f, Z:%.3f)\n", (double)setpoint->position.x, (double)setpoint->position.y, (double)setpoint->position.z);
  }
  // Call the PID controller instead in this example to make it possible to fly
  controllerPid(control, setpoint, sensors, state, tick);
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(multirobot)
/**
 * @brief Setpoint_x
 */
LOG_ADD(LOG_FLOAT, cmd_x, &multi_agent_setpoint.position.x)
/**
 * @brief Setpoint_y
 */
LOG_ADD(LOG_FLOAT, cmd_y, &multi_agent_setpoint.position.y)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_FLOAT, cmd_z, &multi_agent_setpoint.position.z)
/**
 * @brief Setpoint_z
 */
LOG_ADD(LOG_UINT16, n, &num_agents)
LOG_GROUP_STOP(multirobot)
