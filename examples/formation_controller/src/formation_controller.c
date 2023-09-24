/**
 *
 * Crazyflie Formation control firmware
 *
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "controller.h"
#include "controller_pid.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FORMATION_CONTROLLER"
#include "debug.h"

#define MAX_AGENTS 16

typedef struct {
    char id[20];
    float d;
    float pose[3];
} agent;

agent agent_list[MAX_AGENTS];
int num_agents = 0;

void add_new_agent(char* name, float pose[], float d) {
    if (num_agents >= MAX_AGENTS) {
        DEBUG_PRINT("Cannot add new agent. Maximum number of agents reached.\n");
        return;
    }

    strcpy(agent_list[num_agents].id, name);
    memcpy(agent_list[num_agents].pose, pose, sizeof(float)*3);
    agent_list[num_agents].d = d;

    num_agents++;
}

void update_agent_pose(char* name, float new_pose[]) {
    int i;
    for (i = 0; i < num_agents; i++) {
        if (strcmp(agent_list[i].id, name) == 0) {
            memcpy(agent_list[i].pose, new_pose, sizeof(float)*3);
            return;
        }
    }

    DEBUG_PRINT("Could not find agent with id %s\n", name);
}

void controller_update(setpoint_t* setpoint, const state_t *state) {
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;
    float error_x = 0.0;
    float error_y = 0.0;
    float error_z = 0.0;
    float error_r = 0.0;
    float distance = 0.0;
    
    for (int i = 0; i < num_agents; i++) {
      error_x = state->position.x - agent_list[i].pose[0];
      error_y = state->position.y - agent_list[i].pose[1];
      error_z = state->position.z - agent_list[i].pose[2];
      distance = powf(error_x,2)+powf(error_y,2)+powf(error_z,2);
      dx += (powf(agent_list[i].d,2) - distance) * error_x;
      dy += (powf(agent_list[i].d,2) - distance) * error_y;
      dz += (powf(agent_list[i].d,2) - distance) * error_z;
    }

    error_r = powf(1.0,2) - (powf(state->position.x,2)+powf(state->position.y,2)+powf(state->position.z-0.5f,2));
    dx += 2 * (error_r * state->position.x);
    dy += 2 * (error_r * state->position.y);
    dz += 2 * (error_r * (state->position.z-0.5f));

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

    setpoint->position.x = state->position.x + dx/4.0f;
    setpoint->position.y = state->position.y + dy/4.0f;
    setpoint->position.z = state->position.z + dz/4.0f;
}


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");
  float pose1[3] = {0.0, -0.2, 1.0};
  float pose2[3] = {0.0, 0.2, 1.0};

  add_new_agent("agent1", pose1, 0.5);
  add_new_agent("agent2", pose2, 0.5);

  DEBUG_PRINT("Initial agent list:\n");
  for (int i = 0; i < num_agents; i++) {
    DEBUG_PRINT("%s: (%.3f, %.3f, %.3f) d=%.3f\n", agent_list[i].id,
            (double)agent_list[i].pose[0], (double)agent_list[i].pose[1], (double)agent_list[i].pose[2],
            (double)agent_list[i].d);
  }

  float new_pose[3] = {0.0, 0.8, 1.0};
  update_agent_pose("agent2", new_pose);
  
  DEBUG_PRINT("Agent list after updating agent2's pose:\n");
  for (int i = 0; i < num_agents; i++) {
      DEBUG_PRINT("%s: (%.3f, %.3f, %.3f) d=%.3f\n", agent_list[i].id, 
             (double)agent_list[i].pose[0], (double)agent_list[i].pose[1], (double)agent_list[i].pose[2],
             (double)agent_list[i].d);
  }
  DEBUG_PRINT("Update fake agent's pose:\n");
  update_agent_pose("agent3", new_pose);

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit() {
  // Call the PID controller instead in this example to make it possible to fly
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t* setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Implement your controller here...
  // controller_update(setpoint, state);
  // Call the PID controller instead in this example to make it possible to fly
  DEBUG_PRINT("SETPOINT: (X:%.3f, Y:%.3f, Z:%.3f)\n", (double)setpoint->position.x, (double)setpoint->position.y, (double)setpoint->position.z);
  controllerPid(control, setpoint, sensors, state, tick);
}

