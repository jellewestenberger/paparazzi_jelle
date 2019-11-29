/*
 * Copyright (C) MAVLab
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/ctrl/dronerace//dronerace.c"
 * @author MAVLab
 * Autonomous Drone Race
 */


#include "modules/ctrl/dronerace/dronerace.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"
#include "state.h"
#include "filter.h"
#include "control.h"
#include "ransac.h"
#include "flightplan.h"
#include "subsystems/imu.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "predictor.h"
#include "boards/bebop/actuators.h"
#include "pthread.h"
#include "predict_thread.h"

//initialize predict variables:

// #include "predictor.h"

// to know if we are simulating:
#include "generated/airframe.h"

#define MAXTIME 8.0
#define FILE_LOGGER_PATH /data/ftp/internal_000

float dt = 1.0f / 512.f;

float input_phi;
float input_theta;
float input_psi;


volatile int input_cnt = 0;
volatile float input_dx = 0;
volatile float input_dy = 0;
volatile float input_dz = 0;

#include <stdio.h>
// Variables
struct dronerace_control_struct dr_control;


// float est_state_roll = 0;
// float est_state_pitch = 0;
// float est_state_yaw = 0;

struct FloatEulers *rot; 
struct NedCoor_f *pos;   
struct FloatRates *rates;
int wp_id;
float posx;
float posy;
float posz;
float dist2target; 
///////////////////////////////////////////////////////////////////////////////////////////////////
// TELEMETRY


// sending the divergence message to the ground station:
static void send_dronerace(struct transport_tx *trans, struct link_device *dev)
{
  
  // float est_roll = est_state_roll*(180./3.1416);
  // float est_pitch = est_state_pitch*(180./3.1416);
  // float est_yaw = est_state_yaw*(180./3.1416);

  //float ez = POS_FLOAT_OF_BFP(guidance_v_z_sp); 
  // pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID,&est_roll,&est_pitch,&est_yaw);
}


// ABI receive gates!
#ifndef DRONE_RACE_ABI_ID
// Receive from: 33 (=onboard vision) 34 (=jevois) or 255=any
#define DRONE_RACE_ABI_ID ABI_BROADCAST
#endif

static abi_event gate_detected_ev;
float test0 =0;
float est_psi;
static void send_alphapahrs(struct transport_tx *trans, struct link_device *dev)
{
  
 

 pprz_msg_send_AHRS_ALPHAPILOT(trans, dev, AC_ID,&test0,&test0,&est_psi,&test0,&test0,&test0,&test0,&test0,&test0,&posx,&posy,&posz);

}




static void gate_detected_cb(uint8_t sender_id __attribute__((unused)), int32_t cnt, float dx, float dy, float dz, float vx __attribute__((unused)), float vy __attribute__((unused)), float vz __attribute__((unused)))
{
  // Logging
  input_cnt = cnt;
  input_dx = dx;
  input_dy = dy;
  input_dz = dz;

}

// MT

// Input
// volatile struct predict_input; //initialized in control.c
// Output
volatile bool work;
volatile bool hasresult;
volatile float optimized_roll;
 
int it= 0; 
void* main2(void* p) {
  work = false;
  hasresult=false;
  // 
  while (true)
  {

    if (work) {
        hasresult = true;
        optimized_roll=find_roll(pred_inputs);
        hasresult= true;
        work = false;
      
    }
    
   
  // Sleep
  sys_time_usleep(100);
  }
}

void dronerace_init(void)
{ 
  wp_id = 0; 
  // Receive vision
  //AbiBindMsgRELATIVE_LOCALIZATION(DRONE_RACE_ABI_ID, &gate_detected_ev, gate_detected_cb);
  POS_I = 0;
  lookI = 0;
  pthread_t tid; 
  // Send telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW_HOVER, send_dronerace);
  register_periodic_telemetry(DefaultPeriodic,  PPRZ_MSG_ID_AHRS_ALPHAPILOT, send_alphapahrs);

  pthread_create(&tid, NULL, main2, NULL);
 //reset integral
  // Compute waypoints
  dronerace_enter();
  
}

bool start_log = 0;
float psi0 = 0;
void dronerace_enter(void)
{
   wp_id=0;
  psi0 = stateGetNedToBodyEulers_f()->psi;
  dr_control.psi_cmd=psi0;
  filter_reset();
  control_reset();
  printf("Wp enter: %d\n",wp_id);
}


void dronerace_periodic(void)
{  
  // printf("PSI0: %f  ",psi0);
  est_psi = stateGetNedToBodyEulers_f()->psi;
  struct NedCoor_f *pos_gps = stateGetPositionNed_f();
  input_phi   = stateGetNedToBodyEulers_f()->phi;
  input_theta = stateGetNedToBodyEulers_f()->theta;
  input_psi   = stateGetNedToBodyEulers_f()->psi - psi0;

  dr_state.phi   = input_phi;
  dr_state.psi   = input_psi;
  dr_state.theta = input_theta;
    posx = pos_gps->x;
    posy = pos_gps->y;
    posz = pos_gps->z; 
    dist2target = sqrtf((posx-waypoints_circle[wp_id].wp_x)*(posx-waypoints_circle[wp_id].wp_x)+(posy-waypoints_circle[wp_id].wp_y)*(posy-waypoints_circle[wp_id].wp_y));

    if(dist2target<=1){
      if(wp_id<(MAX_GATES-1)){
        wp_id+=1; 
      }
      else{
        wp_id=0; 
      }
      pred_inputs.range_a= -30*3.145/180.0;
      pred_inputs.range_b= 30*3.145/180.0;
      printf("t: %f, new wp: %d\n",get_sys_time_float(),wp_id);
    }
    
  filter_predict(input_phi, input_theta, input_psi, dt);

  
  struct NedCoor_f target_ned;
  target_ned.x = dr_fp.gate_y;
  target_ned.y = dr_fp.gate_x;
  target_ned.z = -dr_fp.gate_z;

  if (autopilot.mode_auto2 == AP_MODE_MODULE) {
    ENU_BFP_OF_REAL(navigation_carrot, target_ned);
    ENU_BFP_OF_REAL(navigation_target, target_ned);
  }
}

void dronerace_set_rc(UNUSED float rt, UNUSED float rx, UNUSED float ry, UNUSED float rz)
{
}

void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi_cmd)
{

  control_run(dt);
  
  *phi     = dr_control.phi_cmd;
  *theta   = dr_control.theta_cmd;
  *psi_cmd = dr_control.psi_cmd;// + psi0;
  *alt     = - dr_control.z_cmd; 
  // guidance_v_z_sp = POS_BFP_OF_REAL(dr_control.z_cmd);
}