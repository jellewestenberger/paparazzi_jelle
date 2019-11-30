
#include "std.h"


#include "control.h"
#include "predictor.h"
#include "flightplan.h"
#include "ransac.h"
#include <math.h>
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "predict_thread.h"
// #include "predictor.h"
// Variables
struct dronerace_control_struct dr_control;
volatile struct predict_input pred_inputs; 


/** The file pointer */
FILE *file_logger_t = NULL;
FILE *predic_logger = NULL;

static void open_log(void) 
{
  char filename[512];
  char filename2[512];
  char filename3[512];
  
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "lllllog_file");
  sprintf(filename2, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "predict_prop");
  sprintf(filename3, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "predict_debug");

  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);
  file_logger_t = fopen(filename, "w+"); 
  predic_prop_t = fopen(filename2,"w+");
  predic_logger = fopen(filename3, "w+"); 
  // prediction_logger_t = fopen(filename2,"w+");
}


// Settings


// Slow speed
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(18)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(35)    // rad
#define CTRL_MAX_R      RadOfDeg(90)    // rad/sec

float bound_angle(float angle, float max_angle){
  if(angle>max_angle){
    angle=max_angle;
  }
  if(angle<(-max_angle)){
    angle=-max_angle;
  }
  return angle;
}

void control_reset(void)
{ 
  POS_I = 0.0; 
  lookI = 0.0;
  // Reset flight plan logic
  flightplan_reset();
  open_log();
  // Reset own variables
  dr_control.psi_ref = 0;
  dr_control.psi_cmd = 0;
}

static float angle180(float r)
{
  if (r < RadOfDeg(-180))
  {
    r += RadOfDeg(360.0f);
  }
  else if (r > RadOfDeg(180.0f))
  {
    r -= RadOfDeg(360.0f);
  }

  return r;
}


float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}
#define KP_look 0.1
#define KI_look 0.0 
#define KP_POS  0.2
#define KI_POS 0.02
#define KP_VEL_X  0.1
#define KP_VEL_Y  0.1
#define KD_VEL_X  0.05
#define KD_VEL_Y  0.05
#define radius_des 2
float lookahead = 25 * PI/180.0;
#define PITCHFIX  -5 * PI/180.0
#define DIRECTION 1 // 1 for clockwise, -1 for counterclockwise
float posx_old = 1234234;
float posy_old = 1123;
float yaw_direction=1;
float phi_meas_avg; 
float psi_meas_avg; 
float avg_cntr=1;
bool avged = false; 
void control_run(float dt)
{

  float psi_meas=stateGetNedToBodyEulers_f()->psi;
  float theta_meas=stateGetNedToBodyEulers_f()->theta;
  float phi_meas=stateGetNedToBodyEulers_f()->phi;
  // printf("phi: %f, theta: %f, psi: %f\n",phi_meas*r2d,theta_meas*r2d,psi_meas*r2d);


  dr_control.z_cmd = dr_fp.z_set;

  // outer loop velocity control

  struct NedCoor_f *pos_gps = stateGetPositionNed_f();
  dr_state.x = pos_gps->x;
  dr_state.y = pos_gps->y;
  dr_state.z = pos_gps->z;

   struct NedCoor_f *vel_gps = stateGetSpeedNed_f();
   dr_state.vx = vel_gps->x; // In earth reference frame
   dr_state.vy = vel_gps->y;
   
   // transform  velocity to body frame 
  //  float vxb = dr_state.vx*(cosf(theta_meas)*cosf(psi_meas))+dr_state.vy*cosf(theta_meas)*sinf(psi_meas);
   float vxb_plane = dr_state.vx*(cosf(psi_meas))+dr_state.vy*sinf(psi_meas); //body vx in plane of circle 


 
 // from predictor

   // import
   
   if (hasresult)
   {
    // printf("roll from thread: %f",optimized_roll*r2d);
     hasresult = false;
   }
   

  // export -> t
  if((dr_state.x!=posx_old)&&(dr_state.y!=posy_old)){ //only run optimizer if a new position reading is available
  pred_inputs.v = vxb_plane;
  pred_inputs.xi = dr_state.x; 
  pred_inputs.yi = dr_state.y;
  pred_inputs.tx = waypoints_circle[wp_id].wp_x;
  pred_inputs.ty = waypoints_circle[wp_id].wp_y;
  if(avged){
    pred_inputs.phi = phi_meas_avg; 
    pred_inputs.psi = psi_meas_avg; 
    avged = false; 
    
  }
  else{
    pred_inputs.phi = phi_meas;
    pred_inputs.psi = psi_meas;
  }
  work = true;
  // printf("optimized roll: %f\n",optimized_roll);
  optimized_roll=bound_angle(optimized_roll,CTRL_MAX_ROLL);
  dr_control.phi_cmd  = optimized_roll;
  if(gate_reset){
    pred_inputs.range_a = - CTRL_MAX_ROLL;
    pred_inputs.range_b = CTRL_MAX_ROLL;
  }
  else{
  pred_inputs.range_a = optimized_roll - 3 * d2r;
  pred_inputs.range_b = optimized_roll + 3 * d2r;
  }
  posx_old=dr_state.x;
  posy_old=dr_state.y;
  
  phi_meas_avg=0;
  psi_meas_avg=0;
  avg_cntr = 0; 
  }
  else{ //if no new position measurements are available use this loop to average the roll measurements; 
    phi_meas_avg = ((phi_meas_avg*(avg_cntr))+phi_meas)/(avg_cntr+1); 
    psi_meas_avg =((psi_meas_avg*(avg_cntr))+psi_meas)/(avg_cntr+1); 
    printf("phi_meas_avg: %f , phi_meas: %f \n psi_meas_avg: %f, psi_meas: %f\n avg_cntr: %f\n",phi_meas_avg,phi_meas, psi_meas_avg,psi_meas,avg_cntr);
    avg_cntr+=1.0;
    avged = true;
  }
  
  

  dr_control.phi_cmd = bound_angle(dr_control.phi_cmd,CTRL_MAX_ROLL);

  

  //from predictor:  
  dr_control.psi_cmd = find_yaw(dr_control.psi_cmd,phi_meas,vxb_plane,(1.0/512.0)); 



  dr_control.theta_cmd = PITCHFIX ;
  dr_control.theta_cmd = bound_angle(dr_control.theta_cmd,CTRL_MAX_PITCH);


  // printf("tx: %f, pos_x: %f, ty: %f, posy: %f, roll_cmd: %f, roll: %f, psi_cmd: %f, psi: %f \n",0,dr_state.x,0,dr_state.y,test_roll*r2d,phi_meas*r2d,dr_control.psi_cmd*r2d,psi_meas*r2d);

  static int counter = 0;
  fprintf(predic_logger, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", get_sys_time_float(),dr_state.x,dr_state.y,dr_control.phi_cmd,dr_control.theta_cmd,dr_control.psi_cmd, phi_meas, theta_meas, psi_meas, dr_state.vx, dr_state.vy,vxb_plane);
  counter++;
  

}
