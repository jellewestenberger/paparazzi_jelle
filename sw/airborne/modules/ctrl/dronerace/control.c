
#include "std.h"


#include "control.h"

#include "flightplan.h"
#include "ransac.h"
#include <math.h>
#include "state.h"
#include "predictor.h"
// Variables
struct dronerace_control_struct dr_control;

/** The file pointer */
FILE *file_logger_t = NULL;

static void open_log(void) 
{
  char filename[512];
  char filename2[512];
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "lllllog_file");
  sprintf(filename2, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "predict_prop");

  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);
  file_logger_t = fopen(filename, "w+"); 
  prediction_logger_t = fopen(filename2,"w+");
}


// Settings


// Slow speed
#define CTRL_MAX_SPEED  6.0             // m/s
#define CTRL_MAX_PITCH  RadOfDeg(18)    // rad
#define CTRL_MAX_ROLL   RadOfDeg(45)    // rad
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
#define PITCHFIX  -10 * PI/180.0
#define DIRECTION 1 // 1 for clockwise, -1 for counterclockwise
float yaw_direction=1;
void control_run(float dt)
{

  float psi_meas=stateGetNedToBodyEulers_f()->psi;
  float theta_meas=stateGetNedToBodyEulers_f()->theta;
  float phi_meas=stateGetNedToBodyEulers_f()->phi;


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
   float vxb = dr_state.vx*(cosf(theta_meas)*cosf(psi_meas))+dr_state.vy*cosf(theta_meas)*sinf(psi_meas);
   float vyb = dr_state.vx*(sinf(phi_meas)*sinf(theta_meas)*cosf(psi_meas)-cosf(phi_meas)*sinf(psi_meas)) +
    dr_state.vy * (sinf(phi_meas)*sinf(theta_meas)*sinf(psi_meas)+cosf(phi_meas)*cosf(psi_meas)); 

  float dist2target = sqrtf(dr_state.x * dr_state.x + dr_state.y*dr_state.y);
  float absvel = (dr_state.vx * dr_state.vx + dr_state.vy*dr_state.vy);
  float radiuserror = (dist2target-radius_des);


  float phase_angle = atan2f(dr_state.y,dr_state.x); 
  float Rx = radius_des * cosf(phase_angle);
  float Ry = radius_des * sinf(phase_angle);
  float perpx = dr_state.vx; //perpx and perpy are the vector components perpendicular to the radius which should be the ideal airspeed. vx is chosen so that it points in the right direction
  float perpy = (-dr_state.vx * Rx)/Ry;

  float ang1 = acosf(perpx/(sqrtf(1*1)*sqrtf(perpx*perpx+perpy*perpy)));  //angle with x-axis
  float ang2 = acosf((dr_state.vx)/(1*sqrtf(dr_state.vx*dr_state.vx+dr_state.vy*dr_state.vy)));
  float ang = (ang1 - ang2); // angle between current velocity and desired velocity vector 

  float r_error_x=dr_state.x - Rx;// radiuserror * cosf(phase_angle);
  float r_error_y=dr_state.y - Ry;//radiuserror * sinf(phase_angle);

  float rx = r_error_x - dr_state.x; //translate error to body 
  float ry = r_error_y - dr_state.y;

  //rotate error to body
  float rxb = rx*cosf(theta_meas)*cosf(psi_meas)+ry*cosf(theta_meas)*sinf(psi_meas);
  float ryb = rx*(sinf(phi_meas)*sinf(theta_meas)*cosf(psi_meas)-cosf(phi_meas)*sinf(psi_meas)) + ry*(sinf(phi_meas)*sinf(theta_meas)*sinf(psi_meas)+cosf(phi_meas)*cosf(psi_meas));
  

  float centriterm = DIRECTION*atan2f(absvel * cosf(dr_state.theta), (abs(GRAVITY) * dist2target));

 
 // from predictor
  dr_control.phi_cmd = find_roll(vxb,phi_meas,psi_meas,dr_state.x,dr_state.y,0.0,0.0);
  
  dr_control.phi_cmd = bound_angle(dr_control.phi_cmd,CTRL_MAX_ROLL);

  POS_I = POS_I + radiuserror/512.0;
  
  if(ang>(PI*30.0/180.)){
    ang = PI * 30.0/180.;
  }
  if(ang<(PI*-30.0/180.)){
    ang = PI * -30.0/180.;
  }
  if(isnan(ang)){
    ang=0;
  }
  lookI = lookI + ang/512.0;
  

  //from predictor:  
  dr_control.psi_cmd = find_yaw(dr_control.psi_cmd,phi_meas,vxb,512.0); 



  dr_control.theta_cmd = PITCHFIX ;
  dr_control.theta_cmd = bound_angle(dr_control.theta_cmd,CTRL_MAX_PITCH);


  // printf("tx: %f, pos_x: %f, ty: %f, posy: %f, roll_cmd: %f, roll: %f, psi_cmd: %f, psi: %f \n",0,dr_state.x,0,dr_state.y,test_roll*r2d,phi_meas*r2d,dr_control.psi_cmd*r2d,psi_meas*r2d);

  printf("yaw_cmd: %f, yaw_meas: %f\n",dr_control.psi_cmd*r2d,dr_state.psi*r2d);
  
  static int counter = 0;
  // fprintf(file_logger_t, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, %f, %f, %f, %f, %f, %f, %f, %f\n", counter, dr_control.theta_cmd, test_roll*r2d, theta_meas,phi_meas,dr_state.x,dr_state.y, dist2target, phase_angle,rxb,ryb,centriterm, 
  // dr_control.psi_cmd,psi_meas,dr_state.vx,dr_state.vy,rx,ry,radiuserror,ang1,ang2,ang);
  counter++;
  

}
