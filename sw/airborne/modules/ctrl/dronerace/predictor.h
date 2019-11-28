

#define PI 3.14159265359
#define d2r  PI/180.0 
#define r2d 1.0/d2r
#define GR (1.0 + sqrtf(5))/2.0

#define MAX_ROLL_RATE 50.0*d2r


// FILE *prediction_logger_t= NULL;
struct predict_input findroll_input; 

float predict(float roll_cmd, float xi, float yi,float v, float tx,float ty, float phi, float psi);

float find_roll(struct predict_input findroll_input);

float find_yaw(float psi_cmd,float phi,float v,float drone_dt);
