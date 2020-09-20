#include "bangbang.h"
#include <math.h>
#include "compensation.h"
#include "std.h"
#include "filter.h"
#include "subsystems/datalink/telemetry.h"
#include "../ctrl_module_outerloop_demo.h"
// #include "state.h"
#define SATURATION 0
#define SECOND 1
#define r2d 180./M_PI
#define d2r M_PI/180.0f
#define LOG
#define FORCESATDIM 0
// #define USETHRUSTALTCTRL
// float m = 0.42; //bebop mass [kg]
float g = 9.80665;//gravity
int type; 
bool brake = false;
float dtt = 1.0f / 512.f;



float bang_ctrl[3] = {0}; //control inputs that will be the final product of the bangbang optimizer 

struct BangDim sat_angle = {0};
// struct BangDim sat_corr;
struct BangDim sign_corr=
{
    1,
    1,    
};
struct controllerstatestruct controllerstate={
    false, //apply_compensation boolean2
    false, // in_transition boolean
    0.2, // compensation time (add to 2nd section of prediction) 
    0,   //delta_v (add to initial condition of second section) should normally be negative but can be different because of delay in velocity estimation
    0     //delta_y (add to second section of prediction)
};
struct anaConstant constant;
struct anaConstant constant_sat_accel;
struct anaConstant constant_sat_brake; //braking part of prediction
struct anaConstant constant_sec; //for logging 

struct estimators *compensation_estimator;


float meas_angle[2];
// struct BangDim pos_error;
float satangle;
float secangle;
float signcorrsat;
float signcorrsec;
float pos_error[2];
float sat_corr[2];
float T;
float T_sat;
float T_sec; // for debugging purposes
// int dim;
float Cd = 0.56;
float ang0 ;
float ang1 ;
float angc ;
float angc_old ;
float E_pos= 1e9; 

float ys;
float vs;

int satdim=1; //default satdim is forward. 
int satdim_prev=10; 
float t_s; 
float t_target;


float v_velframe[2];
float mass=0.42;
float y_target;
float t_s=1e9;

float t_0_trans; //start time of transition
float v_0_trans; //velocity at start of transition.
float y_0_trans; //position at start of transition ;
float satang_0_trans;


float t_1_trans; //end time of transition
float v_1_trans; //velocity at end of transition.
float y_1_trans; //position at end of transition ;
float satang_1_trans;

float t_target_old;


void optimizeBangBang(float pos_error_vel_x, float pos_error_vel_y, float v_desired){
    pos_error[0]=pos_error_vel_x;
    pos_error[1]=pos_error_vel_y;

    v_velframe[0]=dr_state.vx*cosf(psi_command)+dr_state.vy*sinf(psi_command); 
    v_velframe[1]=-dr_state.vx*sinf(psi_command)+dr_state.vy*cosf(psi_command);

    meas_angle[0]=dr_state.theta;
    meas_angle[1]=dr_state.phi;
    // Determine which dimension will be saturated 
    
    float error_thresh = 1e-3; 
    if(pos_error[0]<0){
        sign_corr.x=-1; // sign_corr is used in the prediction part to correct for flying forward or backwards in Xb and Yb
    }
    else
    {
        sign_corr.x=1;
    }
    if(pos_error[1]<0){
        sign_corr.y=-1;
    }
    else
    {
        sign_corr.y=1;
    }
    sat_corr[0]=sat_angle.x;
    sat_corr[1]=sat_angle.y;
    
    if(!controllerstate.in_transition|| fabs(pos_error[0]-pos_error[1])>0.5){   //don't update satdim when in transition or when too close to the waypoint or when the error components are too close together
        if(fabs(pos_error[0])>=fabs(pos_error[1])){                                              // to avoid twitching.
            satdim=0;        
        }
        else{
            satdim=1;        
        }
    }
    #ifdef FORCESATDIM
        satdim=FORCESATDIM;
    #endif

    if(satdim==0){
        satangle=sat_corr[0];
        secangle=sat_corr[1];
        signcorrsat=sign_corr.x;
        signcorrsec=sign_corr.y;
    }
    else{
        satangle=sat_corr[1];
        secangle=sat_corr[0];
        signcorrsat=sign_corr.y;
        signcorrsec=sign_corr.x;
    }

    if(satdim_prev!=satdim){
        brake=false;
        
    }
    satdim_prev = satdim;
   

    type=SATURATION;
    dim=satdim;
    if(!brake){
        float t0 = 0; 
        float t1 = fabs(pos_error[satdim])+3;
        t_s = (t0+t1)/2.0;
        float t_s_old = 1e2;
        
        while(fabs(t_s-t_s_old)>2*dtt){
            t_s_old=t_s; 
            E_pos=get_E_pos(v_desired, satangle*signcorrsat);
            // printf("satdim: %i, E_pos: %f, t_s: %f , t0: %f, t1: %f\n",dim,E_pos*signcorrsat,t_s,t0,t1);
            if(E_pos*signcorrsat>0){
                t1=t_s;
            }
            else{
                t0=t_s;
            }
            t_s=(t0+t1)/2.0;            
        }
        bang_ctrl[dim]=satangle*signcorrsat;
    }
    
    //if braking:
    else{

        if(controllerstate.in_transition){ //when in transition desired angle is max brake angle
            bang_ctrl[dim]=-satangle*signcorrsat;
            t_target=t_target-dtt; // update t_target for second dimension (no prediction should be done during transition since v_d cannot be reached for the intermediate angles in transition)
        }
        else{   // find optimal braking angle to reach the target at the desired velocity
            ang0=-satangle;
            ang1=satangle;
            angc=(ang0+ang1)/2.0;
            angc_old = 1e9;

            while(fabs(angc-angc_old)>(0.1*d2r)){
                angc_old=angc;                
                E_pos = get_E_pos(v_desired,angc);

                if(E_pos>0){
                    ang1=angc;
                }
                else{
                    ang0=angc;
                }
                
               angc=(ang0+ang1)/2.;              
            }
            
            bang_ctrl[dim]=angc;
        }
       
    }
    //optimize second dimension
    type = SECOND;
    dim = 1-satdim; //switch dimension identifier from saturation to second 
    
    ang0 = -secangle;
    ang1 = secangle;
    angc = (ang0+ang1)/2.0;
    angc_old = 1e9;
    E_pos = 1e9;

    while(fabs(angc-angc_old)>0.1*d2r){
        angc_old=angc;
        E_pos=get_E_pos(v_desired,angc); //v_desired doesn't do anything here. The point is to reach the target at t_target.
        if(E_pos>0){
            ang1=angc;
        }
        else
        {
            ang0=angc;
        }
        angc=(ang0+ang1)/2;
    }
    bang_ctrl[dim]=angc;
    

    // Check if we need to brake    
    if(!brake){
        if(t_s<0.15 && t_s<t_target){
            brake=true;
            if(!controllerstate.in_transition){
               
                t_0_trans=get_sys_time_float(); // starttime of transition
                v_0_trans=v_velframe[satdim];
                y_0_trans=pos_error[satdim];
                satang_0_trans=meas_angle[satdim];
                controllerstate.in_transition=true;
                
            }
        }
        else if(controllerstate.in_transition){            
            controllerstate.in_transition=false; // make sure that in_transition is always false when not braking (in_transition we're basically flying blind because the controller forces to brake at maximum angle regardless of the state)
            
        }
    }

    // go out of transition when the measured angle is close to the commanded brake angle (timer is added as temporary measure to deal with the delay of bang_ctrl being updated to brake angle command)
    if(controllerstate.in_transition && fabs((bang_ctrl[satdim]-meas_angle[satdim])/bang_ctrl[satdim])<0.15 && ((get_sys_time_float()-t_0_trans)>3*dtt)){// ((get_sys_time_float()-t_0_trans)>controllerstate.delta_t)){ // transition ends when it has been active for more than delta_t 
        controllerstate.in_transition=false;                                    //  ^ will break down if bang_ctrl[satdim] is zero (not likely in practice);

        //measure transition values 
        satang_1_trans=meas_angle[satdim];
        y_1_trans=pos_error[satdim];
        v_1_trans=v_velframe[satdim];
        t_1_trans=get_sys_time_float();                               
        
        #ifdef LOG // log transition measurements
        if(t_1_trans-t_0_trans>dtt){
            comp_log_t=fopen(filename5,"a");                                                                            // note that y_0 and y_1 are actually the position errors to the wp which is why delta_y = y_0-y_1 instead of y_1-y_0. y0>y1
            fprintf(comp_log_t,"%f, %d, %f, %f, %f, %f, %f, %f\n",get_sys_time_float(),satdim,v_0_trans,satang_0_trans,satang_1_trans,t_1_trans-t_0_trans,y_0_trans-y_1_trans,v_1_trans-v_0_trans);
            fclose(comp_log_t);
        }
        #endif
    }   





    #ifdef LOG
    fprintf(bang_bang_t,"%f, %i, %i, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d, %f, %f, %f, %f, %f, %f\n",get_sys_time_float(), satdim, brake, t_s, t_target, 
    pos_error_vel_x, pos_error_vel_y, dr_state.x, dr_state.y, v_velframe[0], v_velframe[1],
    constant_sat_accel.c1,constant_sat_accel.c2, constant_sat_brake.c1, constant_sat_brake.c2, constant_sec.c1,
     constant_sec.c2, T_sat, T_sec,controllerstate.apply_compensation,controllerstate.in_transition,
     controllerstate.delta_t,controllerstate.delta_y,controllerstate.delta_v,ys,vs,delta_angle_in);
  
    #endif
};


float get_E_pos(float Vd, float angle){
   
    y_target = predict_path_analytical(angle,Vd); //y_target is the position at which the desired speed is reached. Ideally we want this value at pos_error (distance to wp)
    return y_target-pos_error[dim]; //TODO it is more intuitive to have this the other way around -> check signs in bisection parts
}
float predict_path_analytical(float angle,float Vd){
   
    #ifdef USETHRUSTALTCTRL //use the thrust as commanded by the altitude controller 
        T= (thrust_cmd/HOVERTHRUST)*mass*g*cosf(dr_state.phi)*cosf(dr_state.theta); // - earth z 
    #else
        T=mass*g;            // - earth z
    #endif
    if(dim==0){             
        
        T = -T*tanf(angle);//Thrust component forward, velframe x
    }
    else{
        T = T*tanf(angle)/cosf(bang_ctrl[0]);//Thrust component forward, velframe y  // TODO: maybe should use dr_state.theta instead of bang_ctrl[0]?
    }

    
    if(type==0){ // if saturation
        T_sat = T; 
        if(!brake){
            find_constants(0.0,v_velframe[dim]); // find constant for accelerating part; 
            constant_sat_accel = constant;
            ys = get_position_analytical(t_s);
            vs = get_velocity_analytical(t_s);

            //predict braking part:
            T=-1*T; 
            if(controllerstate.apply_compensation){
                
                if(dim==0){
                    if(angle<0){
                        compensation_estimator= &estimator_pitch_fwd;
                    }
                    else{
                        compensation_estimator=&estimator_pitch_bckwd;
                    } 
                }
                else{
                    compensation_estimator=&estimator_roll;
                }

                find_losses(vs,-2*angle,compensation_estimator); // -2 because delta_angle in estimator is final angle - initial angle  
                find_constants(ys+controllerstate.delta_y,vs+controllerstate.delta_v); // find constants for assumed conditions after transition
                }
            else{
                find_constants(ys,vs); //update constants for second part;
                }
            constant_sat_brake = constant;
            t_target = get_time_analytical(Vd);
            y_target = get_position_analytical(t_target);
            
            if(controllerstate.apply_compensation){
                t_target=t_target+t_s+controllerstate.delta_t; // t_target for 2nd section is relative to t_s. So add t_s and compensation to get absolute ETA
            }
            else{
                t_target=t_target+t_s;
            }
           
        }
        else{ //if already braking, predict only the braking part
            find_constants(0.0,v_velframe[dim]);
            constant_sat_brake = constant;
            t_target=get_time_analytical(Vd);
            y_target = get_position_analytical(t_target);            
        }
    }
    else{ //if second dimension
        T_sec = T;
        find_constants(0.0,v_velframe[dim]);
        constant_sec = constant; 
        y_target=get_position_analytical(t_target);//find position in lateral direction using the predicted eta of the saturation dimension;   
    }
    return y_target;
}

 void find_constants(float yi,float vi){
    constant.c1 = (vi-(T/Cd))*(-mass/Cd);
    constant.c2 = yi-constant.c1;
}
float get_position_analytical(float t){
    return constant.c1*expf(-(Cd/mass)*t)+constant.c2+(T/Cd)*t;
}
float get_velocity_analytical(float t){
    return constant.c1*(-Cd/mass)*expf(-(Cd/mass)*t)+(T/Cd);
}
float get_time_analytical(float V){
    float t_t = (-mass/Cd)*logf((V-(T/Cd))/(constant.c1*(-Cd/mass)));
    if(isnan(t_t)){
        // t_t = (-mass/Cd)*logf((V-(T/Cd))/(-1*constant.c1*(-Cd/mass))); //TODO: temporary fix that seems to work well in practice. Need to look into this more thoroughly
        t_t = 0; 
    }
    return t_t;
}
