

#define PI 3.14159265359
#define d2r  PI/180.0 
#define r2d 1.0/d2r

#define MAX_ROLL_RATE 50.0*d2r
float t_v;
float t_phi;
float t_psi;
float t_x;
float t_y;
float t_tx;
float t_ty;

float predict(float t_roll_cmd){
    
    
    const float prop_dt = 0.1;

    // saveguard no divide by zero
    if(t_v < 1){
        t_v = 1; 
    }
 

    float apo2 = 1e20; 
    
    for(int i =0; i <15 ; i=i+1){
      
        
        float t_delta_phi = t_roll_cmd - t_phi;
    if(t_delta_phi>MAX_ROLL_RATE){
        t_delta_phi = MAX_ROLL_RATE;
    }
    if(t_delta_phi<-MAX_ROLL_RATE){
        t_delta_phi=-MAX_ROLL_RATE;
    }

        t_phi = t_phi + t_delta_phi * prop_dt;
      
        float delayed_psi = t_psi;

        float dpsidt  = tanf(t_phi) * 9.81 / t_v;
        t_psi = t_psi + dpsidt*prop_dt; 

        t_x = t_x + cosf(delayed_psi) * t_v * prop_dt;
        t_y = t_y + sinf(delayed_psi) * t_v * prop_dt;

        float d2 = (t_tx-t_x)*(t_tx-t_x)+(t_ty-t_y)*(t_ty-t_y);

        if(d2<apo2){
            apo2= d2;
        }
        

    }
    
    return apo2;
}


float find_roll(t_v, t_phi, t_psi, t_x, t_y, t_tx ,t_ty) { 
    // printf("test4");
    float best = 0; 
    float  bestcost = 1e30; 
    float min = -45*d2r;
    float max = 45*d2r; 
    float costmin = predict(min);
    float costmax = predict(max);

    for(int j=1; j<10; j=j+1){
        if(costmin<costmax){
            max = (max+min)/2.0;
            costmax = predict(max);
            // printf("it %d, mincost: %f\n",j,costmin);
        }
        else{
            min = (max+min)/2.0;
            costmin=predict(min);
            // printf("it %d, mincost: %f\n",j,costmax);
        }
    }
    best=(max+min)/2.0;
    /*
    for(float t_roll = -45*d2r; t_roll<45*d2r; t_roll= t_roll + 1*d2r){
        float cost = predict(t_roll);
        if(cost < bestcost){
            best = t_roll;
            bestcost = cost; 
        }
    }
    */


    
    return best;
}

float find_yaw(float t_psi,float t_phi,float t_v,float drone_dt){
    float dpsidt = (9.81/t_v)*tanf(t_phi);
    t_psi=t_psi+(dpsidt*drone_dt);
    
    if(t_psi>2*PI){
        t_psi=t_psi-2*PI;
    }
    if(t_psi<-2*PI){
        t_psi=t_psi+2*PI;
    }

    return t_psi;
}