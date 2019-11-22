

#define PI 3.14159265359
#define d2r  PI/180.0 
#define r2d 1.0/d2r

#define MAX_ROLL_RATE 50.0*d2r
float v;
float phi;
float psi;
float x;
float y;
float tx;
float ty;
float predict(float roll_cmd){
 
    const float prop_dt = 0.1;

    // saveguard no divide by zero
    if(v < 1){
        v = 1; 
    }
 

    float apo2 = 1e20; 

    for(int i =0; i <35; i=i=1){
        float delta_phi = roll_cmd - phi;
    if(delta_phi>MAX_ROLL_RATE){
        delta_phi = MAX_ROLL_RATE;
    }
    if(delta_phi<-MAX_ROLL_RATE){
        delta_phi=-MAX_ROLL_RATE;
    }

        phi = phi + delta_phi * prop_dt;

        float delayed_psi = psi;

        float dpsidt  = tanf(phi) * 9.81 / v;
        psi = psi + dpsidt*prop_dt; 

        x = x + cosf(delayed_psi) * v * prop_dt;
        y = y + sinf(delayed_psi) * v * prop_dt;

        float d2 = (tx-x)*(tx-x)+(ty-y)*(ty-y);

        if(d2<apo2){
            apo2= d2;
        }
    }
    return apo2;
}


float find_roll(v, phi, psi, x, y, tx ,ty) { 
    float best = 0; 
    float  bestcost = 1e30; 
    for(float roll = -45*d2r; roll<45*d2r; roll= roll + 1*d2r){
        float cost = predict(roll);
        if(cost < bestcost){
            best = roll;
            bestcost = cost; 
        }
    }
    return best;
}

float find_yaw(float psi,float phi,float v,float drone_dt){
    float dpsidt = (9.81/v)*tanf(phi);
    psi=psi+(dpsidt*drone_dt);
    
    while(psi>2*PI){
        psi=psi-2*PI;
    }
    while(psi<-2*PI){
        psi=psi+2*PI;
    }

    return psi;
}