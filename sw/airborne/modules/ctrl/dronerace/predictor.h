

#define PI 3.14159265359
#define d2r  PI/180.0 
#define r2d 1.0/d2r
#define GR (1.0 + sqrtf(5))/2.0

#define MAX_ROLL_RATE 50.0*d2r


FILE *prediction_logger_t= NULL;
int optcounter= 0;
float predict(float roll_cmd, float xi, float yi,float v, float tx,float ty, float phi, float psi){
    
    
    const float prop_dt = 0.1;
    float x = xi; 
    float y = yi; 
    // saveguard no divide by zero
    if(v < 1){
        v = 1; 
    }
 

    float apo2 = 1e20; 
    
    for(int i =0; i <15 ; i=i+1){
      
        
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
        if(psi>2*PI){
            psi=psi-2*PI;
        }
        else if (psi<-2*PI)
        {
            psi=psi+2*PI;
        }
        

        x = x + cosf(delayed_psi) * v * prop_dt;
        y = y + sinf(delayed_psi) * v * prop_dt;

        float d2 = (tx-x)*(tx-x)+(ty-y)*(ty-y);
        // fprintf(prediction_logger_t,"%f, %f, %f, %f, %f, %f, %f\n",x,y,xi,yi,v,phi,roll_cmd,psi,d2);
        if(d2<apo2){
            apo2= d2;
            
        }
        

    }
    
    return apo2;
}


float find_roll(float v, float phi, float psi, float xi, float yi, float tx ,float ty) { 
    // printf("test4");
    float best = 0; 
    float bestcost = 1e30; 
    float a = -45*d2r;
    float b = 45*d2r; 
    // printf("xi: %f, yi: %f\n",xi, yi);
   
    
    for(int j=1; j<10; j=j+1){
        float c = b - (b-a)/GR;
        float d = a + (b-a)/GR;

        float cost_c = predict(c,xi,yi, v, tx, ty, phi, psi);
        
        float cost_d = predict(d, xi, yi, v,tx, ty,phi,psi);

        if(cost_c<cost_d){
            a = d; 
            best = c;
            bestcost=cost_c;
        }
        else{
            b = c; 
            best = d; 
            bestcost=cost_d;
        }
        // fprintf(prediction_logger_t,"%d, %f, %f\n",optcounter,best,bestcost);
        if((c-d)<0.1*d2r){
            break;
        }
        
       
    }
    optcounter=optcounter+1;
    /*
    for(float roll = -45*d2r; roll<45*d2r; roll= roll + 1*d2r){
        float cost = predict(roll);
        if(cost < bestcost){
            best = roll;
            bestcost = cost; 
        }
    }
    */


    
    return best;
}

float find_yaw(float psi,float phi,float v,float drone_dt){
    float dpsidt = (9.81/v)*tanf(phi);
    psi=psi+(dpsidt*drone_dt);
    
    if(psi>(2*PI)){
        psi=psi-(2*PI);
    }
    if(psi<(-2*PI)){
        psi=psi+(2*PI);
    }
    printf("psi: %f\n",psi*r2d);

    return psi;
}