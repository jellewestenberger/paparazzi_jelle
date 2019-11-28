#ifndef PRED_THREAD_H_INCLUDED__
#define  PRED_THREAD_H_INCLUDED__

#include <stdbool.h>
#include <stdio.h>
extern volatile bool work ;
extern volatile bool hasresult;
extern volatile float optimized_roll; 
volatile struct predict_input{
    float v; 
    float xi;
    float yi; 
    float tx; 
    float ty; 
    float phi;
    float psi; 
};
extern volatile struct predict_input pred_inputs; 
#endif
