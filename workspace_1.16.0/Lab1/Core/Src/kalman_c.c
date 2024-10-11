#include <math.h>
#include "kalman_c.h"

int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
	for(int i =0; i <Length; i++){
		if (isnan(kstate->x) || isnan(kstate->p) || isinf(kstate->x) || isinf(kstate->p) || (kstate->p + kstate->r==0)){
			return -1;
		}
		kstate->p = kstate->p + kstate->q;
		kstate->k = kstate->p/(kstate->p + kstate->r);
		kstate->x = kstate->x + kstate->k * (InputArray[i] - kstate->x);
		kstate->p = (1 - kstate->k) * kstate->p;
		OutputArray[i]=kstate->x;
	}

	return 0;
}

