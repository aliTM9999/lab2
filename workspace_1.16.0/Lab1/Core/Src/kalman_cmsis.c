#include <math.h>
#include "kalman_c.h"

int KalmanfilterCMSIS(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
	for(int i =0; i <Length; i++){
		if (isnan(kstate->x) || isnan(kstate->p) || isinf(kstate->x) || isinf(kstate->p) || (kstate->p + kstate->r==0)){
			return -1;
		}

		float p[]={kstate->p};
		float k[]={kstate->k};
		float q[]={kstate->q};
		float x[]={kstate->x};
		float r[]={kstate->r};
		float measurement[]={InputArray[i]};
		float temp[1];
		float oneArr[]={1.0};

		//p <- p + q
		arm_add_f32(p, q, p,1);

		//k <- p / (p+r)
		arm_add_f32(p, r, temp,1);
		kstate->k = p[0]/temp[0];
		k[0]=kstate->k;

		//x <- x + k*(measurement - x)
		arm_negate_f32(x, temp, 1);
		arm_add_f32(measurement, temp,temp,1);
		arm_mult_f32(k, temp, temp,1);
		arm_add_f32(x, temp,x,1);
		kstate->x=x[0];

		//p <- (1-k)*p
		arm_negate_f32(k,temp,1);
		arm_add_f32(oneArr,temp,temp,1);
		arm_mult_f32(p,temp,p,1);
		kstate->p = p[0];
		OutputArray[i]=kstate->x;
	}

	return 0;
}
