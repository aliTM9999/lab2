/*
 * kalman_c.h
 *
 *  Created on: Sep 17, 2024
 *      Author: atherr14
 */

#ifndef INC_KALMAN_C_H_
#define INC_KALMAN_C_H_

typedef struct kalman_state{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float p; //estimation error covariance
	float x; //estimated value
	float k; // adaptive Kalman filter gain


}kalman_state;

int Kalmanfilter(float* InputArray, float* OutputArray, kalman_state* kstate, int Length);


#endif /* INC_KALMAN_C_H_ */
