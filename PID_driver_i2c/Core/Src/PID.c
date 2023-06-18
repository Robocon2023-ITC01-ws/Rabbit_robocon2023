 #include "PID.h"

void PID_Init(PIDController *pid, int N_input) {//nMotor is the number of motor to control

	/* Clear controller variables */
	for(int i = 0;i < N_input; i++){
		pid->integrator[i] = 0.0f;
		pid->prevError[i]  = 0.0f;

		pid->differentiator[i]  = 0.0f;
		pid->prevMeasurement[i] = 0.0f;

		pid->out[i] = 0.0f;
	}


}

float PID(PIDController *pid, float setpoint, float measurement,float Kp,float Ki,float Kd, int i){// "i" input identity
	/*
	* -------------------------------------Error signal------------------------------------------
	*/
	float error = setpoint - measurement;;
	/*
	* Proportional
	*/
    float proportional = Kp * error;
	/*
	* ----------------------------------------Integral-----------------------------------------------
	*/
    pid->integrator[i]	= pid->integrator[i] + 0.5f * Ki * pid->T
    					* (error + pid->prevError[i]);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator[i] > pid->limMaxInt) {
        pid->integrator[i] = pid->limMaxInt;
    }
    else if (pid->integrator[i] < pid->limMinInt) {
        pid->integrator[i] = pid->limMinInt;
    }
    else{
    	pid->integrator[i] = pid->integrator[i];
    }

	/*
	* Derivative (band-limited differentator)
	*/

    pid->differentiator[i] = -(2.0f * Kd * (measurement - pid->prevMeasurement[i])	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        	 + (2.0f * pid->tau - pid->T) * pid->differentiator[i])
							 / (2.0f * pid->tau + pid->T);
	/*
	* Compute output and apply limits
	*/
    pid->out[i] = proportional + pid->integrator[i] + pid->differentiator[i];
    //sat[i] = pid->out[i];
    if (pid->out[i] > pid->limMax) {
        pid->out[i] = pid->limMax;
    }
    else if (pid->out[i] < pid->limMin) {
        pid->out[i] = pid->limMin;
    }
    else{
    	pid->out[i] = pid->out[i];
    }

    /* Store error and measurement for later use */
    pid->prevError[i]       = error;
    pid->prevMeasurement[i] = measurement;

	/* Return controller output */
    return pid->out[i];

}
