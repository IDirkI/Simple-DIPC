#ifndef PID_H
#define PID_H

typedef struct PID {
    /* Controller Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Controller Parameters*/
    float Ts;       // Sampling time (in sec)

    float tau;      // LPF time constant

    float outMin;   // Min output cap
    float outMax;   // Max output cap

    /* ############################################# */

    /* Controller memory */
    float proportional;     // $$p[n]$$

    float integrator;       // $$i[n]$$
    float prevErr;          // $$e[n-1]$$ | for integrator

    float differentiator;   // $$d[n]$$
    float prevMeasurement;  // $$y[n-1]$$ | for differentiator

    /* Input */ 
    float refPoint;         // $$r[n]$$     | Reference point to stablize around
    float currMeasurement;  // $$y[n]$$     | Measured sensor data

    /* Output */
    float output;      
} PID;

void pidInit(PID *pid);
void pidReset(PID *pid);
float pidUpdate(PID *pid, float measurement);

void pidTune(PID *pid, float P, float I, float D);
void pidSetReference(PID *pid, float ref);
void pidSetClamp(PID *pid, float min, float max);

// #### Actions ####
/* Initialize the PID controller */
void pidInit(PID *pid) {
    // Default Values if left unetered
    if(pid->Kp == 0.0f) pid->Kp = 1.0f;
    if(pid->Ki == 0.0f) pid->Ki = 0.0f;
    if(pid->Kd == 0.0f) pid->Kd = 0.0f;

    if(pid->Ts == 0.0f) pid->Ts = 1.0f;
    if(pid->tau == 0.0f) pid->tau = 100.0f;
    if(pid->outMax == 0.0f) pid->outMax = 1.0f;
    if(pid->outMin == 0.0f) pid->outMin = -1.0f;
    
    pidReset(pid);
}   

/* Reset the PID controller & its memory */
void pidReset(PID *pid) {
    pid->integrator = 0.0f;
    pid->prevErr = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->output = 0.0f;
}   

/* Update the PID state and output controller result */
float pidUpdate(PID *pid, float measurement) {
    pid->currMeasurement = measurement; // Update current measurement

    float err = pid->refPoint - measurement;    // Error signal

   /*
    * P - Signal
    *
    * $$ p[n] = K_P \cdot e[n] $$
    */
    pid->proportional = (pid->Kp) * err;       // Proportional signal part

   /*
    * I - Signal
    *
    * $$ i[n] = i[n-1] + \frac{K_IT_s}{2}\left(e[n] + e[n-1]\right) $$
    */
    pid->integrator += (0.5f*(pid->Ts)*(pid->Ki))*(err + pid->prevErr); // Integrator signal part
        // Anti wind-up clamper
        float integratorMax = (pid->outMax > pid->proportional) ?(pid->outMax - pid->proportional) :0.0f;
        float integratorMin = (pid->outMin < pid->proportional) ?(pid->outMin - pid->proportional) :0.0f;
        
        // Clamp integrator signal between [integratorMin, integratorMax]
        pid->integrator = (pid->integrator > integratorMax) ?integratorMax :((pid->integrator < integratorMin) ?integratorMin :(pid->integrator));

    /*
    * D - Signal (band-limited diff.)
    *
    * $$ d[n] = \frac{2\tau - T_s}{2\tau + T_s}d[n-1] + \frac{2K_D}{2\tau + T_s}\left(e[n] - e[n-1]\right) $$
    */
    pid->differentiator = ((2*pid->tau - pid->Ts)/(2*pid->tau + pid->Ts))*pid->differentiator + ((2*pid->Kd)/(2*pid->tau + pid->Ts))*(measurement - pid->prevMeasurement);

    /*
    * Output Signal
    *
    * $$u[n] = p[n] + i[n] + d[n]$$
    */
    pid->output = pid->proportional + pid->integrator + pid->differentiator;
        // Clamp the PID output
         pid->output = (pid->output > pid->outMax) ?(pid->outMax) :((pid->output < pid->outMin) ?(pid->outMin) :(pid->output));

    // Store previous error ($$e[n]$$) and measurement ($$y[n]$$) signals
    pid->prevErr = err;
    pid->prevMeasurement = measurement;

    // Return output
    return pid->output;
}

// #### Setters ####
/* Sets the Kp, Ki & Kd gains to the given floats*/
void pidTune(PID *pid, float P, float I, float D) {
    pid->Kp = P;
    pid->Ki = I;
    pid->Kd = D;
}

/* Sets the reference point of the controller to the given float value*/
void pidSetReference(PID *pid, float ref) {
    pid->refPoint = ref;
}

/* Sets the min & max output clamp values to the given two floats*/
void pidSetClamp(PID *pid, float min, float max) {
    pid->outMin = min;
    pid->outMax = max;
}

#endif