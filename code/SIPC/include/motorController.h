#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor.h"

typedef struct MotorController {
    Motor motor1;
    Motor motor2;

    /* Pin Info*/
    int m1PinIN1;   // IN1 pin of motor1
    int m1PinIN2;   // IN2 pin of motor1
    int m1PinPWM;   // PWM pin of motor1

    int m2PinIN1;   // IN1 pin of motor1
    int m2PinIN2;   // IN2 pin of motor1
    int m2PinPWM;   // PWM pin of motor1

    /* Motor Constants */
    float J1;      // Moment of inertia of the shaftof motor1
    float R1;      // Input resistance of motor1
    float L1;      // Input inductance of motor1
    float Kt1;     // Torque constant  of motor1
    float tau1;    // Rise time of motor1
    float maxV1;   // Max supply voltage of motor1

    float J2;      // Moment of inertia of the shaftof motor2
    float R2;      // Input resistance of motor2
    float L2;      // Input inductance of motor2
    float Kt2;     // Torque constant  of motor2
    float tau2;    // Rise time of motor2
    float maxV2;   // Max supply voltage of motor2

    /* ############################################# */

    /* Operation Settings */
    bool    m1Enabled;      // Specify if motor1 is enabled
    int     m1Direction;    // Specify motor1's direction (Forward: 1, Reversed: -1)
    float   m1PWM;          // Specify motor1's PWM duty cycle [-1, 1]

    bool    m2Enabled;
    int     m2Direction;
    float   m2PWM;

} MotorController;

// #### Actions ####
/* Setup the motor controller pins and initially disable motors*/
void mcInit(MotorController *mControl) {
    mControl->motor1 = {
        .pinIN1 = mControl->m1PinIN1,
        .pinIN2 = mControl->m1PinIN2,
        .pinPWM = mControl->m1PinPWM
    };
    mControl->motor2 = {
        .pinIN1 = mControl->m2PinIN1,
        .pinIN2 = mControl->m2PinIN2,
        .pinPWM = mControl->m2PinPWM
    };

    motorInit(&(mControl->motor1));
    motorInit(&(mControl->motor1));

    mControl->m1Direction = mControl->motor1.direction;
    mControl->m2Direction = mControl->motor2.direction;

    mControl->m1Enabled = false;
    mControl->m2Enabled = false;

    mControl->J1 = 0.0f;
    mControl->R1 = 0.0f;
    mControl->L1 = 0.0f;
    mControl->Kt1 = 0.0f;
    mControl->tau1 = 0.0f;
    mControl->maxV1 = 0.0f;

    mControl->J2 = 0.0f;
    mControl->R2 = 0.0f;
    mControl->L2 = 0.0f;
    mControl->Kt2 = 0.0f;
    mControl->tau2 = 0.0f;
    mControl->maxV2 = 0.0f;
}

/* Run the motors at a given PWM duty cycle*/
void mcRun(MotorController *mControl, float pwm1, float pwm2) {
    mControl->m1PWM = pwm1;
    mControl->m2PWM = pwm2;

    motorRun(&(mControl->motor1), pwm1);
    motorRun(&(mControl->motor2), pwm2);

    mControl->m1Enabled = mControl->motor1.enabled;
    mControl->m2Enabled = mControl->motor1.enabled;
}

/* Disable motor & stop movemnt*/
void mcStop(MotorController *mControl) {
    mControl->m1Enabled = false;
    mControl->m2Enabled = false;

    motorStop(&(mControl->motor1));
    motorStop(&(mControl->motor2));
}

/* Reverse the direction of a motors */
void mcReverse(MotorController *mControl, bool reverse1, bool reverse2) {
    if(reverse1) motorReverse(&(mControl->motor1));
    if(reverse2) motorReverse(&(mControl->motor2));
}

// #### Setters ####
/* Set moments of inertia [ J₁, J₂ ] */
void mcSetJ(MotorController *mControl, float J1, float J2) {
    mControl->motor1.J = J1;
    mControl->motor2.J = J2;

    mControl->J1 = J1;
    mControl->J2 = J2;
}

/* Set input resistances [ R₁, R₂ ] */
void mcSetR(MotorController *mControl, float R1, float R2) {
    mControl->motor1.R = R1;
    mControl->motor2.R = R2;

    mControl->R1 = R1;
    mControl->R2 = R2;
}

/* Set input inductances [ L₁, L₂ ] */
void mcSetL(MotorController *mControl, float L1, float L2) {
    mControl->motor1.L = L1;
    mControl->motor2.L = L2;

    mControl->L1 = L1;
    mControl->L2 = L2;
}

/* Set torque constants [ Kₜ₁, Kₜ₂ ] */
void mcSetKt(MotorController *mControl, float Kt1, float Kt2) {
    mControl->motor1.Kt = Kt1;
    mControl->motor2.Kt = Kt2;

    mControl->Kt1 = Kt1;
    mControl->Kt2 = Kt2;
}

/* Set rise times [ 𝜏₁, 𝜏₂ ] */
void mcSetTau(MotorController *mControl, float tau1, float tau2) {
    mControl->motor1.tau = tau1;
    mControl->motor2.tau = tau2;

    mControl->tau1 = tau1;
    mControl->tau2 = tau2;
}

/* Set max supply voltgaes [ Vₘₐₓ₁, Vₘₐₓ₂ ] */
void mcSetVMax(MotorController *mControl, float maxV1, float maxV2) {
    mControl->motor1.maxV = maxV1;
    mControl->motor2.maxV = maxV2;

    mControl->maxV1 = maxV1;
    mControl->maxV2 = maxV2;
}

#endif