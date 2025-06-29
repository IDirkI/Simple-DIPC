#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

typedef struct Motor {
    /* Pin Info*/
    int pinIN1; // IN1 pin #
    int pinIN2; // IN2 pin #
    int pinPWM; // PWM pin #

    /* Motor Constants */
    float J;       // Moment of inertia of shaft
    float R;       // Input resistance of motor
    float L;       // Input inductance of motor
    float Kt;      // Torque constant  of motor
    float tau;     // Rise time of motor
    float maxV;    // Max supply voltage of motor

    /* ############################################# */

    /* Operation Settings */
    bool    enabled;    // Specify if motor is enabled
    int     direction;  // Specify motor's direction (Forward: 1, Reversed: -1)
    float   pwm;        // Specify motor's PWM duty cycle [-1, 1]
} Motor;

// #### Actions ####
/* Initialize the motor pins and initially disable motor*/
void motorInit(Motor *motor) {
    pinMode(motor->pinIN1, OUTPUT);
    pinMode(motor->pinIN2, OUTPUT);
    pinMode(motor->pinPWM, OUTPUT);

    digitalWrite(motor->pinIN1, LOW);
    digitalWrite(motor->pinIN2, LOW);

    // Optional Values
    if(motor->direction == 0) motor->direction = 1;

    if(motor->J == 0) motor->J = 0.0f;
    if(motor->R == 0) motor->R = 0.0f;
    if(motor->L == 0) motor->L = 0.0f;
    if(motor->Kt == 0) motor->Kt = 0.0f;
    if(motor->tau == 0) motor->tau = 0.0f;
    if(motor->maxV == 0) motor->maxV = 0.0f;


    // Default Values
    motor->enabled = false;
    motor->pwm = 0.0f;
    

}

/* Run the motor at a given PWM duty cycle*/
void motorRun(Motor *motor, float pwm) {
    motor->pwm = pwm;
    if(pwm != 0.0f && !(motor->enabled)) motor->enabled = true;

    int input = 255*pwm*(motor->direction);    // Scale out pwm value from [-1, 1] to [-255, 255]

    if(input >= 0) {
        digitalWrite(motor->pinIN1, HIGH);
        digitalWrite(motor->pinIN2, LOW);
    }
    else {
        digitalWrite(motor->pinIN1, LOW);
        digitalWrite(motor->pinIN2, HIGH);
        input *= -1;
    }
    analogWrite(motor->pinPWM, input);  // Set motor speed
}

/* Disable motor & stop movemnt*/
void motorStop(Motor *motor) {
    motor->enabled = false;

    digitalWrite(motor->pinIN1, HIGH);
    digitalWrite(motor->pinIN2, HIGH);
}

/* Reverse the direction of a motor */
void motorReverse(Motor *motor) {
    motor->direction *= -1;
}

#endif