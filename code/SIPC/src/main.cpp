#include <Arduino.h>
#include <Wire.h>

#include "SensorQMI8658.hpp"

#include "motorController.h"
#include "qmi.h"
#include "pid.h"

/* QMI Sensor Pins*/
#define QMI_SCL 17
#define QMI_SDA 18

/* Motor Controller Pins*/
#define M1_IN1 25
#define M1_IN2 26
#define M1_PWM 27

/* QMI Sensor*/
QMI sensor = {
              .SCL_PIN = QMI_SCL,
              .SDA_PIN = QMI_SDA
            };

/* Motor Controller*/
MotorController mc = {
                    .m1PinIN1 = M1_IN1,
                    .m1PinIN2 = M1_IN2,
                    .m1PinPWM = M1_PWM
                  };

/* PID Controller */
PID pid = {
            .Kp = 1.0f,
            .Ki = 1.0f,
            .Kd = 1.0f,

            .Ts = 0.02f,
            .tau = 50.f,

            .outMin = -1.0f,
            .outMax = 1.0f
        };

Vector3 acc = {0, 0, 0};
Vector3 gyro = {0, 0, 0};
Vector3 angle = {0, 0, 0};

void setup() {
  Serial.begin(115200);

  qmiInit(&sensor);
  mcInit(&mc);

  pidInit(&pid);
}

void loop() {
    delay(20);
    qmiGetFull(&sensor, &acc, &gyro, &angle);

    float output = (angle.y - (-M_PI_2))*2 / (M_PI) - 1;

    mcRun(&mc, output, 0.0);
}
