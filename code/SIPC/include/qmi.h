#ifndef QMI_H
#define QMI_H

#include <Wire.h>

#include "SensorQMI8658.hpp"
#include "vector3.h"

#define G_CONST 9.80665


typedef struct QMI {
    SensorQMI8658 qmi;

    /* QMI Parameters*/
    const int SCL_PIN;  // CLK pin for the I2C communication
    const int SDA_PIN;  // DATA pin for the I2C communication

    uint16_t gK[3]; // Calibration gains
    Vector3 cV;     // Correction factor/vector required for the sensor to output [0, 0, -9.81] when at rest

    /* ############################################# */

    /* Outputs */
    IMUdata acc;
    IMUdata gyro;
    Vector3 angle;

    /* Constants */
    uint8_t chipID;     // ID no. of the chip
    Vector3 restAcc;    // [0.0, 0.0, -9.80665]
} QMI;

void qmiUpdateData(QMI *sensor);
void qmiGetAcc(QMI *sensor, Vector3 *vAcc);
uint8_t qmiGetChipID(QMI *sensor);

// #### Actions ####
/* Initialize the QMI sensor */
void qmiStart(QMI *sensor) {
    bool validate = sensor->qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, sensor->SDA_PIN, sensor->SCL_PIN);
    
    sensor->chipID = sensor->qmi.getChipID();

    if(!validate) {
        Serial.print("Failed to find QMI8658 ❌");
    }

    // Constants Setup
    sensor->cV = {0, 0, 0};
    sensor->restAcc = {0, 0, -G_CONST};
}

/* Calibration sequence and test for accurate data*/
void qmiCalibrate(QMI *sensor) {
    bool calibrated = false;
    sensor->gK[0] = 0;
    sensor->gK[1] = 0;
    sensor->gK[2] = 0;

    while(!calibrated){
        calibrated = sensor->qmi.calibration(&(sensor->gK[0]), &(sensor->gK[1]), &(sensor->gK[2]));

        if(calibrated) {
            Serial.print("Calibration complete ✅\n");
            break;
        }
        else {
            Serial.print("Calibration failed ❌\n");
            Serial.print("Retrying calibration...\n");
        }
    }

    calibrated = sensor->qmi.writeCalibration(sensor->gK[0], sensor->gK[1], sensor->gK[2]);

    if(calibrated) {
        Serial.print("Calibration gains written to memory ✅\n");
    }
    else {
        Serial.print("Calibration couldn't be saved to memory ❌\n");
    }
}   

/* Setup of the config of the sensor */
void qmiSetConfig(QMI *sensor) {
    sensor->qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0);

    sensor->qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3); 
}

/* Self-test script of the sensor to check for hardware faults*/
void qmiSelfTest(QMI *sensor) {
    if (sensor->qmi.selfTestAccel()) {
        Serial.print("Accelerometer self-test successful ✅\n");
    } else {
        Serial.print("Accelerometer self-test failed! ❌\n");
    }

    if (sensor->qmi.selfTestGyro()) {
        Serial.print("Gyroscope self-test successful ✅\n");
    } else {
        Serial.print("Gyroscope self-test failed! ❌\n");
    }
}

/* Enables QMI accelerometer and gyroscope */
void qmiEnable(QMI *sensor) {
    sensor->qmi.enableAccelerometer();
    sensor->qmi.enableGyroscope();
}

/* Disables QMI accelerometer and gyroscope */
void qmiDisable(QMI *sensor) {
    sensor->qmi.disableAccelerometer();
    sensor->qmi.disableGyroscope();
}

/* Calculates the correction vector to add to the measurement for consistency. Sensor must be flat on start up for it to work correctly */
void qmiSetCorrection(QMI *sensor) {
    qmiUpdateData(sensor);

    sensor->cV.x = sensor->restAcc.x - sensor->acc.x;
    sensor->cV.y = sensor->restAcc.y - sensor->acc.y;
    sensor->cV.z = sensor->restAcc.z - sensor->acc.z;
}

/* Wrapper start function for all QMI functionality */
void qmiInit(QMI *sensor) {
    /* Establish connection */
    qmiStart(sensor);

    /* Print Chip ID*/
    Serial.print("Device ID:");
    Serial.println(qmiGetChipID(sensor), HEX);

    /* Calibrate */
    qmiCalibrate(sensor);

    /* Self Test */
    qmiSelfTest(sensor);

    /* Configure */
    qmiSetConfig(sensor);

    /* Enable */
    qmiEnable(sensor);

    /* Set up correction term*/
    qmiSetCorrection(sensor);
}

// #### Data ####
/* Load up the current measuremnets to the local accelerometer, gyrosope and tilt vectors*/
void qmiUpdateData(QMI *sensor) {
    bool ready = false;
    while(!ready) {
        ready = sensor->qmi.getDataReady();
    }

    // Accelerometer + Gyro
    sensor->qmi.getAccelerometer(sensor->acc.x, sensor->acc.y, sensor->acc.z);
    sensor->qmi.getGyroscope(sensor->gyro.x, sensor->gyro.y, sensor->gyro.z);

    // Angle Calculation
    sensor->angle.x = atan((sensor->acc.x)/(sqrt(pow(sensor->acc.y, 2) + pow(sensor->acc.z, 2))));
    sensor->angle.y = atan((sensor->acc.y)/(sqrt(pow(sensor->acc.x, 2) + pow(sensor->acc.z, 2))));
    sensor->angle.z = atan((sqrt(pow(sensor->acc.x, 2) + pow(sensor->acc.y, 2)))/(sensor->acc.z));
}

/* Recieve a time stamp from the QMI */
uint32_t qmiTimeStamp(QMI *sensor) {
    return sensor->qmi.getTimestamp();
}

// #### Getters ####
/* Returns the ChipID */
uint8_t qmiGetChipID(QMI *sensor) {
    return sensor->chipID;
}

/* Return the last accelerometer data inside the inputed vector */
void qmiGetAcc(QMI *sensor, Vector3 *vAcc) {
    vAcc->x = (sensor->acc.x)*G_CONST + sensor->cV.x;
    vAcc->y = (sensor->acc.y)*G_CONST + sensor->cV.y;
    vAcc->z = -(sensor->acc.z)*G_CONST + sensor->cV.z;
}

/* Return the last gyroscope data inside the inputed vector */
void qmiGetGyro(QMI *sensor, Vector3 *vGyro) {
    vGyro->x = sensor->gyro.x;
    vGyro->y = sensor->gyro.y;
    vGyro->z = sensor->gyro.z;
}

/* Return the last tilt data inside the inputed vector */
void qmiGetAngles(QMI *sensor, Vector3 *angles) {
    angles->x = sensor->angle.x;
    angles->y = sensor->angle.y;
    angles->z = sensor->angle.z;
}

/* Return the last accelerometer, gyroscope and tilt data inside the inputed vectors */
void qmiGetFull(QMI *sensor, Vector3 *vAcc, Vector3 *vGyro, Vector3 *angles) {
    qmiUpdateData(sensor);
    qmiGetAcc(sensor, vAcc);
    qmiGetGyro(sensor, vGyro);
    qmiGetAngles(sensor, angles);
}

#endif