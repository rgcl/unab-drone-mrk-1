// Copyright (c) 2023 Rodrigo González,
// basado en ArduProject
// Copyright (c) 2022 ArduProject

#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>
#include <Wire.h>

struct MPU6050_Angles {
  float pitch;
  float roll;
  float yaw;
  float gyro_X;
  float gyro_Y;
  float gyro_Z;
};

class MPU6050 {
private:
  float angulo_pitch, angulo_roll, angulo_yaw, angulo_pitch_acc, angulo_roll_acc, temperature;
  float angulo_pitch_ant, angulo_roll_ant, angulo_yaw_ant;
  int gx, gy, gz, gyro_Z, gyro_X, gyro_Y, gyro_X_ant, gyro_Y_ant, gyro_Z_ant;
  float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
  float ax, ay, az, acc_X_cal, acc_Y_cal, acc_Z_cal, acc_total_vector;
  bool set_gyro_angles, accCalibOK = false;
  float tiempo_ejecucion_MPU6050, tiempo_MPU6050_1;

public:
  static const int ERROR_INIT = 1;
  static const int adress = 0x68;

  MPU6050();
  int init();
  void calibrate();
  void read();
  void process();
  void readAngles(MPU6050_Angles& angles);
};

#endif
