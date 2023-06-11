// Copyright (c) 2023 Rodrigo González,
// basado en ArduProject
// Copyright (c) 2022 ArduProject

#include "MPU6050.h"

MPU6050::MPU6050() {}

int MPU6050::init() {
  Wire.beginTransmission(adress);
  Wire.write(0x6B);  //Registro 6B hex)
  Wire.write(0x00);  //00000000 para activar giroscopio
  Wire.endTransmission();
  Wire.beginTransmission(adress);
  Wire.write(0x1B);  //Register 1B hex
  Wire.write(0x08);  //Girscopio a 500dps (full scale)
  Wire.endTransmission();
  Wire.beginTransmission(adress);
  Wire.write(0x1C);  //Register (1A hex)
  Wire.write(0x10);  //Acelerometro a  +/- 8g (full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(adress);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(adress, 1);
  while (Wire.available() < 1) {}

  // Si hay un error en el sensor MPU6050 avisamos y enclavamos el programa
  if (Wire.read() != 0x08) {
    return ERROR_INIT;
  }

  // Activar y configurar filtro pasa bajos LPF que incorpora el sensor
  Wire.beginTransmission(adress);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  /*
    Frecuencia de corte del filtro pasa bajos:
    256Hz(0ms):0x00
    188Hz(2ms):0x01
    98Hz(3ms):0x02
    42Hz(4.9ms):0x03
    20Hz(8.5ms):0x04
    10Hz(13.8ms):0x05
    5Hz(19ms):0x06
  */
  return 0;
}

void MPU6050::calibrate() {

  // Calibrar giroscopio tomando 3000 muestras
  for (int cal_int = 0; cal_int < 3000; cal_int++) {
    read();
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal += ax;
    acc_Y_cal += ay;
    acc_Z_cal += az;
    delayMicroseconds(20);
    Serial.println(cal_int);
  }
  // Calcular el valor medio de las 3000 muestras
  gyro_X_cal = gyro_X_cal / 3000;
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
  acc_X_cal = acc_X_cal / 3000;
  acc_Y_cal = acc_Y_cal / 3000;
  acc_Z_cal = acc_Z_cal / 3000;
  accCalibOK = true;
  Serial.println("Ending calibration");
}

// Leer sensor MPU6050
void MPU6050::read() {
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(adress);  // Empezamos comunicación
  Wire.write(0x3B);                // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(adress, 14);      // Solicitar un total de 14 registros
  while (Wire.available() < 14) {};  // Esperamos hasta recibir los 14 bytes

  ax = Wire.read() << 8 | Wire.read();           // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();           // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();           // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();           // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();           // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();           // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // Restar valores de calibracion del acelerómetro
  if (accCalibOK == true) {
    ax -= acc_X_cal;
    ay -= acc_Y_cal;
    az -= acc_Z_cal;
    az = az + 4096;
  }
}

// Cálculo de velocidad angular (º/s) y ángulo (º)
void MPU6050::process() {
  // Restar valores de calibración del acelerómetro y calcular
  // velocidad angular en º/s. Leer 65.5 en raw equivale a 1º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calculamos exactamente cuánto tiempo ha pasado desde que se ha ejecutado el cálculo del ángulo.
  // Al tener señales PWM variables entre 1 y 2ms, este cálculo del ángulo no se ejecuta siempre
  // con un periodo constante.
  tiempo_ejecucion_MPU6050 = (micros() - tiempo_MPU6050_1) / 1000;

  // Calcular ángulo de inclinación con datos de giroscopio:
  // velocidad (º/s) * tiempo (s) = grados de inclinación (º)
  angulo_pitch += gyro_X * tiempo_ejecucion_MPU6050 / 1000;
  angulo_roll += gyro_Y * tiempo_ejecucion_MPU6050 / 1000;
  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  angulo_pitch += angulo_roll * sin((gz - gyro_Z_cal) * tiempo_ejecucion_MPU6050 * 0.000000266);
  angulo_roll -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion_MPU6050 * 0.000000266);
  tiempo_MPU6050_1 = micros();

  // Calcular vector de aceleración
  // 57.2958 = Conversion de radianes a grados 180/PI
  acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc = asin((float)ax / acc_total_vector) * -57.2958;

  if (set_gyro_angles) {
    // Filtro complementario
    angulo_pitch = angulo_pitch * 0.995 + angulo_pitch_acc * 0.005;  // Angulo Pitch de inclinacion
    angulo_roll = angulo_roll * 0.995 + angulo_roll_acc * 0.005;     // Angulo Roll de inclinacion
  } else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll = angulo_roll_acc;
    set_gyro_angles = true;
  }
}

void MPU6050::readAngles(MPU6050_Angles& angles) {
  read();
  process();
  angles.pitch = angulo_pitch;
  angles.roll = angulo_roll;
  angles.yaw = angulo_yaw;
  angles.gyro_X = gyro_X;
  angles.gyro_Y = gyro_Y;
  angles.gyro_Z = gyro_Z;
}
