#include <Wire.h>
#include <Servo.h>

#include "MPU6050.h"

// our direction as a slave drone.
#define DRONE_I2C_ID 8

#define THROTTLE_UP "throttleUp"
#define THROTTLE_DOWN "throttleDown"
#define YAW_LEFT "yawLeft"
#define YAW_RIGHT "yawRight"
#define PITCH_UP "pitchUp"
#define ROLL_LEFT "rollLeft"
#define ROLL_RIGHT "rollRight"
#define PITCH_DOWN "pitchDown"

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000  // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000  // Maximum pulse length in µs
#define MAX_ANGLE 30
#define THROTTLE_STEP 10
#define ANGLE_STEP 10
/*#define motA 3
#define motB 4
#define motC 5
#define motD 6*/
Servo motA, motB, motC, motD;
#define usPeriod 6000

char data;
MPU6050 sensors;
MPU6050_Angles anglesReaded;
MPU6050_Angles anglesReadedPrev;
int motAPulse, motBPulse, motCPulse, motDPulse;

// PID
struct PIDSetting {
  double kp;
  double ki;
  double kd;
};
PIDSetting pitchAngleSetting = { 0.5, 0.05, 10 };
PIDSetting rollAngleSetting = { 0.5, 0.05, 10 };
PIDSetting yawAngleSetting = { 0.5, 0.05, 10 };
PIDSetting pitchWSetting = { 2, 0.02, 0 };
PIDSetting rollWSetting = { 2, 0.02, 0 };
PIDSetting yawWSetting = { 1, 0.05, 0 };

int pidLimitWI = 380;        // Limitar parte integral PID velocidad
int pidLimitWOut = 380;      // Limitar salida del PID velocidad
int pidLimitAngleI = 130;    // Limitar parte integral PID ángulo
int pidLimitAngleOut = 130;  // Limitar salida del PID ángulo

struct PID {
  float error;
  float p;
  float i;
  float d;
  float out;
};
PID pidAnglePitch, pidAngleRoll, pidAngleYaw, pidWPitch, pidWRoll, pidWYaw;

struct Position {
  int throttle;
  int pitch;
  int roll;
  int yaw;
};
Position posConsigna;

float pidWPitchConsigna, pidWRollConsigna;

long pwmStartAt, loopTimer;

volatile bool dataReceived = false;
String incommingData;


void setup() {

  Serial.begin(9600);
  delay(10);
  Serial.println("Setup");

  /*
  pinMode(motA, OUTPUT);
  pinMode(motB, OUTPUT);
  pinMode(motC, OUTPUT);
  pinMode(motD, OUTPUT);

  digitalWrite(motA, LOW);
  digitalWrite(motB, LOW);
  digitalWrite(motC, LOW);
  digitalWrite(motD, LOW);
  */

  motA.attach(3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  Serial.println("Init Wire to master");

  Wire.begin(DRONE_I2C_ID);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("End Wire to master");

  posConsigna.throttle = MIN_PULSE_LENGTH;
  motAPulse = posConsigna.throttle;
  motBPulse = posConsigna.throttle;
  motCPulse = posConsigna.throttle;
  motDPulse = posConsigna.throttle;

  Serial.println("Init sensors calib");

  if (sensors.init() != 0) {
    Serial.println("Error in MPU6050 connection. Los cables están bien??");
    while (1) { delay(1000); }
  }
  sensors.calibrate();
  Serial.println("End sensors calib");

  loopTimer = micros();
}

void loop() {

  // Esperamos hasta completar el periodo (ciclo)
  while (micros() - loopTimer < usPeriod) {}
  loopTimer = micros();

  // Ejecutamos el ciclo de trabajo de cada ESC de motor.
  pwm();

  // Leemos los sensores.
  sensors.readAngles(anglesReaded);

  // calculamos PID
  pidAngles();
  pidW();

  // modulamos las salidas de PID
  modulator();

  // guardamos una version de los sensores para el siguiente ciclo (usado en PID)
  anglesReadedPrev = anglesReaded;

  // Leemos el requerimiento del usuario, si hay.
  readOrder();
}

void readOrder() {
  if (dataReceived) {
    // Procesar los datos recibidos
    if (isNumber(incommingData)) {
      int number = atoi(incommingData.c_str());
      posConsigna.throttle = number;
    } else {
      // acc va de 1000 a 2000us
      // pitch,roll,yaw van de -30 a 30grados
      // de momento, tendremos botones en los controles, de modo que cada
      // pulsación será de:
      // acc: -10/+10
      // pitch,roll,yaw: -10/+10
      if (incommingData == THROTTLE_UP && posConsigna.throttle < MAX_PULSE_LENGTH - THROTTLE_STEP) {
        posConsigna.throttle += THROTTLE_STEP;
      } else if (incommingData == THROTTLE_DOWN && posConsigna.throttle > MIN_PULSE_LENGTH + THROTTLE_STEP) {
        posConsigna.throttle -= THROTTLE_STEP;
      } else if (incommingData == PITCH_UP && posConsigna.pitch < MAX_ANGLE - ANGLE_STEP) {
        posConsigna.pitch += ANGLE_STEP;
      } else if (incommingData == PITCH_DOWN && posConsigna.pitch > -MAX_ANGLE + ANGLE_STEP) {
        posConsigna.pitch -= ANGLE_STEP;
      } else if (incommingData == ROLL_LEFT && posConsigna.roll < MAX_ANGLE - ANGLE_STEP) {
        posConsigna.roll += ANGLE_STEP;
      } else if (incommingData == ROLL_RIGHT && posConsigna.roll > -MAX_ANGLE + ANGLE_STEP) {
        posConsigna.roll -= ANGLE_STEP;
      } else if (incommingData == YAW_LEFT && posConsigna.yaw < MAX_ANGLE - ANGLE_STEP) {
        posConsigna.yaw += ANGLE_STEP;
      } else if (incommingData == YAW_RIGHT && posConsigna.yaw > -MAX_ANGLE + ANGLE_STEP) {
        posConsigna.yaw -= ANGLE_STEP;
      }
    }

    dataReceived = false;
  }
}

// PID ángulo
void pidAngles() {
  // PID ángulo - PITCH
  pidAnglePitch.error = posConsigna.pitch - anglesReaded.pitch;                            // Error entre lectura y consigna
  pidAnglePitch.p = pitchAngleSetting.kp * pidAnglePitch.error;                            // Parte proporcional
  pidAnglePitch.i += (pitchAngleSetting.ki * pidAnglePitch.error);                         // Parte integral (sumatorio del error en el tiempo)
  pidAnglePitch.i = constrain(pidAnglePitch.i, -pidLimitAngleI, pidLimitAngleI);           // Limitar parte integral
  pidAnglePitch.d = pitchAngleSetting.kd * (anglesReaded.pitch - anglesReadedPrev.pitch);  // Parte derivativa (diferencia entre el error actual y el anterior)

  pidAnglePitch.out = pidAnglePitch.p + pidAnglePitch.i + pidAnglePitch.d;                // Salida PID
  pidAnglePitch.out = constrain(pidAnglePitch.out, -pidLimitAngleOut, pidLimitAngleOut);  // Limitar salida del PID

  // PID ángulo - ROLL
  pidAngleRoll.error = posConsigna.roll - anglesReaded.roll;                           // Error entre lectura y consigna
  pidAngleRoll.p = rollAngleSetting.kp * pidAngleRoll.error;                           // Parte proporcional
  pidAngleRoll.i += (rollAngleSetting.ki * pidAngleRoll.error);                        // Parte integral (sumatorio del error en el tiempo)
  pidAngleRoll.i = constrain(pidAngleRoll.i, -pidLimitAngleI, pidLimitAngleI);         // Limitar parte integral
  pidAngleRoll.d = rollAngleSetting.kd * (anglesReaded.roll - anglesReadedPrev.roll);  // Parte derivativa (diferencia entre el error actual y el anterior)

  pidAngleRoll.out = pidAngleRoll.p + pidAngleRoll.i + pidAngleRoll.d;                  // Salida PID
  pidAngleRoll.out = constrain(pidAngleRoll.out, -pidLimitAngleOut, pidLimitAngleOut);  // Limitar salida del PID
}

void pidW() {

  // En modo estable las consignas de los PID de velocidad vienen de las salidas de los PID de ángulo
  pidWPitchConsigna = pidAnglePitch.out;
  pidWRollConsigna = pidAngleRoll.out;

  // PID velocidad - PITCH
  pidWPitch.error = pidWPitchConsigna - anglesReaded.gyro_X;                             // Error entre lectura y consigna
  pidWPitch.p = pitchAngleSetting.kp * pidWPitch.error;                                  // Parte proporcional
  pidWPitch.i += (pitchAngleSetting.ki * pidWPitch.error);                               // Parte integral (sumatorio del error en el tiempo)
  pidWPitch.i = constrain(pidWPitch.i, -pidLimitWI, pidLimitWI);                         // Limitar parte integral
  pidWPitch.d = pitchAngleSetting.kd * (anglesReaded.gyro_X - anglesReadedPrev.gyro_X);  // Parte derivativa (diferencia entre el error actual y el anterior)

  pidWPitch.out = pidWPitch.p + pidWPitch.i + pidWPitch.d;                // Salida PID
  pidWPitch.out = constrain(pidWPitch.out, -pidLimitWOut, pidLimitWOut);  // Limitar salida del PID

  // PID velocidad - ROLL
  pidWRoll.error = pidWRollConsigna - anglesReaded.gyro_Y;                             // Error entre lectura y consigna
  pidWRoll.p = rollAngleSetting.kp * pidWRoll.error;                                   // Parte proporcional
  pidWRoll.i += (rollAngleSetting.ki * pidWRoll.error);                                // Parte integral (sumatorio del error en el tiempo)
  pidWRoll.i = constrain(pidWRoll.i, -pidLimitWI, pidLimitWI);                         // Limitar parte integral
  pidWRoll.d = rollAngleSetting.kd * (anglesReaded.gyro_Y - anglesReadedPrev.gyro_Y);  // Parte derivativa (diferencia entre el error actual y el anterior)

  pidWRoll.out = pidWRoll.p + pidWRoll.i + pidWRoll.d;                  // Salida PID
  pidWRoll.out = constrain(pidWRoll.out, -pidLimitWOut, pidLimitWOut);  // Limitar salida del PID

  // PID velocidad - YAW
  pidWYaw.error = posConsigna.yaw - anglesReaded.gyro_Z;                             // Error entre lectura y consigna
  pidWYaw.p = yawAngleSetting.kp * pidWYaw.error;                                    // Parte proporcional
  pidWYaw.i += (yawAngleSetting.ki * pidWYaw.error);                                 // Parte integral (sumatorio del error en el tiempo)
  pidWYaw.i = constrain(pidWYaw.i, -pidLimitWI, pidLimitWI);                         // Limitar parte integral
  pidWYaw.d = yawAngleSetting.kd * (anglesReaded.gyro_Z - anglesReadedPrev.gyro_Z);  // Parte derivativa (diferencia entre el error actual y el anterior)

  pidWYaw.out = pidWYaw.p + pidWYaw.i + pidWYaw.d;                    // Salida PID
  pidWYaw.out = constrain(pidWYaw.out, -pidLimitWOut, pidLimitWOut);  // Limitar salida del PID
}

void modulator() {
  // Si el Throttle es menos a 1300us, el control de estabilidad se desactiva. La parte integral
  // de los controladores PID se fuerza a 0.
  if (posConsigna.throttle <= 1300) {
    pidWPitch.i = 0;
    pidWRoll.i = 0;
    pidWYaw.i = 0;
    pidAnglePitch.i = 0;
    pidAngleRoll.i = 0;

    motAPulse = posConsigna.throttle;
    motBPulse = posConsigna.throttle;
    motCPulse = posConsigna.throttle;
    motDPulse = posConsigna.throttle;

    // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    if (motAPulse < 1000) motAPulse = 950;
    if (motBPulse < 1000) motBPulse = 950;
    if (motCPulse < 1000) motCPulse = 950;
    if (motDPulse < 1000) motDPulse = 950;
  }

  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  else {
    // Limitar throttle a 1800 para dejar margen a los PID
    if (posConsigna.throttle > 1800) posConsigna.throttle = 1800;

    // Modulador
    motAPulse = posConsigna.throttle + pidWPitch.out - pidWRoll.out - pidWYaw.out;
    motBPulse = posConsigna.throttle + pidWPitch.out + pidWRoll.out + pidWYaw.out;
    motCPulse = posConsigna.throttle - pidWPitch.out + pidWRoll.out - pidWYaw.out;
    motDPulse = posConsigna.throttle - pidWPitch.out - pidWRoll.out + pidWYaw.out;

    // Evitamos que alguno de los motores de detenga completamente en pleno vuelo
    if (motAPulse < 1100) motAPulse = 1100;
    if (motBPulse < 1100) motBPulse = 1100;
    if (motCPulse < 1100) motCPulse = 1100;
    if (motDPulse < 1100) motDPulse = 1100;
    // Evitamos mandar consignas mayores a 2000us a los motores
    if (motAPulse > 2000) motAPulse = 2000;
    if (motBPulse > 2000) motBPulse = 2000;
    if (motCPulse > 2000) motCPulse = 2000;
    if (motDPulse > 2000) motDPulse = 2000;
  }
}

void pwm() {

  /*
  // Iniciamos DC llevando los motores a HIGHT
  digitalWrite(motA, HIGH);
  digitalWrite(motB, HIGH);
  digitalWrite(motC, HIGH);
  digitalWrite(motD, HIGH);
  pwmStartAt = micros();

  // Terminamos el DC según corresponda a cada motor.
  while (digitalRead(motA) == HIGH || digitalRead(motB) == HIGH || digitalRead(motC) == HIGH || digitalRead(motD) == HIGH) {
    if (pwmStartAt + motAPulse <= micros()) digitalWrite(motA, LOW);
    if (pwmStartAt + motBPulse <= micros()) digitalWrite(motB, LOW);
    if (pwmStartAt + motCPulse <= micros()) digitalWrite(motC, LOW);
    if (pwmStartAt + motDPulse <= micros()) digitalWrite(motD, LOW);
  }
  */


  Serial.print("throttle:");
  Serial.print(posConsigna.throttle);
  Serial.print(" A:");
  Serial.print(motAPulse);
  Serial.print(" B:");
  Serial.print(motBPulse);
  Serial.print(" C:");
  Serial.print(motCPulse);
  Serial.print(" D:");
  Serial.print(motDPulse);
  Serial.println("---------------");
  if (motAPulse >= MIN_PULSE_LENGTH && motAPulse <= MAX_PULSE_LENGTH) {
    motA.writeMicroseconds(motAPulse);
  }
  if (motBPulse >= MIN_PULSE_LENGTH && motBPulse <= MAX_PULSE_LENGTH) {
    motB.writeMicroseconds(motBPulse);
  }
  if (motCPulse >= MIN_PULSE_LENGTH && motCPulse <= MAX_PULSE_LENGTH) {
    motC.writeMicroseconds(motCPulse);
  }
  if (motDPulse >= MIN_PULSE_LENGTH && motDPulse <= MAX_PULSE_LENGTH) {
    motD.writeMicroseconds(motDPulse);
  }
}



void receiveEvent(int count) {
  dataReceived = true;
  incommingData = "";
  while (Wire.available()) {
    char c = Wire.read();
    incommingData += c;
  }
}

void requestEvent() {
  /*char message[35];
  sprintf(message, "%02d|%02d|%02d|%04d|%04d|%04d|%04d|%04d",
          anglesReaded.pitch, anglesReaded.roll, anglesReaded.yaw, posConsigna.throttle, motAPulse, motBPulse, motCPulse, motDPulse);
  */
  String buffer = "";
  buffer += formatFloat(anglesReaded.pitch) + "|";
  buffer += formatFloat(anglesReaded.roll) + "|";
  buffer += formatFloat(anglesReaded.yaw) + "|";
  buffer += String(posConsigna.throttle) + "|";
  buffer += String(motAPulse) + "|";
  buffer += String(motBPulse) + "|";
  buffer += String(motCPulse) + "|";
  buffer += String(motDPulse);

  //Serial.println(anglesReaded.pitch);
  //Serial.println(anglesReaded.roll);
  //Serial.println(anglesReaded.yaw);
  //Serial.println(posConsigna.throttle);
  //Serial.println(motAPulse);
  //Serial.println(motBPulse);
  //Serial.println(motCPulse);
  //Serial.println(motDPulse);
  //Serial.println("================");
  //Serial.print(anglesReaded.pitch);
  //Serial.println("->");
  //Serial.println(buffer.c_str());

  Wire.write(buffer.c_str(), buffer.length());
}

bool isNumber(String str) {
  for (size_t i = 0; i < str.length(); i++) {
    if (!isDigit(str.charAt(i))) {
      return false;
    }
  }
  return true;
}

String formatFloat(float value) {
  char floatString[6];
  dtostrf(value, 5, 2, floatString);
  String formattedString = String(floatString);
  formattedString = formattedString.substring(0, formattedString.length() - 1);
  if (formattedString.length() == 3) {
    formattedString = formattedString.substring(0, 1) + formattedString.substring(2);
  } else if (formattedString.length() == 4) {
    formattedString = formattedString.substring(0, 2);
  }
  formattedString = formattedString.startsWith("-") ? formattedString : "+" + formattedString;
  return formattedString;
}
