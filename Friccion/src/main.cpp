
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "AS5600.h"


#define IN1 25   
#define IN2 26   

#define SDA_PIN 21
#define SCL_PIN 22

float percent = 75;
bool experimentRunning = true;

int percentToPWM(int percent) {
  return map(percent, 0, 100, 0, 255);
}


void setMotorPWM(int pwmValue) {
  analogWrite(IN1, pwmValue);
  analogWrite(IN2, 0);
}

void stopMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}

DFRobot_BMI160 bmi160;
AS5600 as5600;
const int8_t BMI160_ADDR   = 0x68;
const float  ANGLE_OFFSET  = 192.217+28.388;  


float readAngle() {
  return as5600.rawAngle() * AS5600_RAW_TO_DEGREES - ANGLE_OFFSET;
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  if (!as5600.isConnected()) {
    Serial.println("AS5600 no detectado (0x36)");
  } else {
    Serial.println("AS5600 OK");
  }

  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("Reset BMI160 fallo");
    while (1) delay(1000);
  }
  if (bmi160.I2cInit(BMI160_ADDR) != BMI160_OK) {
    Serial.println("Init BMI160 fallo");
    while (1) delay(1000);
  }
  Serial.println("BMI160 OK");
  Serial.println();


}

void loop() {
  int16_t gyro[3] = {0, 0, 0};
  if (experimentRunning) {
    int pwmRaw = percentToPWM(percent);

    setMotorPWM(pwmRaw);
    delay(5000);
    experimentRunning = false;
    Serial.println("INICIO EXPERIMENTO");
    stopMotor();
    
  }
  else {
    
    bool gyroOk = (bmi160.getGyroData(gyro) == BMI160_OK);
    float angle = readAngle();
    if (angle < 0.7 && angle > -0.7 && gyroOk && gyro[2] < 10 && gyro[2] > -10) {
      Serial.println("Pendulo en reposo. FIN");
    }

    Serial.printf("%d,%lu,%.3f,", 0, millis(), angle);
    if (gyroOk) {
      Serial.printf("%d",gyro[2]);
    } else {
      Serial.print("NaN");
    }
    Serial.println();  
  }
}