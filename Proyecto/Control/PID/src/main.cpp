#include <Arduino.h>
#include <esp_system.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "AS5600.h"

DFRobot_BMI160 bmi160;
const int8_t BMI160_ADDR   = 0x68;
const uint8_t BMI160_GYRO_CONFIG_REG = 0x42;
const uint8_t BMI160_GYRO_RANGE_REG = 0x43;
const uint8_t BMI160_GYRO_DATA_REG = 0x0C;
const uint32_t GYRO_SAMPLE_INTERVAL_MS = 5;
const double GYRO_DEG_PER_COUNT = 1.0 / 131.0; // BMI160 sensitivity at +/-250 deg/s
//double Ki = 766.4;
//double Kp = 965.664;
//double Kd = 331.232;



//double Ki = 126.0;
//double Kp = 260.82;
//double Kd = 118.19;

//double Ki = 188.016;
//double Kp = 94.008;
//double Kd = 11.751;

double Kp = 85.4038;
double Ki = 179.7975;
double Kd = 7.853;

//MUY LENTO
//double Kp = 12.5952;
//double Ki = 30.72;
//double Kd = 1.2718;

const double ANG_TO_DEG =  3.14159/180.0;

double Integral = 0;
double Derivada = 0;
double Proporcional = 0;
double Ts = 5; 
double angle = 0;
double error = 0;
double lastError = 0;
double u = 0;
double last_Samble_time = 0;
double last_Samble_time_Ramp = 0;
double reference = 60*ANG_TO_DEG;  // 70  grados, en radianes
double N = 500;
double last_Derivada = 0;
double sat_time;

uint32_t last_sample_bmi = 0;


// Red de adelanto
const float a = 0.0530;    // ejemplo T2 en segundos
const float b  = 4.5625;     // ejemplo b
float RED;
float lastRED;
float lastu;
const float c1 = 1+b*2*a/Ts;
const float c2 = 1-b*2*a/Ts;
const float c3 = 1+2*a/Ts;
const float c4 = 1-2*a/Ts;

#define IN1 25
#define IN2 26
#define PWM_FREQUENCY 20000

#define SDA_PIN 21
#define SCL_PIN 22



AS5600 as5600;

const float  ANGLE_OFFSET  = (173.16 + 9.65)*ANG_TO_DEG;  // 173.16 + 9.65 grados, en radianes


int percentToPWM(int percent) {
  return map(constrain(percent, 0, 100), 0, 100, 0, 255);
}

void setMotorPWM(int pwmValue) {
  analogWrite(IN1, pwmValue);
  analogWrite(IN2, 0);
}
float readAngle() {
  return as5600.rawAngle()*AS5600_RAW_TO_RADIANS - ANGLE_OFFSET;
}

bool readBMI160Registers(uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(static_cast<uint8_t>(BMI160_ADDR));
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(static_cast<uint8_t>(BMI160_ADDR), length) != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!Wire.available()) {
      return false;
    }
    data[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool writeBMI160Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(static_cast<uint8_t>(BMI160_ADDR));
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool configureBMI160Gyro() {
  uint8_t configRange[2];
  if (!readBMI160Registers(BMI160_GYRO_CONFIG_REG, configRange, sizeof(configRange))) {
    return false;
  }

  const uint8_t gyroConfig =
      (configRange[0] & 0xF0) | BMI160_GYRO_ODR_3200HZ;
  const uint8_t gyroRange =
      (configRange[1] & 0xF8) | BMI160_GYRO_RANGE_250_DPS;
  if (!writeBMI160Register(BMI160_GYRO_CONFIG_REG, gyroConfig) ||
      !writeBMI160Register(BMI160_GYRO_RANGE_REG, gyroRange) ||
      !readBMI160Registers(BMI160_GYRO_CONFIG_REG, configRange, sizeof(configRange))) {
    return false;
  }

  return (configRange[0] & 0x0F) == BMI160_GYRO_ODR_3200HZ &&
         (configRange[1] & 0x07) == BMI160_GYRO_RANGE_250_DPS;
}

void stopMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}
double derivativo(double error, double lastError, double Ts, double N, double Kd, double &last_Derivada) {
  Derivada = (Kd * N / (1.0 + N * Ts)) * (error - lastError) + (1.0 / (1.0 + N * Ts)) * last_Derivada;
  last_Derivada = Derivada;
  return Derivada;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogWriteFrequency(PWM_FREQUENCY);
  stopMotor();

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
  if (!configureBMI160Gyro()) {
    Serial.println("Config gyro BMI160 fallo");
    while (1) delay(1000);
  }
  Serial.println("BMI160 gyro: 3200 Hz, rango +/-125 grados/s");
  Serial.println();
  last_sample_bmi = millis();
  
  
}

void loop() {
  if (millis() - last_Samble_time >= Ts) {
    angle = readAngle();
    error = reference - angle; 

    Integral += Ki * error * (Ts / 1000.0);

    Derivada = derivativo(error, lastError, Ts / 1000.0, N, Kd, last_Derivada);
    //Derivada = Kd*(error - lastError) / (Ts / 1000.0);

    Proporcional = Kp * error;
    
    u = Proporcional + Integral + Derivada;
    lastu = u;
    if (u > 100) {
      u = 100;
      //Integral = 0;
      
    }
    if (u < 0) {
      u = 0;
      //Integral = 0;
    }
    
    
    //RED = (u*c1+lastu*c2-lastRED*c4)/c3;
    //lastRED = RED;
    //if (RED > 100) RED = 100;
    //if (RED < 0) RED = 0;

    lastError = error;
    
    if (millis() - last_sample_bmi >= GYRO_SAMPLE_INTERVAL_MS) {
      uint8_t gyroData[6];
      if (readBMI160Registers(BMI160_GYRO_DATA_REG, gyroData, sizeof(gyroData))) {
        const uint16_t rawGyroZ =
            static_cast<uint16_t>(gyroData[4]) |
            (static_cast<uint16_t>(gyroData[5]) << 8);
        const int16_t gyroZ = static_cast<int16_t>(rawGyroZ);
        Serial.printf(">V_ang[deg/s]:%.2f\r\n", gyroZ * GYRO_DEG_PER_COUNT);
      } else {
        Serial.println(">Error:lectura gyro BMI160");
      }
      last_sample_bmi = millis();
    }
    Serial.printf(">Referencia[°]:%.2f\r\n", reference / ANG_TO_DEG);
    Serial.printf(">Angulo[°]:%.2f\r\n", angle / ANG_TO_DEG);
    Serial.printf(">Error[°]:%.2f\r\n", error/ANG_TO_DEG);
    Serial.printf(">Control[PWM]:%.2f\r\n", u);
    Serial.printf(">Control_no_sat[PWM]:%.2f\r\n", lastu);
    

    
    //Serial.printf("Referencia: %.2f, Angulo: %.2f, Error: %.2f, Control: %.2f, control no sat: %.2f",  reference/ANG_TO_DEG, angle/ANG_TO_DEG, error, u, lastu);
    setMotorPWM(percentToPWM(u));
    last_Samble_time = millis();
  }
  

  //if(millis() - last_Samble_time_Ramp >= 4000) {
  //    last_Samble_time_Ramp = millis();
  //    
  //    if (reference > 178*ANG_TO_DEG) {
  //      
  //    }
  //    else{
  //      reference += 4*ANG_TO_DEG;
  //    }
  //}
}