#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"


//double Ki = 766.4;
//double Kp = 965.664;
//double Kd = 331.232;



//double Ki = 126.0;
//double Kp = 260.82;
//double Kd = 118.19;

double Kp = 94.008;
double Ki = 188.016;
double Kd = 11.751;

//MUY LENTO
//double Kp = 12.5952;
//double Ki = 30.72;
//double Kd = 1.2718;

const double ANG_TO_DEG =  3.14159/180.0;

double Integral = 0;
double Derivada = 0;
double Proporcional = 0;
double Ts = 2; //2000 Hz
double angle = 0;
double error = 0;
double lastError = 0;
double u = 0;
double last_Samble_time = 0;
double last_Samble_time_Ramp = 0;
double reference = 130*ANG_TO_DEG;  // 70  grados, en radianes
double N = 1000;
double last_Derivada = 0;
double sat_time;


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

void stopMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}
double derivativo(double error, double lastError, double Ts, double N, double Kd, double last_Derivada) {
  Derivada = Kd * N / (1 + N * Ts) * (error - lastError) + (1 ) / (1 + N * Ts) * last_Derivada;
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
  
  
}

void loop() {
  if (millis() - last_Samble_time >= Ts) {
    angle = readAngle();
    error = reference - angle; 

    Integral += Ki * error * (Ts / 1000.0);

    //Derivada = derivativo(error, lastError, Ts, N, Kd, last_Derivada);
    Derivada = Kd*(error - lastError) / (Ts / 1000.0);

    Proporcional = Kp * error;
    lastu = u;
    u = Proporcional + Integral + Derivada;
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
    last_Samble_time = millis();
    Serial.printf("Referencia: %.2f, Angulo: %.2f, Error: %.2f, Control: %.2f\n",  reference/ANG_TO_DEG, angle/ANG_TO_DEG, error, u);
    setMotorPWM(percentToPWM(u));
  }

  if(millis() - last_Samble_time_Ramp >= 4000) {
      last_Samble_time_Ramp = millis();
      
      if (reference > 172*ANG_TO_DEG) {
        
      }
      else{
        reference += 4*ANG_TO_DEG;
      }
  }
}