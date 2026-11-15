#include <Arduino.h>
#include <Wire.h>

// =====================
// CONFIGURACIÓN I2C (AS5600)
// =====================
const uint8_t AS5600_ADDR = 0x36;
const uint8_t RAW_ANGLE_HI = 0x0C; // Registro alto del ángulo

// =====================
// CONFIGURACIÓN PWM (ESP32)
// =====================
const int PWM_PIN = 25;       // ESP32: El pin 9 suele estar reservado para flash interna. Recomendado usar 25, 26, 27, 32 o 33.
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES = 10;       // Resolución de 10 bits (0-1023)
const int PWM_MAX = 1023;     // Equivalente al antiguo ICR1 para 100% duty cycle

// =====================
// CONFIGURACIÓN PID
// =====================
float kp = 0.2;
float ki = 0.36;
float kd = 0.9;
float N  = 100.0;     // filtro derivativo
float D = 0.0;

// Variables internas PID
float error, lastError = 0.0;
float integral = 0.0;
float derivative;
float PID;
float lastPID;

// Tiempo para el muestreo de 1 ms
unsigned long LastSample = 0;
const unsigned long Ts = 1000;   // 1 ms = 1000 microsegundos

// --- Parámetros de la rampa ---
float setpoint = 0.0;            // Punto actual de la referencia
float targetSetpoint = 120.0;     // Donde queremos llegar (ej. 90 grados)
float slope = 3.0;               // Velocidad de cambio (grados por segundo)
const float dt = 0.001;          // 1 ms

// Red de adelanto
const float a = 0.00097649;    // ejemplo T2 en segundos
const float b  = 13.0024;       // ejemplo b
float RED;
float lastRED;
const float c1 = 1 + b * 2 * a / dt;
const float c2 = 1 - b * 2 * a / dt;
const float c3 = 1 + 2 * a / dt;
const float c4 = 1 - 2 * a / dt;

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  
  // Iniciar bus I2C (SDA=21, SCL=22 por defecto en ESP32)
  Wire.begin();
  Wire.setClock(400000); // 400kHz (Fast Mode) crucial para no romper el Ts de 1ms

  // Configurar PWM en ESP32
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);  // duty inicial 0
}

// =====================
// FUNCIÓN LECTURA AS5600
// =====================
int readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HI);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  
  if (Wire.available() >= 2) {
    uint8_t high = Wire.read();
    uint8_t low  = Wire.read();
    return ((high << 8) | low) & 0x0FFF; // Asegurar 12 bits
  }
  return -1; // En caso de error de lectura
}

// =====================
// LOOP PRINCIPAL
// =====================
void loop() {
  // Ejecutar PID cada 1 ms
  if (micros() - LastSample >= Ts) {
    LastSample = micros();

    // ------------------------------------------------------
    //     GENERACIÓN DE LA RAMPA PARA LA REFERENCIA
    // ------------------------------------------------------
    float delta = slope * dt;   // cuánto avanza la referencia cada 1 ms

    if (setpoint < targetSetpoint) {
      setpoint += delta;
      if (setpoint > targetSetpoint) setpoint = targetSetpoint;
    }
    else if (setpoint > targetSetpoint) {
      setpoint -= delta;
      if (setpoint < targetSetpoint) setpoint = targetSetpoint;
    }

    // =======================
    // Lectura del AS5600 vía I2C
    // =======================
    int rawValue = readAS5600();
    float angle = 0.0;
    
    if(rawValue != -1) {
      // El AS5600 es de 12 bits (0–4095) en lugar de los 10 bits del ADC análogo
      angle = (rawValue * 360.0) / 4095.0 - (88.33+94.44);   
    }

    // =======================
    // Cálculo del PID
    // =======================
    error = setpoint - angle;

    integral += error * 0.001;               // dt = 1 ms = 0.001 s

    float Td = kd;
    float denom = Td + N * dt;
    float alpha = Td / denom;
    float beta  = kd * N / denom;

    D = alpha * D + beta * (error - lastError);
    PID = kp * error + ki * integral + D;

    lastError = error;
    lastPID = PID;

    RED = (PID * c1 + lastPID * c2 - lastRED * c4) / c3;
    lastRED = RED;

    // =======================
    // Convertir PID a PWM
    // =======================
    float pwmValue = constrain(RED, 0.0, 100.0);
    int conversion = (pwmValue * PWM_MAX) / 100;
    
    // Escribir el hardware PWM del ESP32
    ledcWrite(PWM_CHANNEL, conversion);

    // =======================
    // (Opcional) imprimir cada cierto tiempo
    // =======================
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print("   error: ");
      Serial.print(error);
      Serial.print("   Ángulo: ");
      Serial.print(angle);
      Serial.print("   PWM%: ");
      Serial.print(pwmValue, 5);
      Serial.print("   PID: ");
      Serial.print(PID, 5);
      Serial.print("   RED: ");
      Serial.println(RED, 5);
    }
  }
}