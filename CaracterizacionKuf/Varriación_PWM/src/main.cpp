#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "AS5600.h"


#define IN1 25
#define IN2 26

#define SDA_PIN 21
#define SCL_PIN 22


DFRobot_BMI160 bmi160;
AS5600 as5600;
const int8_t BMI160_ADDR   = 0x68;
const float  ANGLE_OFFSET  = 192.217 + 28.388;

// ---- Parámetros del experimento ----
const unsigned long HOLD_TIME   = 5000; // ms que se mantiene cada nivel de PWM
const unsigned long SAMPLE_MS   = 100;  // ms entre filas CSV
const unsigned long RAMP_DELAY  = 0;    // ms de pausa entre niveles


const int PWM_START = 90;   // Primer nivel (%)
const int PWM_END   = 10;   // Último nivel (%)
const int PWM_STEP  = 2;    // Tamaño del paso en %, siempre positivo

static_assert(PWM_STEP > 0, "PWM_STEP debe ser positivo; el sentido sale de PWM_START y PWM_END");

// ---- Variables globales ----
bool experimentRunning = true;

int percentToPWM(int percent) {
  return map(constrain(percent, 0, 100), 0, 100, 0, 255);
}


void setMotorPWM(int pwmValue) {
  analogWrite(IN1, pwmValue);
  analogWrite(IN2, 0);
}

void stopMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
}


float readAngle() {
  return as5600.rawAngle() * AS5600_RAW_TO_DEGREES - ANGLE_OFFSET;
}


void printHeader() {
  Serial.println("==============================================");
  Serial.println("  CARACTERIZACION PWM vs FUERZA - Kuf");
  Serial.println("  Pendulo Invertido");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("Instrucciones:");
  Serial.println("  - Coloca el dinamometro / celda de carga");
  Serial.println("    contra el carro del pendulo.");
  Serial.println("  - Registra la fuerza [N] para cada nivel.");
  Serial.println("  - Kuf = Fuerza [N] / Porcentaje [%]");
  Serial.println();
  Serial.println("%PWM,Tiempo_ms,Angulo_deg,GyroZ");
}

void runPWMLevel(int stepNum, int percent) {
  int pwmRaw = percentToPWM(percent);

  setMotorPWM(pwmRaw);

  unsigned long startTime = millis();
  unsigned long lastSample = 0;

  while (millis() - startTime < HOLD_TIME) {

    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'x' || cmd == 'X') {
        stopMotor();
        experimentRunning = false;
        Serial.println("\n[!] Experimento detenido por el usuario.");
        return;
      }
    }

    if (millis() - lastSample >= SAMPLE_MS) {
      lastSample = millis();

      int16_t gyro[3] = {0, 0, 0};
      bool gyroOk = (bmi160.getGyroData(gyro) == BMI160_OK);

      Serial.printf("%d,%lu,%.3f,", percent, millis(), readAngle());
      if (gyroOk) {
        Serial.printf("%d", gyro[2]);
      } else {
        Serial.print("NaN");
      }
      Serial.println();
    }
  }
}


void runExperiment() {
  int stepNum = 1;


  const int dir = (PWM_END >= PWM_START) ? 1 : -1;

  const int n = abs(PWM_END - PWM_START) / PWM_STEP;

  for (int i = 0; i <= n; i++) {
    if (!experimentRunning) break;

    int pct = PWM_START + dir * i * PWM_STEP;
    runPWMLevel(stepNum, pct);
    stepNum++;

    if (experimentRunning && i < n) {
      delay(RAMP_DELAY);
    }
  }

  if (experimentRunning) {
    stopMotor();
    Serial.println();
    Serial.println("==============================================");
    Serial.println("  EXPERIMENTO COMPLETADO");
    Serial.println("==============================================");
    experimentRunning = false;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
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
  Serial.println();

  printHeader();
}

void loop() {
  runExperiment();
}