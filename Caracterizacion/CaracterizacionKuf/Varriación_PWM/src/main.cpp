#include <Arduino.h>
#include <esp_system.h>
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
const float  ANGLE_OFFSET  = 220.342;

// ---- Parámetros del experimento ----
const unsigned long HOLD_TIME   = 3000; // ms mínimos tras cambiar el PWM antes de evaluar estabilidad (no se envía nada)
const unsigned long SAMPLE_MS   = 100;  // ms entre lecturas
const unsigned long RAMP_DELAY  = 0;    // ms de pausa entre niveles

// ---- Criterio de estabilidad ----
// Solo se envían datos cuando |GyroZ| <= GYRO_STABLE, y no se pasa al siguiente
// nivel hasta tener STABLE_COUNT lecturas estables consecutivas.
const int GYRO_STABLE  = 1000;   // Umbral de GyroZ (cuentas crudas)
const int STABLE_COUNT = 5;   // Lecturas estables seguidas para dar el nivel por terminado

// ---- Modo de operación ----
//   SWEEP       : barrido de PWM_START a PWM_END (sube o baja según los valores)
//   ROUND_TRIP  : barrido de PWM_START a PWM_END y regreso hasta PWM_START
//   RANDOM_LIST : RAND_COUNT niveles aleatorios entre RANK_MIN y RANK_MAX, en ese orden
enum Mode { SWEEP, ROUND_TRIP, RANDOM_LIST };
const Mode MODE = ROUND_TRIP;

// Parámetros de SWEEP y ROUND_TRIP
const int PWM_START = 0;    // Primer nivel (%)
const int PWM_END   = 90;   // Nivel extremo (%)
const int PWM_STEP  = 2;    // Tamaño del paso en %, siempre positivo

// Parámetros de RANDOM_LIST
const int RAND_COUNT = 100;  // Cantidad de niveles de la lista
const int RANK_MIN   = 0;    // Porcentaje mínimo (%)
const int RANK_MAX   = 80;   // Porcentaje máximo (%)

// ---- Lista de niveles que se ejecutará ----
constexpr int MAX_LEVELS = 256;
constexpr int SWEEP_N = ((PWM_END >= PWM_START) ? (PWM_END - PWM_START) : (PWM_START - PWM_END)) / PWM_STEP;

static_assert(PWM_STEP > 0, "PWM_STEP debe ser positivo; el sentido sale de PWM_START y PWM_END");
static_assert(2 * SWEEP_N + 1 <= MAX_LEVELS, "Demasiados niveles en el barrido: sube PWM_STEP o MAX_LEVELS");
static_assert(RAND_COUNT > 0 && RAND_COUNT <= MAX_LEVELS, "RAND_COUNT fuera de rango (1..MAX_LEVELS)");
static_assert(RANK_MIN >= 0 && RANK_MAX <= 100 && RANK_MIN <= RANK_MAX, "RANK_MIN/RANK_MAX deben cumplir 0 <= MIN <= MAX <= 100");

int levels[MAX_LEVELS];
int nLevels = 0;

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

// Llena levels[] segun el modo elegido
void buildLevels() {
  nLevels = 0;
  const int dir = (PWM_END >= PWM_START) ? 1 : -1;

  switch (MODE) {
    case SWEEP:
      for (int i = 0; i <= SWEEP_N; i++) {
        levels[nLevels++] = PWM_START + dir * i * PWM_STEP;
      }
      break;

    case ROUND_TRIP:
      // Ida: START -> extremo
      for (int i = 0; i <= SWEEP_N; i++) {
        levels[nLevels++] = PWM_START + dir * i * PWM_STEP;
      }
      // Vuelta: extremo -> START (sin repetir el nivel extremo)
      for (int i = SWEEP_N - 1; i >= 0; i--) {
        levels[nLevels++] = PWM_START + dir * i * PWM_STEP;
      }
      break;

    case RANDOM_LIST:
      randomSeed(esp_random());
      for (int i = 0; i < RAND_COUNT; i++) {
        levels[nLevels++] = random(RANK_MIN, RANK_MAX + 1);
      }
      break;
  }
}

// Imprime la lista de niveles (empieza con texto, asi que no se confunde con una fila de datos)
void printLevels() {
  const char *names[] = {"SWEEP", "ROUND_TRIP", "RANDOM_LIST"};
  Serial.printf("MODO: %s | %d niveles\n", names[MODE], nLevels);
  Serial.print("LISTA: ");
  for (int i = 0; i < nLevels; i++) {
    Serial.print(levels[i]);
    if (i < nLevels - 1) Serial.print(",");
  }
  Serial.println();
}

void runPWMLevel(int stepNum, int percent) {
  int pwmRaw = percentToPWM(percent);

  setMotorPWM(pwmRaw);

  unsigned long startTime = millis();
  unsigned long lastSample = 0;
  int stableCount = 0;

  while (true) {

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

      // Tiempo de asentamiento: no se evalua estabilidad ni se envia nada
      if (millis() - startTime < HOLD_TIME) continue;

      int16_t gyro[3] = {0, 0, 0};
      bool gyroOk = (bmi160.getGyroData(gyro) == BMI160_OK);
      bool stable = gyroOk && abs(gyro[2]) <= GYRO_STABLE;

      if (stable) {
        stableCount++;
        Serial.printf("%d,%lu,%.3f,%d\n", percent, millis(), readAngle(), gyro[2]);
      } else {
        stableCount = 0;
      }

      // Solo se pasa al siguiente nivel cuando ya lleva estable el tiempo suficiente
      if (stableCount >= STABLE_COUNT){ 
        stableCount = 0;
        return;
      }
    }
  }
}


void runExperiment() {
  buildLevels();
  printLevels();

  for (int i = 0; i < nLevels; i++) {
    if (!experimentRunning) break;

    runPWMLevel(i + 1, levels[i]);

    if (experimentRunning && i < nLevels - 1) {
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