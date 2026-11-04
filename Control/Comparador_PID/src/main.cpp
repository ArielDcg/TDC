// =============================================================
//  Respuesta dinamica - Planta Pendulo Invertido
//  Recibe por serial una secuencia de PWM (%) y el tiempo por escalon,
//  la aplica y transmite la respuesta de forma continua.
//
//  Protocolo serial (115200):
//    ESP32  -> PC : "ESPERANDO SECUENCIA"  (cada 1 s mientras espera)
//    PC     -> ESP32 : "SEQ,<step_ms>,<n>,<p1>,<p2>,...,<pn>"   (una linea, LF)
//    ESP32  -> PC : "ACK,<step_ms>,<n>"    (o "ERROR,<motivo>")
//    ESP32  -> PC : "INICIO EXPERIMENTO"
//    ESP32  -> PC : filas "%PWM,Tiempo_ms,Angulo_deg,GyroZ"
//                   (Tiempo_ms = ms desde el inicio de la secuencia)
//    ESP32  -> PC : "EXPERIMENTO COMPLETADO"
//
//  Comando 'x' durante la secuencia: detiene el motor y vuelve a esperar.
// =============================================================

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "AS5600.h"

double Kp = 94.008;
double Ki = 188.016;
double Kd = 11.751;


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
double reference = 0;  // 70  grados, en radianes
double N = 1000;
double last_Derivada = 0;



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


DFRobot_BMI160 bmi160;
const int8_t BMI160_ADDR  = 0x68;


// ---- Parametros ----
const unsigned long SAMPLE_MS = 20;     // ms entre filas enviadas (50 Hz)
const int           MAX_LEVELS = 200;   // maximo de niveles en la secuencia
const unsigned int  MAX_LINE   = 4096;  // maximo de caracteres de la linea SEQ
const unsigned long MIN_STEP_MS = 100;  // tiempo minimo por escalon
const unsigned long MAX_STEP_MS = 600000;

// ---- Secuencia recibida ----
int seq[MAX_LEVELS];
int nSeq = 0;
unsigned long stepMs = 0;


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
float readAngle() {
  return as5600.rawAngle() * AS5600_RAW_TO_DEGREES - ANGLE_OFFSET;
}


// =============================================================
//  Interpreta "SEQ,<step_ms>,<n>,<p1>,...,<pn>"
//  Devuelve true si es valida; si no, deja el motivo en err
// =============================================================
bool parseSequence(const String &line, String &err) {
  char *buf = strdup(line.c_str());
  if (!buf) { err = "sin memoria"; return false; }

  char *save = nullptr;
  char *tok = strtok_r(buf, ",", &save);
  if (!tok || strcmp(tok, "SEQ") != 0) {
    err = "comando desconocido";
    free(buf);
    return false;
  }

  tok = strtok_r(nullptr, ",", &save);
  long step = tok ? atol(tok) : 0;
  tok = strtok_r(nullptr, ",", &save);
  int n = tok ? atoi(tok) : 0;

  if (step < (long)MIN_STEP_MS || step > (long)MAX_STEP_MS) {
    err = "step_ms fuera de rango";
    free(buf);
    return false;
  }
  if (n < 1 || n > MAX_LEVELS) {
    err = "n fuera de rango";
    free(buf);
    return false;
  }

  int count = 0;
  while ((tok = strtok_r(nullptr, ",", &save)) != nullptr) {
    if (count >= MAX_LEVELS) {
      err = "demasiados valores";
      free(buf);
      return false;
    }
    int p = atoi(tok);
    if (p < 0 || p > 100) {
      err = "porcentaje fuera de 0..100";
      free(buf);
      return false;
    }
    seq[count++] = p;
  }
  free(buf);

  if (count != n) {
    err = "n no coincide con la cantidad de valores";
    return false;
  }

  stepMs = (unsigned long)step;
  nSeq = n;
  return true;
}


// =============================================================
//  Aplica la secuencia y transmite la respuesta
// =============================================================
void runSequence() {
  Serial.println("INICIO EXPERIMENTO");

  const unsigned long total = stepMs * (unsigned long)nSeq;
  int currentLevel = -1;
  unsigned long nextSample = 0;
  unsigned long tStart = millis();

  while (true) {
    unsigned long elapsed = millis() - tStart;
    if (elapsed >= total) break;

    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'x' || cmd == 'X') {
        stopMotor();
        Serial.println("\n[!] Experimento detenido por el usuario.");
        return;
      }
    }

    // Cambio de nivel segun el tiempo transcurrido
    int lvl = (int)(elapsed / stepMs);
    if (lvl != currentLevel) {
      currentLevel = lvl;
      reference = seq[currentLevel]*ANG_TO_DEG;
    }

    // Muestreo periodico
    if (elapsed >= nextSample) {
      nextSample += SAMPLE_MS;

      int16_t gyro[3] = {0, 0, 0};
      bool gyroOk = (bmi160.getGyroData(gyro) == BMI160_OK);

      Serial.printf("%d,%lu,%.3f,", seq[currentLevel], elapsed, readAngle());
      if (gyroOk) {
        Serial.printf("%d\n", gyro[2]);
      } else {
        Serial.println("NaN");
      }
    }
  }

  stopMotor();
  Serial.println("EXPERIMENTO COMPLETADO");
}


void handleLine(String line) {
  String err;
  if (!parseSequence(line, err)) {
    Serial.print("ERROR,");
    Serial.println(err);
    return;
  }
  Serial.printf("ACK,%lu,%d\n", stepMs, nSeq);
  runSequence();
}


void setup() {
  Serial.setRxBufferSize(2048);   // la linea SEQ puede pasar de 256 bytes
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
  Serial.println();
}


void loop() {
  static String line;
  static unsigned long lastPrompt = 0;

  // Aviso periodico mientras espera la secuencia
  if (millis() - lastPrompt >= 1000) {
    lastPrompt = millis();
    Serial.println("ESPERANDO SECUENCIA");
  }

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      line.trim();
      if (line.length() > 0) {
        handleLine(line);
      }
      line = "";
      lastPrompt = millis();
    } else if (line.length() < MAX_LINE) {
      line += ch;
    } else {
      line = "";
      Serial.println("ERROR,linea demasiado larga");
    }
  }
  if (millis() - last_Samble_time >= Ts) {
    angle = readAngle();
    error = reference - angle; 

    Integral += Ki * error * (Ts / 1000.0);

    //Derivada = derivativo(error, lastError, Ts, N, Kd, last_Derivada);
    Derivada = Kd*(error - lastError) / (Ts / 1000.0);

    Proporcional = Kp * error;
    lastu = u;
    u = Proporcional + Integral + Derivada;
    
    //RED = (u*c1+lastu*c2-lastRED*c4)/c3;
    //lastRED = RED;
    //if (RED > 100) RED = 100;
    //if (RED < 0) RED = 0;

    if (u > 100) u = 100;
    if (u < 0) u = 0;
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