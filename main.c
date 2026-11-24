#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>

// ---------------------------------
// CONFIGURACION DE PINES
// ---------------------------------

const uint8_t MPU = 0x68;

const int SDA_PIN = 13;
const int SCL_PIN = 14;

const int SERVO_PIN = 27;
const int SWITCH_PIN = 26;

// ---------------------------------
// CONFIGURACION DEL SENSOR
// ---------------------------------

const float ACCEL_SENS = 8192.0;
const float GYRO_SENS = 65.5;

const float alpha = 0.98;

const float UMBRAL_IMPACTO = 2.5;

const unsigned long BLOQUEO_MS = 1000;

// Frecuencia de muestreo: 1000 Hz
const unsigned long PERIODO_US = 1000;

// ---------------------------------
// VARIABLES
// ---------------------------------

Servo servoAirbag;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

float roll = 0;
float pitch = 0;
float aceleracionTotal = 0;

unsigned long ultimaMuestra = 0;
unsigned long ultimoImpacto = 0;

bool airbagDesplegado = false;

// ---------------------------------
// ESCRIBIR REGISTRO
// ---------------------------------

bool escribirRegistro(uint8_t registro, uint8_t valor) {

  Wire.beginTransmission(MPU);
  Wire.write(registro);
  Wire.write(valor);

  return Wire.endTransmission() == 0;
}

// ---------------------------------
// LEER EJE
// ---------------------------------

int16_t leerEje() {

  uint16_t alto = Wire.read();
  uint16_t bajo = Wire.read();

  return (int16_t)((alto << 8) | bajo);
}

// ---------------------------------
// LEER SENSOR
// ---------------------------------

bool leerSensor() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU, (uint8_t)14, true) != 14) {
    return false;
  }

  int16_t rawAx = leerEje();
  int16_t rawAy = leerEje();
  int16_t rawAz = leerEje();

  leerEje(); // Temperatura

  int16_t rawGx = leerEje();
  int16_t rawGy = leerEje();
  int16_t rawGz = leerEje();

  ax = rawAx / ACCEL_SENS;
  ay = rawAy / ACCEL_SENS;
  az = rawAz / ACCEL_SENS;

  gx = rawGx / GYRO_SENS - offsetX;
  gy = rawGy / GYRO_SENS - offsetY;
  gz = rawGz / GYRO_SENS - offsetZ;

  return true;
}

// ---------------------------------
// CALIBRAR GIROSCOPIO
// ---------------------------------

void calibrarGiroscopio() {

  long sumaX = 0;
  long sumaY = 0;
  long sumaZ = 0;

  int validas = 0;

  for (int i = 0; i < 500; i++) {

    Wire.beginTransmission(MPU);
    Wire.write(0x43);

    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(MPU, (uint8_t)6, true) == 6) {

      sumaX += leerEje();
      sumaY += leerEje();
      sumaZ += leerEje();

      validas++;
    }

    delay(5);
  }

  if (validas > 0) {

    offsetX = (sumaX / (float)validas) / GYRO_SENS;
    offsetY = (sumaY / (float)validas) / GYRO_SENS;
    offsetZ = (sumaZ / (float)validas) / GYRO_SENS;
  }
}

// ---------------------------------
// FILTRO COMPLEMENTARIO
// ---------------------------------

void calcularFiltro(float dt) {

  float rollAcc = atan2(ay, az) * 180.0 / PI;

  float pitchAcc = atan2(
    -ax,
    sqrt(ay * ay + az * az)
  ) * 180.0 / PI;

  roll = alpha * (roll + gx * dt)
       + (1.0 - alpha) * rollAcc;

  pitch = alpha * (pitch + gy * dt)
        + (1.0 - alpha) * pitchAcc;
}

// ---------------------------------
// ESTADO DEL CINTURON
// ---------------------------------

bool cinturonAbrochado() {

  return digitalRead(SWITCH_PIN) == LOW;
}

// ---------------------------------
// DESPLEGAR AIRBAG
// ---------------------------------

void desplegarAirbag() {

  if (airbagDesplegado) {
    return;
  }

  if (cinturonAbrochado()) {

    servoAirbag.write(90);

    airbagDesplegado = true;
  }
}

// ---------------------------------
// DETECCION DE IMPACTO
// ---------------------------------

void detectarImpacto() {

  aceleracionTotal = sqrt(
    ax * ax + ay * ay + az * az
  );

  unsigned long ahora = millis();

  if (aceleracionTotal >= UMBRAL_IMPACTO &&
      ahora - ultimoImpacto >= BLOQUEO_MS &&
      !airbagDesplegado) {

    ultimoImpacto = ahora;

    desplegarAirbag();
  }
}

// ---------------------------------
// SETUP
// ---------------------------------

void setup() {

  Serial.begin(921600);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  servoAirbag.setPeriodHertz(50);
  servoAirbag.attach(SERVO_PIN, 500, 2400);

  servoAirbag.write(0);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  delay(500);

  // Despertar MPU6050
  bool configurado = escribirRegistro(0x6B, 0x00);

  // Configurar acelerometro: +/- 4g
  configurado &= escribirRegistro(0x1C, 0x08);

  // Configurar giroscopio: +/- 500 grados/s
  configurado &= escribirRegistro(0x1B, 0x08);

  // Configurar filtro digital DLPF
  configurado &= escribirRegistro(0x1A, 0x03);

  // Divisor de frecuencia: 0
  // Frecuencia de salida: 1000 Hz
  configurado &= escribirRegistro(0x19, 0x00);

  if (!configurado) {

    while (true) {
      delay(1000);
    }
  }

  delay(200);

  calibrarGiroscopio();

  if (leerSensor()) {

    roll = atan2(ay, az) * 180.0 / PI;

    pitch = atan2(
      -ax,
      sqrt(ay * ay + az * az)
    ) * 180.0 / PI;
  }

  ultimaMuestra = micros();

  ultimoImpacto = millis() - BLOQUEO_MS;
}

// ---------------------------------
// LOOP
// ---------------------------------

void loop() {

  unsigned long ahora = micros();

  if (ahora - ultimaMuestra < PERIODO_US) {
    return;
  }

  float dt = (ahora - ultimaMuestra) / 1000000.0;

  ultimaMuestra = ahora;

  if (!leerSensor()) {
    return;
  }

  calcularFiltro(dt);

  detectarImpacto();

  // ENVIO DE DATOS AL SERIAL PLOTTER

  Serial.print("Roll:");
  Serial.print(roll, 2);

  Serial.print("\tPitch:");
  Serial.print(pitch, 2);

  Serial.print("\tAceleracion:");
  Serial.println(aceleracionTotal, 2);
}
