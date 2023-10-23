#include <Wire.h>
#include <Stepper.h>
#include "Adafruit_TCS34725.h"

// Inicialización del sensor de color
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Definición de constantes para estados de relé
#define RELAY_ON 0
#define RELAY_OFF 1

// Definición de constantes para el motor paso a paso
const int stepsPerRevolution = 2048;
const int motSpeed = 15;

// Inicialización del motor paso a paso
Stepper myStepper(stepsPerRevolution, 2, 4, 3, 6);

void setup() {
  // Inicialización de la comunicación serial
  Serial.begin(9600);
  myStepper.setSpeed(motSpeed);

  // Inicialización del sensor de color
  if (tcs.begin()) {
    Serial.println("Inicializando...");
    delay(2000);
  } else {
    Serial.println("Error...");
    Serial.println("Debe revisar las conexiones...");
    while (1) delay(500);
  }

  // Configuración y apagado inicial de los relés
  int relayPins[] = {8, 9, 10, 11};
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_OFF);
  }
}

void loop() {
  // Lectura de valores de color
  uint16_t R, G, B, C;
  tcs.getRawData(&R, &G, &B, &C);

  // Asegurarse de que los valores de color no superen 255
  R = min(R, 255);
  G = min(G, 255);
  B = min(B, 255);

  // Acumulación de valores de color rojo y verde
  static float red = 0, green = 0;
  red += float(R);
  green += float(G);

  // Contador para acumulación
  static int i = 0;

  if (i > 4) {
    if (red > green && int(B) <= 70) {
      red /= 5;
      myStepper.step(stepsPerRevolution);
      Serial.println("Se detectó el color rojo");
      Serial.println("Rojo = " + String(R) + "\n");

      // Configuración de pines PWM y control de relés
      analogWrite(11, 255);  // Activa color rojo
      analogWrite(10, 0);
      analogWrite(9, 0);
      delay(2000);
      toggleRelay(9, 1000);  // Controla relé 9
      toggleRelay(8, 1000);  // Controla relé 8
    } else if (green > red && int(B) >= 35) {
      green /= 5;
      Serial.println("Se detectó el color verde");
      Serial.println("Verde = " + String(G) + "\n");

      // Configuración de pines PWM y control de relés
      analogWrite(11, 0);
      analogWrite(10, 255);  // Activa color verde
      analogWrite(9, 0);
      delay(2000);
      toggleRelay(11, 1000);  // Controla relé 11
      toggleRelay(10, 1000);  // Controla relé 10
    }
    // Reiniciar acumuladores y contador
    red = 0;
    green = 0;
    i = 0;
  }

  // Imprimir valores de color
  Serial.println("R = " + String(R) + " G = " + String(G) + " B = " + String(B));
  delay(1000);
}

// Función para activar y desactivar un relé con un retardo
void toggleRelay(int pin, int delayTime) {
  digitalWrite(pin, RELAY_ON);
  delay(delayTime);
  digitalWrite(pin, RELAY_OFF);
  delay(delayTime);
}
