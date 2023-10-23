// Incluye las librerías necesarias
#include <Wire.h>
#include <Stepper.h>
#include "Adafruit_TCS34725.h"

// Inicializa el sensor de color
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Define constantes para estados de relés
#define RELAY_ON 0    
#define RELAY_OFF 1

// Define constantes para el motor paso a paso
const int stepsPerRevolution = 2048;
const int motSpeed = 15;

// Declaración de variables
int i = 0;
Stepper myStepper(stepsPerRevolution, 2, 4, 3, 6);

void setup() {
  // Inicialización de la comunicación serial
  Serial.begin(9600);
  myStepper.setSpeed(motSpeed);

  // Inicialización del sensor de color
  if (tcs.begin()) {
    Serial.println("Inicializando...");
    delay(2000);
  }
  else {
    Serial.println("Error...");
    Serial.println("Debe de revisar las conexiones...");
    while (1) delay(500);
  }

  // Configuración inicial de los relés
  digitalWrite(11, RELAY_OFF);
  digitalWrite(10, RELAY_OFF);
  digitalWrite(9, RELAY_OFF);
  digitalWrite(8, RELAY_OFF);

  // Definición de pines como salida
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
  // Variables de control en el espectro RGB
  uint16_t R, G, B, C; // Valores de color: Rojo, Verde, Azul, y Clear (luz ambiente)
  tcs.getRawData(&R, &G, &B, &C);

  // Asegura que los valores de color no superen 255
  R = min(R, 255);
  G = min(G, 255);
  B = min(B, 255);

  // Variables de control para los actuadores
  float red = 0, green = 0;
  red += float(R);
  green += float(G);
  i++;

  if (i > 4) { // Después de acumular 5 lecturas, realiza la comparación
    if (red > green && int(B) <= 70) {
      red /= 5;

      // Activa el motor paso a paso
      myStepper.step(stepsPerRevolution);
      Serial.println("Se detectó el color rojo");
      Serial.println("Rojo = " + String(R) + "\n");

      // Configura los pines PWM y controla los relés
      analogWrite(11, R=255);
      analogWrite(10, G=0);
      analogWrite(9, B=0);
      delay(2000);
      controlRelay(9, RELAY_ON, 1000); // Activa relé 9
      controlRelay(9, RELAY_OFF, 1000); // Desactiva relé 9
      controlRelay(8, RELAY_ON, 1000); // Activa relé 8
      controlRelay(8, RELAY_OFF, 1000); // Desactiva relé 8
    }
    
    if (green > red && int(B) >= 35){
      green /= 5;

      Serial.println("Se detectó el color verde");
      Serial.println("Verde = " + String(G) + "\n");

      // Configura los pines PWM y controla los relés
      analogWrite(11, R=0);
      analogWrite(10, G=255);
      analogWrite(9, B=0);
      delay(2000);
      controlRelay(11, RELAY_ON, 1000); // Activa relé 11
      controlRelay(11, RELAY_OFF, 1000); // Desactiva relé 11
      controlRelay(10, RELAY_ON, 1000); // Activa relé 10
      controlRelay(10, RELAY_OFF, 1000); // Desactiva relé 10
    }
    
    // Reinicia las variables de acumulación y el contador
    red = 0;
    green = 0;
    i = 0;
  }

  // Imprime los valores de color
  Serial.println(" R= " + String(R) + " G= " + String(G) + " B= " + String(B));
  delay(1000);
}

// Función para controlar un relé con retardo
void controlRelay(int pin, int state, int delayTime) {
  digitalWrite(pin, state);
  delay(delayTime);
}
