#include <Servo.h>

Servo servo;

const int trigPin = 2;
const int echoPin = 3;

long duracion;
float distancia;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  servo.attach(6);      // Servo en el pin 6
  servo.write(90);      // Comienza mirando al frente
  delay(1000);
}

void loop() {

  // Disparo del HC-SR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Lectura del eco
  duracion = pulseIn(echoPin, HIGH, 30000);

  if (duracion == 0) {
    Serial.println("Sin objeto");
    return;
  }

  distancia = duracion * 0.0343 / 2;

  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");

  // Si detecta un obstáculo a menos de 20 cm
  if (distancia < 20) {

    Serial.println("¡Obstáculo!");

    // Mira a la izquierda
    servo.write(0);
    delay(700);

    // Mira a la derecha
    servo.write(180);
    delay(700);

    // Vuelve al centro
    servo.write(90);
    delay(700);
  }

  delay(200);
}
