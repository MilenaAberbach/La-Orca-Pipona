#include <Servo.h>

Servo servoSensor;   // pin 6: de ángulo en ángulo
Servo servoPuerta;   // pin 5: de corrido

const int TRIG = 9;
const int ECHO = 10;
const int DIST_BULTO = 30;        // cm: menos que esto = hay bulto
const int CENTRO = 90;
const int PAUSA_MEDIDA = 1000;    // ms quieto en cada ángulo
const int PAUSA_CENTRO = 2000;    // ms quieto en el medio entre un lado y otro

long medirDistancia() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duracion = pulseIn(ECHO, HIGH, 30000);
  if (duracion == 0) return 999;  // sin eco = libre
  return duracion * 0.034 / 2;
}

// gira a un ángulo, se queda quieto y mide
void medirEn(int anguloServo, int gradosDesdeCentro, const char* lado) {
  servoSensor.write(anguloServo);
  delay(PAUSA_MEDIDA);

  long d = medirDistancia();

  Serial.print(gradosDesdeCentro);
  Serial.print("° ");
  Serial.print(lado);
  if (d < DIST_BULTO) {
    Serial.print(" -> HAY BULTO a ");
    Serial.print(d);
    Serial.println(" cm");
  } else {
    Serial.println(" -> libre");
  }
}

// vuelve al medio y se queda parado
void pararEnCentro() {
  servoSensor.write(CENTRO);
  Serial.println("--- centro ---");
  delay(PAUSA_CENTRO);
}

void ladoIzquierdo() {
  medirEn(CENTRO - 30, 30, "izquierda");   // servo 60
  medirEn(CENTRO - 60, 60, "izquierda");   // servo 30
  medirEn(CENTRO - 90, 90, "izquierda");   // servo 0
}

void ladoDerecho() {
  medirEn(CENTRO + 30, 30, "derecha");     // servo 120
  medirEn(CENTRO + 60, 60, "derecha");     // servo 150
  medirEn(CENTRO + 90, 90, "derecha");     // servo 180
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  servoSensor.attach(6);
  servoPuerta.attach(5);

  servoSensor.write(CENTRO);
  delay(1000);
  servoPuerta.write(10);               // puerta de corrido, no se toca más
}

void loop() {
  ladoIzquierdo();
  pararEnCentro();      // termina izquierda -> se para en el medio

  ladoDerecho();
  pararEnCentro();      // termina derecha -> se para en el medio
}
