#include <Servo.h>

Servo puerta;

void setup() {
  puerta.attach(6);
}

void loop() {
  puerta.write(90);   // detener
}
