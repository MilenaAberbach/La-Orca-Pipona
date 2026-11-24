#include <Servo.h>

Servo servoSensor;   // pin 6: rotación continua

const int PARADO    = 90;    // si se mueve solo estando parado, ajustá (88, 92...)
const int VEL_IZQ   = 80;    // velocidad hacia la izquierda (más cerca de 90 = más lento)
const int VEL_DER   = 100;   // velocidad hacia la derecha

// CALIBRAR: ms que tarda en girar 30° a esa velocidad (cada sentido por separado)
const unsigned long MS_30_IZQ = 300;
const unsigned long MS_30_DER = 300;

const unsigned long PAUSA_MEDIDA = 2000;   // ms parado en cada ángulo
const unsigned long PAUSA_CENTRO = 2000;   // ms parado en el centro

// Gira 'pasos' veces 30° (izq o der) y frena
void girar(bool derecha, int pasos) {
  servoSensor.write(derecha ? VEL_DER : VEL_IZQ);
  delay((derecha ? MS_30_DER : MS_30_IZQ) * pasos);
  servoSensor.write(PARADO);
}

// Un lado completo: 30°, 60°, 90°, parándose en cada uno; vuelve al centro
void barrerLado(bool derecha, const char* nombre) {
  for (int g = 30; g <= 90; g += 30) {
    girar(derecha, 1);                       // avanza 30°
    Serial.print(g);
    Serial.print("° ");
    Serial.println(nombre);
    delay(PAUSA_MEDIDA);                     // parado: acá iría la medición
  }
  girar(!derecha, 3);                        // vuelve 90° al centro
  Serial.println("--- centro ---");
  delay(PAUSA_CENTRO);
}

void setup() {
  Serial.begin(9600);
  servoSensor.attach(6);
  servoSensor.write(PARADO);
  delay(1000);
}

void loop() {
  barrerLado(false, "izquierda");
  barrerLado(true,  "derecha");
}
