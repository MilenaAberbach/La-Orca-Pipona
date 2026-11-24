// ============================================================
// ORQUITA - Version autonoma (bateria, sin USB)
//
// HC-05 por SoftwareSerial en A5 (RX) y A0 (TX).
// Servo del ultrasonido + 2 motores con driver L298N.
//
// Cableado (SUPUESTO: L298N. Si usas otro driver o pines,
// cambia solo la seccion PINES):
//
//   HC-05 TXD -> A5            HC-05 RXD -> A0 (divisor 1k/2k)
//   HC-05 VCC -> 5 V           HC-05 GND -> GND
//
//   Ultrasonido  TRIG -> 2     ECHO -> 3
//   Sensor IR (canica)  -> 4
//   Servo (senal)       -> 9
//
//   L298N  ENA -> 5 (PWM)   IN1 -> 7    IN2 -> 8    (motor izquierdo)
//   L298N  ENB -> 6 (PWM)   IN3 -> 11   IN4 -> 12   (motor derecho)
//
//   TODAS las masas unidas: bateria, L298N, Arduino, servo, HC-05.
//   Los motores y el servo NO se alimentan del pin 5V del Arduino.
//
// Comandos que entiende (llegan por Bluetooth, una linea cada uno):
//   PING
//   CMD,SERVO,<0-180>
//   CMD,AVANZAR / RETROCEDER / IZQUIERDA / DERECHA / DETENER
//   CMD,CAPTURAR   (solo confirma; el servo lo mueve el Cliente)
//
// Seguridad:
//   - Cada movimiento dura un tiempo corto y se detiene solo.
//   - Si se pierde el enlace, los motores se detienen.
//   - Avanzando, frena si el ultrasonido ve algo muy cerca.
// ============================================================

#include <SoftwareSerial.h>
#include <Servo.h>


// ============================== PINES ===============================

const int btRxPin = A5;   // recibe desde TXD del HC-05
const int btTxPin = A1;   // envia hacia RXD del HC-05

const int trigPin  = 2;
const int echoPin  = 3;
const int sensorIR = 4;
const int ledPin   = LED_BUILTIN;

const int servoPin = 6;

const int enaPin = 13;     // motor izquierdo (PWM)
const int in1Pin = 12;
const int in2Pin = 11;

const int enbPin = 8;     // motor derecho (PWM)
const int in3Pin = 9;
const int in4Pin = 10;

SoftwareSerial bt(btRxPin, btTxPin);
Servo servo;


// ============================== AJUSTES =============================

const long BAUDIOS_USB = 9600;
const long BAUDIOS_BT  = 9600;

const unsigned long WATCHDOG_MS        = 3000;
const unsigned long TELEMETRIA_MS      = 200;
const unsigned long INTERVALO_MEDICION = 100;
const unsigned long TIEMPO_ESTABLE_IR  = 40;
const unsigned long LED_RX_MS          = 60;

// Motores (velocidad 0-255 y duracion de cada movimiento)
const int VEL_AVANCE = 200;
const int VEL_GIRO   = 200;
const unsigned long MOV_AVANCE_MS = 600;
const unsigned long MOV_GIRO_MS   = 400;

// Frena si avanza y ve un obstaculo mas cerca que esto
const float DISTANCIA_FRENO_CM = 8.0;

// Tiempo que el servo se queda activo para llegar al angulo.
// Despues se desconecta: asi no interfiere con el Bluetooth.
const unsigned long SERVO_ASENTAR_MS = 350;


// ============================== ESTADO ==============================

char bufferBT[80];
byte indiceBT = 0;

unsigned long ultimoContacto = 0;
bool enlaceActivo = false;

unsigned long ledApagarEn = 0;
unsigned long ultimoRx = 0;

unsigned long ultimaTelemetria = 0;
unsigned long secuencia = 0;

float distanciaActual = -1;
unsigned long ultimaMedicion = 0;

int ultimaLecturaIR = HIGH;
int estadoIRConfirmado = HIGH;

unsigned long tiempoCambioIR = 0;
unsigned long numeroCaptura = 0;

bool moviendo = false;
bool avanzando = false;
unsigned long movimientoInicio = 0;
unsigned long movimientoDuracion = 0;

unsigned long servoDesconectarEn = 0;


// ============================== LED RX ==============================

void indicarRecepcion() {
  ultimoRx = millis();
  digitalWrite(ledPin, HIGH);
  ledApagarEn = millis() + LED_RX_MS;
}

void actualizarLed() {
  if (ledApagarEn != 0 && millis() >= ledApagarEn) {
    digitalWrite(ledPin, LOW);
    ledApagarEn = 0;
  }
}


// ============================== SERVO ===============================

void moverServo(int angulo) {

  angulo = constrain(angulo, 0, 180);

  if (!servo.attached()) {
    servo.attach(servoPin);
  }

  servo.write(angulo);

  servoDesconectarEn = millis() + SERVO_ASENTAR_MS;
}

void actualizarServo() {

  if (servoDesconectarEn != 0 && millis() >= servoDesconectarEn) {

    servo.detach();
    servoDesconectarEn = 0;
  }
}


// ============================== MOTORES =============================

// dir: 1 = adelante, -1 = atras, 0 = parado
void motor(int en, int a, int b, int dir, int vel) {

  if (dir > 0) {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
    analogWrite(en, vel);
  }
  else if (dir < 0) {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
    analogWrite(en, vel);
  }
  else {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
    analogWrite(en, 0);
  }
}

void moverRobot(int izq, int der, int vel, unsigned long duracion) {

  motor(enaPin, in1Pin, in2Pin, izq, vel);
  motor(enbPin, in3Pin, in4Pin, der, vel);

  moviendo  = (izq != 0 || der != 0);
  avanzando = (izq > 0 && der > 0);

  movimientoInicio   = millis();
  movimientoDuracion = duracion;
}

void detenerMotores() {
  moverRobot(0, 0, 0, 0);
}

void actualizarMovimiento() {

  if (!moviendo) {
    return;
  }

  unsigned long ahora = millis();

  // Termino el tiempo del movimiento
  if (ahora - movimientoInicio >= movimientoDuracion) {
    detenerMotores();
    return;
  }

  // Freno de emergencia al avanzar
  if (avanzando
      && ahora - movimientoInicio >= 150
      && distanciaActual > 0
      && distanciaActual < DISTANCIA_FRENO_CM) {

    detenerMotores();

    Serial.println("FRENO_OBSTACULO");
    bt.println("BRAKE");
  }
}


// ============================== ULTRASONIDO =========================

float medirDistancia() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duracion = pulseIn(echoPin, HIGH, 18000);

  if (duracion == 0) {
    return -1;
  }

  return duracion * 0.0343 / 2.0;
}


void controlarUltrasonico() {

  if (millis() - ultimaMedicion < INTERVALO_MEDICION) {
    return;
  }

  ultimaMedicion = millis();
  distanciaActual = medirDistancia();
}


// ============================== SENSOR IR ===========================

void detectarIR() {

  int lectura = digitalRead(sensorIR);

  if (lectura != ultimaLecturaIR) {
    tiempoCambioIR = millis();
    ultimaLecturaIR = lectura;
  }

  if (millis() - tiempoCambioIR >= TIEMPO_ESTABLE_IR
      && lectura != estadoIRConfirmado) {

    estadoIRConfirmado = lectura;

    if (estadoIRConfirmado == LOW) {

      numeroCaptura++;

      bt.print("CAP,");
      bt.println(numeroCaptura);

      Serial.print("CANICA DETECTADA ");
      Serial.println(numeroCaptura);
    }
  }
}


// ============================== BLUETOOTH ===========================

void marcarContacto() {

  ultimoContacto = millis();

  if (!enlaceActivo) {

    enlaceActivo = true;

    bt.println("LINK_OK");
    Serial.println("LINK_OK");
  }
}


bool ejecutarAccion(char *accion) {

  if (strncmp(accion, "SERVO,", 6) == 0) {
    moverServo(atoi(accion + 6));
    return true;
  }

  if (strcmp(accion, "AVANZAR") == 0) {
    moverRobot(1, 1, VEL_AVANCE, MOV_AVANCE_MS);
    return true;
  }

  if (strcmp(accion, "RETROCEDER") == 0) {
    moverRobot(-1, -1, VEL_AVANCE, MOV_AVANCE_MS);
    return true;
  }

  if (strcmp(accion, "IZQUIERDA") == 0) {
    moverRobot(-1, 1, VEL_GIRO, MOV_GIRO_MS);
    return true;
  }

  if (strcmp(accion, "DERECHA") == 0) {
    moverRobot(1, -1, VEL_GIRO, MOV_GIRO_MS);
    return true;
  }

  if (strcmp(accion, "DETENER") == 0) {
    detenerMotores();
    return true;
  }

  if (strcmp(accion, "CAPTURAR") == 0) {
    return true;
  }

  return false;
}


void procesarOrden(char *orden) {

  // Cualquier linea recibida indica que el enlace esta vivo
  marcarContacto();

  Serial.print("RECIBIDO: ");
  Serial.println(orden);

  // Se busca "CMD," y "PING" dentro de la linea para tolerar
  // bytes basura que SoftwareSerial a veces mete al principio.
  char *cmd = strstr(orden, "CMD,");

  if (cmd != NULL) {

    char *accion = cmd + 4;

    if (ejecutarAccion(accion)) {

      bt.print("ACK,");
      bt.println(accion);

      Serial.print("ACK,");
      Serial.println(accion);
    }
    else {

      bt.print("ERR,");
      bt.println(accion);

      Serial.print("ERR,");
      Serial.println(accion);
    }

    return;
  }

  if (strstr(orden, "PING") != NULL) {

    bt.println("PONG");
    return;
  }

  bt.print("ERR,");
  bt.println(orden);
}


void recibirBluetooth() {

  while (bt.available()) {

    char c = bt.read();

    indicarRecepcion();

    if (c == '\n') {

      bufferBT[indiceBT] = '\0';

      if (indiceBT > 0) {
        procesarOrden(bufferBT);
      }

      indiceBT = 0;
    }

    else if (c != '\r') {

      if (indiceBT < sizeof(bufferBT) - 1) {
        bufferBT[indiceBT++] = c;
      }
      else {
        indiceBT = 0;
      }
    }
  }
}


// ============================== TELEMETRIA ==========================

void enviarTelemetria() {

  if (millis() - ultimaTelemetria < TELEMETRIA_MS) {
    return;
  }

  // SoftwareSerial no puede recibir mientras transmite: si esta
  // llegando algo, se espera para no corromper los bytes.
  if (bt.available() || millis() - ultimoRx < 30) {
    return;
  }

  ultimaTelemetria = millis();

  bt.print("TEL,");
  bt.print(secuencia++);
  bt.print(",");
  bt.print(distanciaActual);
  bt.print(",");
  bt.println(digitalRead(sensorIR));
}


// ============================== WATCHDOG ============================

void controlarEnlace() {

  if (enlaceActivo
      && millis() - ultimoContacto >= WATCHDOG_MS) {

    enlaceActivo = false;

    // Sin enlace el robot no se mueve
    detenerMotores();

    bt.println("LINK_LOST");
    Serial.println("LINK_LOST");
  }
}


// ============================== SETUP ===============================

void setup() {

  Serial.begin(BAUDIOS_USB);
  bt.begin(BAUDIOS_BT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(sensorIR, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enbPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  detenerMotores();

  ultimaLecturaIR = digitalRead(sensorIR);
  estadoIRConfirmado = ultimaLecturaIR;
  tiempoCambioIR = millis();

  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
  }

  Serial.println("READY (monitor USB)");
  bt.println("READY");
}


// ============================== LOOP ================================

void loop() {

  recibirBluetooth();
  controlarUltrasonico();
  detectarIR();
  actualizarMovimiento();
  actualizarServo();
  enviarTelemetria();
  controlarEnlace();
  actualizarLed();
}
