// ============================================================
// ORQUITA - HC-05 por SoftwareSerial (pines A5 y A0)
// El USB queda libre para el monitor serial (9600).
//
// Cableado:
//   HC-05 TXD -> Arduino A5 (RX del SoftwareSerial)
//   HC-05 RXD -> Arduino A0 (TX del SoftwareSerial)
//                (ideal: divisor 1k del pin A0 al RXD y 2k del RXD a GND)
//   HC-05 VCC -> 5 V
//   HC-05 GND -> GND
//
// Nota: A5 es tambien el pin SCL del I2C. Si usas algun modulo
// I2C, elegi otro pin para el RX (por ejemplo A1, A2 o A3).
//
// Resultado:
//   Monitor serial del Arduino IDE:  RECIBIDO: PING / RECIBIDO: CMD,FORWARD
//   Consola de NetBeans:             Arduino> RECIBIDO: CMD,FORWARD
// ============================================================

#include <SoftwareSerial.h>


// ============================== PINES ===============================

const int btRxPin = A5;   // recibe desde TXD del HC-05
const int btTxPin = A1;   // envia hacia RXD del HC-05

const int trigPin  = 2;
const int echoPin  = 3;
const int sensorIR = 4;
const int ledPin   = LED_BUILTIN;

SoftwareSerial bt(btRxPin, btTxPin);


// ============================== AJUSTES =============================

const long BAUDIOS_USB = 9600;
const long BAUDIOS_BT  = 9600;

const unsigned long WATCHDOG_MS        = 3000;
const unsigned long TELEMETRIA_MS      = 200;
const unsigned long INTERVALO_MEDICION = 100;
const unsigned long TIEMPO_ESTABLE_IR  = 40;
const unsigned long LED_RX_MS          = 60;


// ============================== ESTADO ==============================

char bufferBT[80];
byte indiceBT = 0;

unsigned long ultimoContacto = 0;
bool enlaceActivo = false;

unsigned long ledApagarEn = 0;
unsigned long ultimoRx = 0;   // ultimo byte recibido por Bluetooth

unsigned long ultimaTelemetria = 0;
unsigned long secuencia = 0;

float distanciaActual = -1;
unsigned long ultimaMedicion = 0;

int ultimaLecturaIR = HIGH;
int estadoIRConfirmado = HIGH;

unsigned long tiempoCambioIR = 0;
unsigned long numeroCaptura = 0;


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


void procesarOrden(char *orden) {

  // Cualquier linea recibida indica que el enlace esta vivo
  marcarContacto();

  // Monitor serial del Arduino IDE
  Serial.print("RECIBIDO: ");
  Serial.println(orden);

  if (strcmp(orden, "PING") == 0) {

    marcarContacto();

    bt.println("PONG");

    return;
  }

  if (strncmp(orden, "CMD,", 4) == 0) {

    marcarContacto();

    char *accion = orden + 4;

    // Esto aparece en NetBeans como "Arduino> RECIBIDO: CMD,..."
    bt.print("RECIBIDO: ");
    bt.println(orden);

    bt.print("ACK,");
    bt.println(accion);

    Serial.print("ACK,");
    Serial.println(accion);

    return;
  }

  bt.print("ERR,");
  bt.println(orden);

  Serial.print("ERR,");
  Serial.println(orden);
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
  // llegando algo, esperamos para no corromper los bytes.
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
  enviarTelemetria();
  controlarEnlace();
  actualizarLed();
}
