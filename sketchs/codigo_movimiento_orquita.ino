// ============================================================
// ORQUITA - Arduino (v5)
//  Cambios v4 (marcados "[v4]"):
//   - Watchdog 10 s, TEL apagada, tiempos de barrido reducidos
//   - Cola de respuestas BT (solo se transmite con el servo quieto)
//   - El barrido se pausa si los motores se mueven
//   - Pasadas con el robot en movimiento se descartan
//  Cambios v5 (marcados "[v5]"):
//   - CAP (captura) tambien sale por la cola (no hace temblar el servo)
//   - La cola, si se llena, descarta el mensaje mas viejo (CAP no se pierde)
//   - El SWP se copia (snapshot) al terminar la pasada: no se mezcla con
//     la medicion siguiente si el envio se demora
//   - AVANZAR/RETROCEDER avanzan una rafaga corta (MS_AVANCE) en vez de
//     2,5 s a ciegas: el robot mira, decide, avanza poco, vuelve a mirar
//
//  - Bluetooth HC-05 por SoftwareSerial (A5 / A1)
//  - Recibe ordenes del servidor: ORD|accion=AVANZAR
//  - Motores con puente H (L298N)
//  - Servo del sensor (pin 6): rotacion continua, gira por tiempo.
//    Barre en 7 posiciones (izq 90 ... der 90) en ida y vuelta,
//    midiendo en cada una, y al terminar cada pasada envia UNA linea:
//        SWP,<n>,<d0>,<d1>,<d2>,<d3>,<d4>,<d5>,<d6>,<ir>
//    d0 = extremo izquierdo, d3 = frente, d6 = extremo derecho.
//    Una distancia de -1 significa "sin eco" (no es lo mismo que libre).
//  - Servo de la puerta (pin 5): rotacion continua, gira 20 s
//    solo cuando el IR detecta una canica
// ============================================================

#include <SoftwareSerial.h>
#include <Servo.h>


// ============================== PINES ===============================

const int btRxPin = A5;   // recibe desde TXD del HC-05
const int btTxPin = A1;   // envia hacia RXD del HC-05

const int trigPin  = 2;
const int echoPin  = 3;
const int sensorIR = 4;

const int servoPuertaPin = 5;
const int servoSensorPin = 6;

// Motor izquierdo
const int ENA = 8;
const int IN1 = 9;
const int IN2 = 10;
// Motor derecho
const int IN3 = 11;
const int IN4 = 12;
const int ENB = 13;

SoftwareSerial bt(btRxPin, btTxPin);
Servo servoSensor;
Servo servoPuerta;


// ============================== AJUSTES =============================

const long BAUDIOS_USB = 9600;
const long BAUDIOS_BT  = 9600;

const unsigned long WATCHDOG_MS       = 10000;   // [v4]
const unsigned long TELEMETRIA_MS     = 200;
const unsigned long TIEMPO_ESTABLE_IR = 40;

const bool ENVIAR_TEL_PERIODICA = false;         // [v4]

// ---- Motores ----
const unsigned long TIMEOUT_ORDEN_MS = 2500;  // limite de seguridad
const unsigned long MS_GIRO          = 400;   // CALIBRAR: giro corto
const unsigned long MS_AVANCE        = 600;   // [v5] CALIBRAR: rafaga de avance/retroceso

// ---- Servo de la puerta (rotacion continua) ----
const int VALOR_PUERTA = 10;                  // velocidad/sentido cuando gira (170 = al reves)
const unsigned long PUERTA_MS = 20000;        // cuanto gira tras detectar una canica

// ---- Servo del sensor (rotacion continua, gira por TIEMPO) ----
const int PARADO  = 90;          // si deriva estando parado, ajustar (88, 92...)
const int VEL_IZQ = 80;
const int VEL_DER = 100;
const unsigned long MS_30_IZQ = 300;   // CALIBRAR: ms para girar 30 grados
const unsigned long MS_30_DER = 300;   // CALIBRAR

// true  = al frenar se corta la senal del servo (no deriva ni tiembla).
const bool SOLTAR_SERVO_QUIETO = true;

// ---- Barrido ----
const byte CANT_POS   = 7;     // izq 90, 60, 30, centro, der 30, 60, 90
const byte POS_CENTRO = 3;
const unsigned long SERVO_ASENTAR_MS = 150;  // [v4] espera antes de medir
const unsigned long PAUSA_MEDIDA     = 300;  // [v4] tiempo total quieto en cada posicion
const byte REINTENTOS_MEDIDA = 2;            // si no hay eco, reintenta

// ---- Cola de respuestas Bluetooth ----
const byte COLA_MAX   = 6;     // [v5] antes 4
const byte COLA_LARGO = 48;

const float DIST_FRENO = 20.0;   // cm: si el frente esta mas cerca, frena

// ============================== ESTADO ==============================

// Bluetooth
char bufferBT[80];
byte indiceBT = 0;
unsigned long ultimoContacto = 0;
unsigned long ultimoRx = 0;
bool enlaceActivo = false;

// Cola de respuestas pendientes
char colaBT[COLA_MAX][COLA_LARGO];
byte colaN = 0;

// Telemetria
unsigned long ultimaTelemetria = 0;
unsigned long secuencia = 0;
float distanciaActual = -1;

// IR
int ultimaLecturaIR = HIGH;
int estadoIRConfirmado = HIGH;
unsigned long tiempoCambioIR = 0;
unsigned long numeroCaptura = 0;

// Motores
unsigned long motorApagarEn = 0;

// Puerta
unsigned long puertaApagarEn = 0;

// Barrido del servo del sensor
float distancias[CANT_POS];        // ultima medicion de cada posicion
float distanciasEnvio[CANT_POS];   // [v5] copia congelada de la pasada lista para enviar
byte posActual = POS_CENTRO;       // el sensor debe arrancar mirando al frente
int8_t sentido = 1;                // +1 hacia la derecha, -1 hacia la izquierda
bool girando = false;
bool medido = false;
bool barridoListo = false;         // hay una pasada completa sin enviar
bool pasadaContaminada = false;    // [v4] true si los motores se movieron durante la pasada
unsigned long secuenciaBarrido = 0;
unsigned long finGiro = 0;
unsigned long inicioQuieto = 0;
byte pasoActual = 0;

int posicionGrados = 0;          // 0 = centro, negativo = izq, positivo = der
bool avanzando = false;
unsigned long ultimaMedicionContinua = 0;

// ============================== MOTORES =============================

void motorIzquierdo(int a, int b) {
  digitalWrite(IN1, a);
  digitalWrite(IN2, b);
}

void motorDerecho(int a, int b) {
  digitalWrite(IN3, a);
  digitalWrite(IN4, b);
}

void detenerMotores() {
  motorIzquierdo(LOW, LOW);
  motorDerecho(LOW, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  motorApagarEn = 0;
  avanzando = false;
}

void moverMotores(int izqA, int izqB, int derA, int derB, unsigned long duracion) {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  motorIzquierdo(izqA, izqB);
  motorDerecho(derA, derB);
  avanzando = false;
  if (duracion == 0) duracion = TIMEOUT_ORDEN_MS;
  motorApagarEn = millis() + duracion;
}

bool frenteBloqueado() {
  bool mirandoAdelante = !girando && abs(posicionGrados) <= 30;
  return mirandoAdelante && distanciaActual > 0 && distanciaActual < DIST_FRENO;
}

void avanzar() {
  if (frenteBloqueado()) {
    detenerMotores();
    Serial.println("FRENO: obstaculo adelante");
    return;
  }
  moverMotores(HIGH, LOW, HIGH, LOW, 0);
  avanzando = true;
}

void retroceder() { moverMotores(LOW, HIGH, LOW, HIGH, MS_AVANCE); }
void izquierda()  { moverMotores(LOW, HIGH, HIGH, LOW, MS_GIRO); }
void derecha()    { moverMotores(HIGH, LOW, LOW, HIGH, MS_GIRO); }

void actualizarMotores() {
  if (motorApagarEn != 0 && millis() >= motorApagarEn) {
    detenerMotores();
  }
}

void frenoEmergencia() {
  if (avanzando && frenteBloqueado()) {
    detenerMotores();
    Serial.println("FRENO DE EMERGENCIA");
  }
}

// ============================== PUERTA ==============================

void abrirPuerta() {
  if (!servoPuerta.attached()) {
    servoPuerta.attach(servoPuertaPin);
  }
  servoPuerta.write(VALOR_PUERTA);
  puertaApagarEn = millis() + PUERTA_MS;   // si llega otra canica, renueva los 20 s
}

void actualizarPuerta() {
  if (puertaApagarEn != 0 && millis() >= puertaApagarEn) {
    servoPuerta.detach();                  // se suelta: queda parado y sin interferir
    puertaApagarEn = 0;
    Serial.println("PUERTA: parada");
  }
}


// ============================== BLUETOOTH ===========================

// SoftwareSerial no puede recibir mientras transmite, y bloquea las
// interrupciones (hace temblar al servo): se transmite solo cuando el
// servo del sensor esta quieto y no esta llegando nada.
bool puedeTransmitir() {
  if (girando) return false;
  if (bt.available() || millis() - ultimoRx < 30) return false;
  return true;
}

// Guarda un mensaje para enviarlo cuando sea seguro.
// [v5] Si la cola esta llena, descarta el mas viejo para que el nuevo entre.
void encolarBT(const char *a, const char *b) {
  if (colaN >= COLA_MAX) {
    for (byte i = 1; i < colaN; i++) {
      strcpy(colaBT[i - 1], colaBT[i]);
    }
    colaN--;
  }
  snprintf(colaBT[colaN], COLA_LARGO, "%s%s", a, b);
  colaN++;
}

// Envia de a un mensaje por vuelta de loop(), solo si es seguro
void vaciarColaBT() {
  if (colaN == 0) return;
  if (!puedeTransmitir()) return;

  bt.println(colaBT[0]);

  for (byte i = 1; i < colaN; i++) {
    strcpy(colaBT[i - 1], colaBT[i]);
  }
  colaN--;
}

void marcarContacto() {

  ultimoContacto = millis();

  if (!enlaceActivo) {
    enlaceActivo = true;
    encolarBT("LINK_OK", "");
    Serial.println("LINK_OK");
  }
}

void recibirBluetooth() {

  while (bt.available()) {

    char c = bt.read();
    ultimoRx = millis();

    if (c == '\n') {

      bufferBT[indiceBT] = '\0';
      if (indiceBT > 0) {
        procesarOrden(bufferBT);
      }
      indiceBT = 0;

    } else if (c != '\r') {

      if (indiceBT < sizeof(bufferBT) - 1) {
        bufferBT[indiceBT++] = c;
      } else {
        indiceBT = 0;
      }
    }
  }
}

void enviarTelemetria() {

  if (!ENVIAR_TEL_PERIODICA) return;
  if (millis() - ultimaTelemetria < TELEMETRIA_MS) return;
  if (!puedeTransmitir()) return;

  ultimaTelemetria = millis();

  bt.print("TEL,");
  bt.print(secuencia++);
  bt.print(",");
  bt.print(distanciaActual);
  bt.print(",");
  bt.println(digitalRead(sensorIR));
}

// Una linea por pasada completa del sensor: las 7 distancias + IR
void enviarBarrido() {

  if (!barridoListo) return;
  if (!puedeTransmitir()) return;

  barridoListo = false;

  bt.print("SWP,");
  bt.print(secuenciaBarrido);
  Serial.print("SWP ");
  Serial.print(secuenciaBarrido);

  for (byte i = 0; i < CANT_POS; i++) {
    bt.print(",");
    bt.print(distanciasEnvio[i]);          // [v5] usa la copia congelada
    Serial.print(" ");
    Serial.print(distanciasEnvio[i]);
  }

  bt.print(",");
  bt.println(digitalRead(sensorIR));
  Serial.println();

  secuenciaBarrido++;
}

void controlarEnlace() {

  if (enlaceActivo && millis() - ultimoContacto >= WATCHDOG_MS) {

    enlaceActivo = false;
    detenerMotores();            // sin enlace, el robot frena

    encolarBT("LINK_LOST", "");
    Serial.println("LINK_LOST");
  }
}


// ============================== ORDENES =============================
// Formato: ORD|accion=AVANZAR

bool ejecutarAccion(const char *accion) {

  if (strcmp(accion, "AVANZAR") == 0)    { avanzar();        return true; }
  if (strcmp(accion, "RETROCEDER") == 0) { retroceder();     return true; }
  if (strcmp(accion, "IZQUIERDA") == 0)  { izquierda();      return true; }
  if (strcmp(accion, "DERECHA") == 0)    { derecha();        return true; }
  if (strcmp(accion, "DETENER") == 0)    { detenerMotores(); return true; }
  if (strcmp(accion, "CAPTURAR") == 0)   { detenerMotores(); return true; }

  return false;
}

void procesarOrden(char *orden) {

  marcarContacto();

  Serial.print("RECIBIDO: ");
  Serial.println(orden);

  if (strcmp(orden, "PING") == 0) {
    encolarBT("PONG", "");
    return;
  }

  // strstr tolera bytes daniados delante de la orden
  char *posicion = strstr(orden, "ORD|accion=");
  if (posicion != NULL) {

    const char *accion = posicion + 11;

    if (ejecutarAccion(accion)) {
      encolarBT("ACK,", accion);
      Serial.print("ACK,");
      Serial.println(accion);
    } else {
      encolarBT("ERR,", orden);
      Serial.print("ERR,");
      Serial.println(orden);
    }
    return;
  }

  encolarBT("ERR,", orden);
  Serial.print("ERR,");
  Serial.println(orden);
}


// ============================== ULTRASONIDO =========================
float medirUna() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long d = pulseIn(echoPin, HIGH, 18000);
  if (d == 0) return -1;
  return d * 0.0343 / 2.0;
}

float medirDistancia() {
  float v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = medirUna();
    delay(10);
  }
  // ordenar y devolver la mediana
  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[1];
}

// Si no vuelve eco, reintenta antes de dar la lectura por perdida
float medirConReintentos() {

  float d = -1;

  for (byte i = 0; i <= REINTENTOS_MEDIDA; i++) {
    d = medirDistancia();
    if (d > 0) break;
    delay(30);     // deja que se apague el eco anterior
  }

  return d;
}


// ============================== BARRIDO DEL SENSOR ==================
// Servo continuo: gira 30 grados (por tiempo), frena, espera, mide y
// pasa a la posicion vecina. Va de la izquierda a la derecha y vuelve,
// midiendo en ambos sentidos. Sin delay() en el movimiento.
// El servo solo esta conectado (attach) mientras gira.

void girarServoSensor(bool derecha) {
  if (!servoSensor.attached()) {
    servoSensor.attach(servoSensorPin);
  }
  servoSensor.write(derecha ? VEL_DER : VEL_IZQ);
}

void frenarServoSensor() {
  if (servoSensor.attached()) {
    servoSensor.write(PARADO);
    if (SOLTAR_SERVO_QUIETO) {
      servoSensor.detach();      // sin señal: no deriva ni tiembla
    }
  }
}

void iniciarGiro(bool derecha, unsigned long duracionMs) {
  girarServoSensor(derecha);
  girando = true;
  medido = false;
  finGiro = millis() + duracionMs;
}

// Avanza a la posicion siguiente.
// Cuando llega a un extremo, invierte el sentido.
void avanzarPosicion() {

  // Si estamos en el extremo derecho, volver hacia la izquierda
  if (posActual >= CANT_POS - 1) {
    sentido = -1;
  }

  // Si estamos en el extremo izquierdo, volver hacia la derecha
  else if (posActual == 0) {
    sentido = 1;
  }

  // Actualizar la posicion
  posActual += sentido;

  // Iniciar movimiento del servo
  iniciarGiro(
    sentido > 0,
    sentido > 0 ? MS_30_DER : MS_30_IZQ
  );

}

void controlarBarrido() {
  unsigned long ahora = millis();

  // 1) Si está girando, esperar a que termine el movimiento
  if (girando) {
    if (ahora >= finGiro) {
      frenarServoSensor();
      girando = false;
      inicioQuieto = ahora;
      medido = false;
    }
    return;
  }

  // 2) Si ya se midió esta posición, esperar antes de avanzar
  if (medido) {
    if (ahora - inicioQuieto >= PAUSA_MEDIDA) {
      avanzarPosicion();
    }
    return;
  }

  // 3) Esperar a que el servo se estabilice
  if (ahora - inicioQuieto < SERVO_ASENTAR_MS) {
    return;
  }

  // 4) Medir la posición actual
  float distancia = medirConReintentos();

  distancias[posActual] = distancia;
  distanciaActual = distancia;
  medido = true;
  ultimaMedicionContinua = ahora;

  Serial.print("POSICION: ");
  Serial.print(posActual);
  Serial.print(" | DISTANCIA: ");
  Serial.println(distancia);

  // Si está mirando al frente, guardar la distancia frontal
  if (posActual == POS_CENTRO) {
    distanciaActual = distancia;
  }

  // 5) ENVIAR SWP SOLAMENTE AL COMPLETAR
  //    la pasada de izquierda a derecha
  if (posActual == CANT_POS - 1 && sentido == 1) {

    Serial.println("BARRIDO COMPLETO: IZQUIERDA A DERECHA");

    // Copiar las 7 mediciones para congelar los datos
    for (byte i = 0; i < CANT_POS; i++) {
      distanciasEnvio[i] = distancias[i];
    }

    barridoListo = true;
  }

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
      abrirPuerta();                       // puerta gira PUERTA_MS

      // [v5] CAP por la cola (antes bt.print directo)
      char num[12];
      snprintf(num, sizeof(num), "%lu", numeroCaptura);
      encolarBT("CAP,", num);

      Serial.print("CANICA DETECTADA ");
      Serial.println(numeroCaptura);
    }
  }
}


// ============================== SETUP ===============================

void configurarPines() {

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(sensorIR, INPUT_PULLUP);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  detenerMotores();
}

void setup() {

  Serial.begin(BAUDIOS_USB);
  bt.begin(BAUDIOS_BT);

  configurarPines();

  ultimaLecturaIR = digitalRead(sensorIR);
  estadoIRConfirmado = ultimaLecturaIR;
  tiempoCambioIR = millis();

  for (byte i = 0; i < CANT_POS; i++) {
    distancias[i] = -1;
    distanciasEnvio[i] = -1;
  }

  // Los servos se conectan solo cuando hace falta:
  // el del sensor en iniciarGiro(), la puerta en abrirPuerta().
  Serial.println("================================");
  Serial.println("ARDUINO INICIANDO");
  Serial.println("SETUP EJECUTADO");
  Serial.println("================================");
  
  bt.println("READY");

  // Arranca mirando al frente: va primero al extremo izquierdo y desde
  // ahi empieza la primera pasada hacia la derecha.
  posActual = 0;
  sentido = 1;
  iniciarGiro(false, MS_30_IZQ * POS_CENTRO);
}

// ============================== LOOP ================================

void loop() {
  static unsigned long contadorLoop = 0;

  if (millis() - contadorLoop >= 1000) {
    contadorLoop = millis();
    Serial.println("LOOP FUNCIONANDO");
  }

  recibirBluetooth();
  controlarBarrido();
  frenoEmergencia();  
  detectarIR();
  enviarBarrido();
  vaciarColaBT();
  enviarTelemetria();
  actualizarMotores();
  actualizarPuerta();
  controlarEnlace();
}
