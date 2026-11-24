// ============================================================
// ORQUITA - Arduino (v6) = tu sketch actual + estas correcciones:
//  [v6] frenteBloqueado(): usaba posicionGrados, que nunca se actualizaba,
//       asi que cualquier lectura (incluso de costado) bloqueaba AVANZAR.
//       Ahora usa la lectura del frente (posicion central del barrido).
//  [v6] DETENER: ya no pone posActual = centro sin mover el servo
//       (eso desplazaba todo el barrido y el sensor terminaba mirando
//       el chasis: lecturas de 5-9 cm).
//  [v6] Modo captura: el sensor se recentra al frente (por tiempo) antes
//       de perseguir. Antes quedaba congelado mirando a la derecha.
//       Si estaba girando, primero termina ese giro y despues recentra.
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
const unsigned long PUERTA_MS = 20000;        // cuanto gira desde que llega la orden de captura

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
const byte COLA_MAX   = 4;     // [v5] antes 4
const byte COLA_LARGO = 48;

const float DIST_FRENO = 20.0;   // cm: si el frente esta mas cerca, frena
const float DIST_DETECCION_CANICA = 80.0; // cm: rango maximo para iniciar persecucion
const unsigned long MS_PERSECUCION = 700; // avance por rafaga durante la persecucion
const unsigned long MS_GIRO_PERSECUCION = 220; // giro inicial; calibrar segun el chasis
const unsigned long CAPTURA_MAX_MS = 12000; // evita perseguir indefinidamente

// ============================== ESTADO ==============================

// Bluetooth
// 40 bytes alcanzan para las ordenes direccionales del protocolo.
char bufferBT[40];
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
unsigned long secuenciaBarrido = 0;
unsigned long finGiro = 0;
unsigned long inicioQuieto = 0;

bool avanzando = false;
unsigned long ultimaMedicionContinua = 0;

// Modos autonomos
enum ModoRobot { EXPLORANDO, CAPTURANDO };
ModoRobot modoRobot = EXPLORANDO;
unsigned long inicioCaptura = 0;
unsigned long proximaAccionCaptura = 0;
bool giroCapturaPendiente = false;
bool esperandoAvanceCaptura = false;
bool recentrarPendiente = false;   // [v6] terminar el giro en curso y despues recentrar
byte fallosEcoCaptura = 0;


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
  avanzando = (izqA == HIGH && izqB == LOW && derA == HIGH && derB == LOW);
  if (duracion == 0) duracion = TIMEOUT_ORDEN_MS;
  motorApagarEn = millis() + duracion;
}

// [v6] Antes usaba posicionGrados (nunca se actualizaba) y la ultima lectura
// de CUALQUIER posicion. Ahora mira solo el frente.
bool frenteBloqueado() {
  float d = distancias[POS_CENTRO];

  // Si el sensor esta mirando al frente (centro +/- 1 posicion) y acaba de
  // medir, esa lectura es la mas fresca.
  if (!girando && medido
      && posActual + 1 >= POS_CENTRO
      && posActual <= POS_CENTRO + 1) {
    d = distanciaActual;
  }

  return d > 0 && d < DIST_FRENO;
}

void avanzar() {
  if (frenteBloqueado()) {
    detenerMotores();
    Serial.println(F("FRENO: obstaculo adelante"));
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
  if (modoRobot == CAPTURANDO) return;
  if (avanzando && frenteBloqueado()) {
    detenerMotores();
    Serial.println(F("FRENO DE EMERGENCIA"));
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
    Serial.println(F("PUERTA: parada"));
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
    Serial.println(F("LINK_OK"));
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

  bt.print(F("TEL,"));
  bt.print(secuencia++);
  bt.print(F(","));
  bt.print(distanciaActual);
  bt.print(F(","));
  bt.println(digitalRead(sensorIR));
}

// Una linea por pasada completa del sensor: las 7 distancias + IR
void enviarBarrido() {

  if (modoRobot == CAPTURANDO) return;
  if (!barridoListo) return;
  if (!puedeTransmitir()) return;

  barridoListo = false;

  bt.print(F("SWP,"));
  bt.print(secuenciaBarrido);
  Serial.print(F("SWP "));
  Serial.print(secuenciaBarrido);

  for (byte i = 0; i < CANT_POS; i++) {
    bt.print(F(","));
    bt.print(distanciasEnvio[i]);          // [v5] usa la copia congelada
    Serial.print(F(" "));
    Serial.print(distanciasEnvio[i]);
  }

  bt.print(F(","));
  bt.println(digitalRead(sensorIR));
  Serial.println();

  secuenciaBarrido++;
}

void controlarEnlace() {

  if (enlaceActivo && millis() - ultimoContacto >= WATCHDOG_MS) {

    enlaceActivo = false;
    detenerMotores();            // sin enlace, el robot frena

    encolarBT("LINK_LOST", "");
    Serial.println(F("LINK_LOST"));
  }
}


// ============================== ORDENES =============================
// Formato: ORD|accion=AVANZAR o ORD|accion=CAPTURAR_IZQUIERDA/FRENTE/DERECHA

bool ejecutarAccion(const char *accion) {

  if (strcmp(accion, "DETENER") == 0) {
    // [v6] Solo frena. NO se toca posActual: el servo no se movio, y si se
    // pone en "centro" sin moverlo, el barrido queda desplazado.
    recentrarPendiente = false;
    detenerMotores();

    if (modoRobot == CAPTURANDO) {
      modoRobot = EXPLORANDO;
      giroCapturaPendiente = false;
      esperandoAvanceCaptura = false;
      if (!girando) {
        recentrarSensorTrasCaptura();   // vuelve a barrer desde el centro
      }
    }
    return true;
  }

  if (strcmp(accion, "AVANZAR") == 0)    { avanzar();        return true; }
  if (strcmp(accion, "RETROCEDER") == 0) { retroceder();     return true; }
  if (strcmp(accion, "IZQUIERDA") == 0)  { izquierda();      return true; }
  if (strcmp(accion, "DERECHA") == 0)    { derecha();        return true; }
  // La red neuronal reconoce la canica y envia su direccion.
  if (strcmp(accion, "CAPTURAR_IZQUIERDA") == 0) { iniciarModoCaptura(-1); return true; }
  if (strcmp(accion, "CAPTURAR_FRENTE") == 0)    { iniciarModoCaptura(0);  return true; }
  if (strcmp(accion, "CAPTURAR_DERECHA") == 0)   { iniciarModoCaptura(1);  return true; }
  // Compatibilidad: CAPTURAR sin direccion supone que esta al frente.
  if (strcmp(accion, "CAPTURAR") == 0)           { iniciarModoCaptura(0);  return true; }

  return false;
}

void procesarOrden(char *orden) {

  marcarContacto();

  Serial.print(F("RECIBIDO: "));
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
      Serial.print(F("ACK,"));
      Serial.println(accion);
    } else {
      encolarBT("ERR,", orden);
      Serial.print(F("ERR,"));
      Serial.println(orden);
    }
    return;
  }

  encolarBT("ERR,", orden);
  Serial.print(F("ERR,"));
  Serial.println(orden);
}


// ============================== MODO CAPTURA =========================

// El servidor ya reconocio la canica y envia su ubicacion aproximada:
// -1 = izquierda, 0 = frente, 1 = derecha.
// Arduino no decide si el objeto es una canica: solo ejecuta la orden.
void iniciarModoCaptura(int direccion) {
  if (modoRobot == CAPTURANDO) return;

  modoRobot = CAPTURANDO;
  inicioCaptura = millis();
  proximaAccionCaptura = millis();
  fallosEcoCaptura = 0;
  esperandoAvanceCaptura = false;
  barridoListo = false;

  // La puerta se acciona inmediatamente al recibir la orden de captura.
  // El IR NO controla la puerta: solo confirma que la canica ingreso.
  abrirPuerta();

  // [v6] Antes: frenarServoSensor() y quedaba mirando donde estuviera
  // (normalmente a la derecha, porque la orden llega justo despues del SWP),
  // y la persecucion medi­a "hacia adelante" con el sensor apuntando de costado.
  // Ahora el sensor se lleva al frente. Si esta en medio de un giro, se deja
  // terminar y recien ahi se recentra (asi posActual es exacto).
  if (girando) {
    recentrarPendiente = true;
  } else {
    recentrarSensorTrasCaptura();
  }

  // Orientar el chasis hacia el sector informado por el servidor.
  if (direccion < 0) {
    moverMotores(LOW, HIGH, HIGH, LOW, MS_GIRO_PERSECUCION);
    giroCapturaPendiente = true;
    Serial.println(F("MODO CAPTURA: objetivo a la IZQUIERDA"));
  } else if (direccion > 0) {
    moverMotores(HIGH, LOW, LOW, HIGH, MS_GIRO_PERSECUCION);
    giroCapturaPendiente = true;
    Serial.println(F("MODO CAPTURA: objetivo a la DERECHA"));
  } else {
    giroCapturaPendiente = false;
    detenerMotores();
    Serial.println(F("MODO CAPTURA: objetivo al FRENTE"));
  }
}

// Se ejecuta mientras se persigue la canica.
// El ultrasónico no barre durante este modo: mide únicamente en la
// direccion en la que quedo orientado al detectar el objeto.
void controlarModoCaptura() {
  if (modoRobot != CAPTURANDO) return;

  unsigned long ahora = millis();

  // El IR confirma la captura en detectarIR(). Este limite es un respaldo.
  if (ahora - inicioCaptura >= CAPTURA_MAX_MS) {
    Serial.println(F("CAPTURA: tiempo agotado, regreso a exploracion"));
    detenerMotores();
    modoRobot = EXPLORANDO;
    giroCapturaPendiente = false;
    esperandoAvanceCaptura = false;
    recentrarPendiente = false;
    if (!girando) {
      recentrarSensorTrasCaptura();
    }
    return;
  }

  // [v6] Terminar el giro del sensor (en modo captura controlarBarrido()
  // no corre, asi que alguien tiene que frenarlo).
  if (girando) {
    if (ahora >= finGiro) {
      frenarServoSensor();
      girando = false;
      inicioQuieto = ahora;

      if (recentrarPendiente) {
        recentrarPendiente = false;
        recentrarSensorTrasCaptura();   // ahora posActual es exacto
      }
    }
    return;
  }

  // Esperar a que termine el giro inicial antes de avanzar.
  if (giroCapturaPendiente) {
    if (motorApagarEn == 0) {
      giroCapturaPendiente = false;
      proximaAccionCaptura = ahora;
    }
    return;
  }

  // Si esta terminando una rafaga de avance, medir hacia adelante
  // sin mover el servo del sensor.
  if (esperandoAvanceCaptura) {
    if (motorApagarEn == 0) {
      esperandoAvanceCaptura = false;
      float d = medirConReintentos();

      if (d > 0 && d <= DIST_DETECCION_CANICA) {
        fallosEcoCaptura = 0;
        distanciaActual = d;
        proximaAccionCaptura = millis() + 100;
      } else if (d <= 0) {
        fallosEcoCaptura++;
        if (fallosEcoCaptura >= 3) {
          Serial.println(F("CAPTURA: se perdio el eco, regreso a exploracion"));
          modoRobot = EXPLORANDO;
          detenerMotores();
          recentrarSensorTrasCaptura();
          return;
        }
        proximaAccionCaptura = millis() + 150;
      } else {
        // Objeto fuera del rango: no seguir avanzando a ciegas.
        Serial.println(F("CAPTURA: objetivo fuera de rango, regreso a exploracion"));
        modoRobot = EXPLORANDO;
        detenerMotores();
        recentrarSensorTrasCaptura();
        return;
      }
    }
    return;
  }

  if (motorApagarEn != 0 || ahora < proximaAccionCaptura) return;

  // Avance corto y controlado; despues se vuelve a medir.
  moverMotores(HIGH, LOW, HIGH, LOW, MS_PERSECUCION);
  esperandoAvanceCaptura = true;
}

// Devuelve el servo ultrasónico al centro usando los tiempos calibrados
// del barrido. Se llama al entrar y al salir del modo captura.
void recentrarSensorTrasCaptura() {
  if (posActual < POS_CENTRO) {
    iniciarGiro(true, MS_30_DER * (POS_CENTRO - posActual));
  } else if (posActual > POS_CENTRO) {
    iniciarGiro(false, MS_30_IZQ * (posActual - POS_CENTRO));
  } else {
    frenarServoSensor();
    girando = false;
    inicioQuieto = millis();
    medido = false;
  }
  posActual = POS_CENTRO;
  sentido = 1;
  barridoListo = false;
}

// Finaliza la persecucion cuando el IR confirma que la canica llego
// al mecanismo de captura.
void finalizarCaptura() {
  if (modoRobot != CAPTURANDO) return;

  detenerMotores();
  modoRobot = EXPLORANDO;
  giroCapturaPendiente = false;
  esperandoAvanceCaptura = false;
  recentrarPendiente = false;
  if (!girando) {
    recentrarSensorTrasCaptura();
  }

  Serial.println(F("CAPTURA CONFIRMADA: regreso a exploracion"));
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
  if (modoRobot == CAPTURANDO) return;
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

  Serial.print(F("POSICION: "));
  Serial.print(posActual);
  Serial.print(F(" | DISTANCIA: "));
  Serial.println(distancia);

  // No iniciar la persecucion por una distancia local: cualquier obstaculo
  // podria activarla. La decision de reconocer la canica y su direccion
  // corresponde exclusivamente al servidor, que responde con una ORD.

  // 5) ENVIAR SWP SOLAMENTE AL COMPLETAR
  //    la pasada de izquierda a derecha
  if (posActual == CANT_POS - 1 && sentido == 1) {

    Serial.println(F("BARRIDO COMPLETO: IZQUIERDA A DERECHA"));

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

    if (estadoIRConfirmado == LOW && modoRobot == CAPTURANDO) {
      // El IR solo confirma que la canica ya ingreso a ORQUITA.
      // La puerta ya fue accionada al recibir la orden de captura.
      numeroCaptura++;
      finalizarCaptura();

      // [v5] CAP por la cola (antes bt.print directo)
      char num[12];
      snprintf(num, sizeof(num), "%lu", numeroCaptura);
      encolarBT("CAP,", num);

      Serial.print(F("CANICA DETECTADA "));
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
  // el del sensor en iniciarGiro(), la puerta al iniciarModoCaptura().
  Serial.println(F("================================"));
  Serial.println(F("ARDUINO INICIANDO"));
  Serial.println(F("SETUP EJECUTADO"));
  Serial.println(F("================================"));

  bt.println(F("READY"));

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
    Serial.println(F("LOOP FUNCIONANDO"));
  }

  recibirBluetooth();
  if (modoRobot == EXPLORANDO) {
    controlarBarrido();
    frenoEmergencia();
  } else {
    controlarModoCaptura();
  }
  detectarIR();
  enviarBarrido();
  vaciarColaBT();
  enviarTelemetria();
  actualizarMotores();
  actualizarPuerta();
  controlarEnlace();
}
