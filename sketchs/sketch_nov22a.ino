#include <Servo.h>

// ============================== PINES ===============================
// Sensores
const int trigPin  = 2;
const int echoPin  = 3;
const int sensorIR = 4;

// Servos (ambos de rotacion continua)
const int PIN_SERVO_PUERTA = 5;   // puerta: gira siempre
const int PIN_SERVO_SENSOR = 6;   // exploracion: barrido con ultrasonido

// Motores TT (puente H L298N)
const int ENA = 8;
const int IN1 = 9;
const int IN2 = 10;
const int IN3 = 11;
const int IN4 = 12;
const int ENB = 13;

// ============================== AJUSTES =============================
// --- Servo puerta ---
const int VELOCIDAD_PUERTA = 0;     // 0 o 180 segun el sentido que quieras (90 = parado)
const int PARADA_PUERTA    = 90;

// --- Servo sensor: barrido por tiempo ---
const int PARADA_SENSOR        = 90;
const int VEL_SENSOR_DERECHA   = 180;
const int VEL_SENSOR_IZQUIERDA = 0;
const unsigned long TIEMPO_180_GRADOS = 700;   // CALIBRAR

// --- Ultrasonido ---
const float DISTANCIA_OBSTACULO = 20.0;        // cm
const unsigned long INTERVALO_MEDICION = 100;  // ms

// --- Evasion ---
const unsigned long TIEMPO_EVASION = 600;      // ms girando a la derecha

// --- IR ---
const unsigned long TIEMPO_ESTABLE_IR = 40;    // ms (antirrebote)

// --- Arranque ---
const unsigned long RETRASO_INICIAL = 2000;    // ms antes de empezar a moverse

// ============================== OBJETOS =============================
Servo servoSensor;
Servo servoPuerta;

// ============================== ESTADO ==============================
// Barrido
bool barriendoADerecha = true;
unsigned long inicioTramo = 0;

// Ultrasonido
float distanciaActual = -1;
unsigned long ultimaMedicion = 0;

// Movimiento del robot
enum EstadoRobot { AVANZANDO, EVITANDO };
EstadoRobot estadoRobot = AVANZANDO;
unsigned long inicioEvasion = 0;

// IR
int ultimaLecturaIR = HIGH;
int estadoIRConfirmado = HIGH;
unsigned long tiempoCambioIR = 0;
unsigned long numeroCaptura = 0;

// ============================== MOTORES =============================
void detenerMotores() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void avanzar() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void girarIzquierda() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void girarDerecha() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ============================== ULTRASONIDO =========================
float medirDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duracion = pulseIn(echoPin, HIGH, 18000);
  if (duracion == 0) return -1;

  return duracion * 0.0343 / 2.0;
}

void controlarUltrasonico() {
  if (millis() - ultimaMedicion < INTERVALO_MEDICION) return;
  ultimaMedicion = millis();

  distanciaActual = medirDistancia();

  // Si hay obstaculo y estaba avanzando, empieza la evasion
  if (estadoRobot == AVANZANDO && distanciaActual > 0 && distanciaActual < DISTANCIA_OBSTACULO) {
    Serial.print("Obstaculo: ");
    Serial.print(distanciaActual);
    Serial.println(" cm");

    estadoRobot = EVITANDO;
    inicioEvasion = millis();
    girarDerecha();
  }
}

// ============================== MOVIMIENTO ==========================
void controlarMovimiento() {
  if (estadoRobot == EVITANDO && millis() - inicioEvasion >= TIEMPO_EVASION) {
    estadoRobot = AVANZANDO;
    avanzar();
  }
}

// ============================== SENSOR IR ===========================
void detectarIR() {
  int lectura = digitalRead(sensorIR);

  if (lectura != ultimaLecturaIR) {
    tiempoCambioIR = millis();
    ultimaLecturaIR = lectura;
  }

  if (millis() - tiempoCambioIR >= TIEMPO_ESTABLE_IR && lectura != estadoIRConfirmado) {
    estadoIRConfirmado = lectura;

    if (estadoIRConfirmado == LOW) {
      numeroCaptura++;
      Serial.print(">>> CANICA DETECTADA <<< ");
      Serial.println(numeroCaptura);
    }
  }
}

// ============================== BARRIDO (pin 6) =====================
void iniciarTramo() {
  inicioTramo = millis();
  servoSensor.write(barriendoADerecha ? VEL_SENSOR_DERECHA : VEL_SENSOR_IZQUIERDA);
}

void controlarBarrido() {
  if (millis() - inicioTramo >= TIEMPO_180_GRADOS) {
    barriendoADerecha = !barriendoADerecha;
    iniciarTramo();
  }
}

// ============================== SETUP ===============================
void setup() {
  Serial.begin(9600);

  // Sensores
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(sensorIR, INPUT_PULLUP);

  // Motores
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  detenerMotores();

  // Servos: arrancan detenidos
  servoPuerta.attach(PIN_SERVO_PUERTA);
  servoSensor.attach(PIN_SERVO_SENSOR);
  servoPuerta.write(PARADA_PUERTA);
  servoSensor.write(PARADA_SENSOR);

  // Inicializar IR
  ultimaLecturaIR = digitalRead(sensorIR);
  estadoIRConfirmado = ultimaLecturaIR;
  tiempoCambioIR = millis();

  Serial.println("Orquita lista. Arranca en 2 segundos...");
  delay(RETRASO_INICIAL);

  // Todo en marcha
  servoPuerta.write(VELOCIDAD_PUERTA);
  iniciarTramo();
  avanzar();
}

// ============================== LOOP ================================
void loop() {
  controlarBarrido();
  controlarUltrasonico();
  controlarMovimiento();
  detectarIR();
}
