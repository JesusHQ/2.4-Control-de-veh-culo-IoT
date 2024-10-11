#include <ESP32Servo.h>

// Pines del sensor ultrasónico
const int EchoPin = 33;   // Pin conectado al Echo del sensor
const int TriggerPin = 25; // Pin conectado al Trigger del sensor

// Matriz de pines de los motores
const int motores[4] = {12, 14, 27, 26}; // {motorDelanteroAdelante, motorDelanteroAtras, motorTraseroAdelante, motorTraseroAtras}

// Pines y configuración del servo
#define PINSERVO 32
Servo servoMotor;
int paso = 50;

void setup() {
  // Iniciar el monitor serie
  Serial.begin(115200);

  // Configurar los pines de los motores como salidas
  for (int i = 0; i < 4; i++) {
    pinMode(motores[i], OUTPUT);
  }

  // Configurar pines del sensor ultrasónico
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // Iniciar el servo
  servoMotor.attach(PINSERVO);
}

void loop() {
  float distancia = ping(TriggerPin, EchoPin);
  Serial.print("Distancia: ");
  Serial.println(distancia);

  if (distancia > 50.0) {
    // No hay obstáculos, avanzar
    avanzar(500);
  } else {
    // Hay un obstáculo, buscar una nueva salida
    detenerse(500);
    explorarYBuscarSalida();
  }
}

// Función para medir la distancia con el sensor ultrasónico
float ping(int TriggerPin, int EchoPin) {
  unsigned long duration;
  float distanceCm;

  digitalWrite(TriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);

  duration = pulseIn(EchoPin, HIGH);
  distanceCm = float(duration) / 58.2;
  return distanceCm;
}

// Función para explorar y buscar una nueva salida
void explorarYBuscarSalida() {
  // Mover el servo a 45 grados a la izquierda y medir
  servoMotor.write(45);
  delay(500);
  float distanciaIzquierda = ping(TriggerPin, EchoPin);
  
  // Mover el servo a 135 grados a la derecha y medir
  servoMotor.write(135);
  delay(500);
  float distanciaDerecha = ping(TriggerPin, EchoPin);

  // Regresar el servo a su posición central
  servoMotor.write(90);
  delay(500);

  if (distanciaIzquierda > 20.0) {
    girarIzquierda(500); // Si hay espacio a la izquierda, girar
  } else if (distanciaDerecha > 20.0) {
    girarDerecha(500); // Si hay espacio a la derecha, girar
  } else {
    retroceder(1000); // Si no hay espacio, retroceder
    girar180(); // Gira 180 grados después de retroceder
  }
}

void girar180() {
  // Para girar 180 grados, activa los motores delanteros y traseros en direcciones opuestas.
  moverMotores(HIGH, LOW, LOW, HIGH, 660);  // Ajusta el tiempo para que gire aproximadamente 180 grados
}

// Función genérica para controlar el movimiento de los motores
void moverMotores(int motor1, int motor2, int motor3, int motor4, int tiempo) {
  digitalWrite(motores[0], motor1);  // Motor delantero adelante
  digitalWrite(motores[1], motor2);  // Motor delantero atrás
  digitalWrite(motores[2], motor3);  // Motor trasero adelante
  digitalWrite(motores[3], motor4);  // Motor trasero atrás
  delay(tiempo);
  detenerMotores();
}

// Funciones de control de movimiento
void retroceder(int tiempo) {
  moverMotores(LOW, HIGH, LOW, HIGH, tiempo);  // Retroceder
}

void avanzar(int tiempo) {
  moverMotores(HIGH, LOW, HIGH, LOW, tiempo);  // Avanzar
}

void girarIzquierda(int tiempo) {
  moverMotores(HIGH, LOW, LOW, HIGH, tiempo);  // Girar a la izquierda
}

void girarDerecha(int tiempo) {
  moverMotores(LOW, HIGH, HIGH, LOW, tiempo);  // Girar a la derecha
}

void detenerse(int tiempo) {
  detenerMotores();
  delay(tiempo);
}

// Función para detener los motores
void detenerMotores() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motores[i], LOW);  // Detener todos los motores
  }
}
