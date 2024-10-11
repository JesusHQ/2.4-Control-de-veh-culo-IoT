#include <ESP32Servo.h>

// Pines del sensor ultrasónico
const int EchoPin = 33;   // Pin conectado al Echo del sensor
const int TriggerPin = 25; // Pin conectado al Trigger del sensor

// Pines de los motores
const int motorDelanteroAtras = 14;     // OUT1
const int motorDelanteroAdelante = 12;  // OUT2
const int motorTraseroAtras = 26;       // OUT3
const int motorTraseroAdelante = 27;    // OUT4

// Pines y configuración del servo
#define PINSERVO 32
Servo servoMotor;
int paso = 50;

void setup() {
  // Iniciar el monitor serie
  Serial.begin(115200);

  // Configurar pines de motores como salidas
  pinMode(motorDelanteroAdelante, OUTPUT);
  pinMode(motorDelanteroAtras, OUTPUT);
  pinMode(motorTraseroAdelante, OUTPUT);
  pinMode(motorTraseroAtras, OUTPUT);

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

// Funciones de control de movimiento
void retroceder(int tiempo) {
  digitalWrite(motorDelanteroAdelante, HIGH);
  digitalWrite(motorTraseroAdelante, HIGH);
  delay(tiempo);
  detenerMotores();
}

void avanzar(int tiempo) {
  digitalWrite(motorDelanteroAtras, HIGH);
  digitalWrite(motorTraseroAtras, HIGH);
  delay(tiempo);
  detenerMotores();
}

void girarIzquierda(int tiempo) {
  digitalWrite(motorDelanteroAdelante, HIGH);
  digitalWrite(motorTraseroAtras, HIGH);
  delay(tiempo);
  detenerMotores();
}

void girarDerecha(int tiempo) {
  digitalWrite(motorDelanteroAtras, HIGH);
  digitalWrite(motorTraseroAdelante, HIGH);
  delay(tiempo);
  detenerMotores();
}

void detenerse(int tiempo) {
  detenerMotores();
  delay(tiempo);
}

void detenerMotores() {
  digitalWrite(motorDelanteroAdelante, LOW);
  digitalWrite(motorDelanteroAtras, LOW);
  digitalWrite(motorTraseroAdelante, LOW);
  digitalWrite(motorTraseroAtras, LOW);
}

void girar180() {
  // Para girar 180 grados, activa los motores delanteros y traseros en direcciones opuestas.
  // Esto hará que el carrito gire sobre su eje.
  digitalWrite(motorDelanteroAdelante, HIGH);
  digitalWrite(motorTraseroAtras, HIGH);
  delay(660);  // Ajusta el tiempo para que gire aproximadamente 180 grados
  detenerMotores(); // Detiene los motores después del giro
}