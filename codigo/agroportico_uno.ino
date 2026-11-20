/*
  AgroPórtico P-01 · Firmware del Arduino Uno (control de bajo nivel)
  Universidad Manuela Beltrán · Sistemas Digitales Aplicados · 2026

  Qué hace esta tarjeta:
    - Lee el joystick y mueve el carro en modo manual.
    - Recibe órdenes del ESP32 por UART en modo automático (mover, frenar, asperjar).
    - Maneja el driver L298N (4 motores TT en dos lados), el servo de la boquilla y la bomba.
  Protocolo con el ESP32 (una línea por orden, termina en '\n'):
    MODE,M            -> modo manual (manda el joystick)
    MODE,A            -> modo automático (manda el ESP32)
    MOV,izq,der       -> velocidad de cada lado, -255 a 255 (PWM con signo)
    SPRAY,A|B|C       -> aspersión según la decisión; responde "DONE,<letra>"
    STOP              -> parada inmediata de motores y bomba
*/
#include <Servo.h>
#include <SoftwareSerial.h>

// ---------- 1. Pines ----------
const uint8_t ENA = 5, IN1 = 7, IN2 = 8;    // lado izquierdo (2 motores en paralelo)
const uint8_t ENB = 6, IN3 = 4, IN4 = 12;   // lado derecho  (2 motores en paralelo)
const uint8_t PIN_SERVO = 3;                // orientación de la boquilla
const uint8_t PIN_BOMBA = A3;               // MOSFET de la bomba
const uint8_t JOY_X = A0, JOY_Y = A1, JOY_SW = A2;
SoftwareSerial enlace(10, 11);              // RX, TX hacia el ESP32 (TX con divisor 5 V -> 3,3 V)

// ---------- 2. Parámetros ----------
const int ZONA_MUERTA = 40;                 // cuentas del ADC alrededor del centro (512)
const unsigned long T_ASPERSION_B = 1000;   // ms, boquillas por pares (opción B)
const unsigned long T_ASPERSION_C = 1800;   // ms, planta completa (opción C)
const unsigned long T_SIN_ORDEN = 500;      // ms sin órdenes en automático -> frena (seguridad)

Servo boquilla;
char modo = 'M';
unsigned long ultimaOrden = 0;
String linea;

// ---------- 3. Motores ----------
void lado(uint8_t en, uint8_t a, uint8_t b, int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(a, pwm > 0);                 // el sentido lo fija el puente H
  digitalWrite(b, pwm < 0);
  analogWrite(en, abs(pwm));                // la magnitud la fija el PWM (Vmed = δ·Vcc)
}
void mover(int izq, int der) { lado(ENA, IN1, IN2, izq); lado(ENB, IN3, IN4, der); }
void parar() { mover(0, 0); digitalWrite(PIN_BOMBA, LOW); }

// ---------- 4. Aspersión ----------
void rociar(int angulo, unsigned long ms) {
  boquilla.write(angulo);
  delay(250);                               // tiempo para que el servo llegue
  digitalWrite(PIN_BOMBA, HIGH);
  delay(ms);
  digitalWrite(PIN_BOMBA, LOW);
}
void asperjar(char d) {
  parar();                                  // nunca se asperja en movimiento
  if (d == 'B') {                           // flancos: izquierda y derecha
    rociar(45, T_ASPERSION_B / 2);
    rociar(135, T_ASPERSION_B / 2);
  } else if (d == 'C') {                    // planta completa: flancos y arriba
    rociar(45, T_ASPERSION_C / 3);
    rociar(90, T_ASPERSION_C / 3);
    rociar(135, T_ASPERSION_C / 3);
  }                                         // 'A': omitir, no abre la bomba
  boquilla.write(90);
  enlace.print("DONE,"); enlace.println(d);
}

// ---------- 5. Joystick (modo manual) ----------
int eje(uint8_t pin) {
  int v = analogRead(pin) - 512;            // ADC de 10 bits centrado en 0
  if (abs(v) < ZONA_MUERTA) return 0;
  return map(v, -512, 511, -255, 255);
}
void manual() {
  int avance = -eje(JOY_Y);                 // arriba = adelante
  int giro = eje(JOY_X);
  mover(avance + giro, avance - giro);      // mezcla diferencial (tanque)
  if (digitalRead(JOY_SW) == LOW) asperjar('C');  // botón: aspersión de prueba
}

// ---------- 6. Órdenes del ESP32 ----------
void atender(String s) {
  ultimaOrden = millis();
  if (s.startsWith("MODE,")) { modo = s.charAt(5); parar(); }
  else if (s.startsWith("MOV,")) {
    int coma = s.indexOf(',', 4);
    mover(s.substring(4, coma).toInt(), s.substring(coma + 1).toInt());
  }
  else if (s.startsWith("SPRAY,")) asperjar(s.charAt(6));
  else if (s == "STOP") parar();
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(PIN_BOMBA, OUTPUT); pinMode(JOY_SW, INPUT_PULLUP);
  boquilla.attach(PIN_SERVO); boquilla.write(90);
  enlace.begin(9600);
  parar();
}

void loop() {
  while (enlace.available()) {              // arma la línea carácter por carácter
    char c = enlace.read();
    if (c == '\n') { linea.trim(); atender(linea); linea = ""; }
    else linea += c;
  }
  if (modo == 'M') manual();
  else if (millis() - ultimaOrden > T_SIN_ORDEN) parar();  // perro guardián del enlace
  delay(10);
}
