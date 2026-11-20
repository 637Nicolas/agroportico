/*
  AgroPórtico P-01 · Firmware del ESP32 (control de alto nivel)
  Universidad Manuela Beltrán · Sistemas Digitales Aplicados · 2026

  Qué hace esta tarjeta:
    - Recibe la misión y las órdenes del celular por Bluetooth.
    - En modo automático avanza planta por planta, frena, pide la aspersión A/B/C al Uno y registra.
    - Lee el GPS (posición de cada aplicación) y el voltaje de batería, y lo muestra en la pantalla OLED.
  Órdenes por Bluetooth (app tipo terminal):
    M            modo manual          A           modo automático
    S            parada inmediata     PLAN:CABCAB decisión por planta (A omitir, B pares, C completa)
*/
#include <BluetoothSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- 1. Pines y periféricos ----------
const int GPS_RX = 16, GPS_TX = 17;          // UART2 <- módulo GPS NEO-6M
const int UNO_RX = 26, UNO_TX = 25;          // UART1 <-> Arduino Uno
const int PIN_BAT = 34;                      // ADC: batería por divisor 100k/47k
BluetoothSerial bt;
TinyGPSPlus gps;
Adafruit_SSD1306 pantalla(128, 64, &Wire, -1);
HardwareSerial gpsSerial(2), uno(1);

// ---------- 2. Parámetros de la misión ----------
const float SEPARACION = 0.30;               // m entre plantas (maqueta)
const float V_CRUCERO = 0.30;                // m/s
const int   PWM_CRUCERO = 170;               // PWM que da 0,30 m/s (calibrado en piso)
const float V_MIN_BAT = 6.6;                 // V, por debajo no arranca
String plan = "CABCAB";                      // decisión por planta
enum Estado { REPOSO, MANUAL, AVANZAR, ASPERJAR, ESPERAR, FIN } estado = REPOSO;
int planta = 0;
unsigned long t0 = 0;
float x = 0;                                 // posición estimada (m)
unsigned long tPrev = 0;                     // última actualización de x
int pwmEnviado = -1;                         // último PWM mandado al Uno
unsigned long tEnvio = 0;

// ---------- 3. Utilidades ----------
float bateria() {                            // V = D/(2^12 - 1)·Vref, corregido por el divisor
  return analogReadMilliVolts(PIN_BAT) / 1000.0 * (147.0 / 47.0);
}
void alUno(const String& s) { uno.println(s); }
void mostrar(const char* msg) {
  pantalla.clearDisplay(); pantalla.setCursor(0, 0);
  pantalla.printf("AgroPortico P-01\nBat: %.1f V\n", bateria());
  pantalla.printf("Planta %d/%d  %c\n", planta + 1, (int)plan.length(),
                  planta < (int)plan.length() ? plan[planta] : '-');
  pantalla.printf("x = %.2f m\n%s", x, msg);
  pantalla.display();
}
void registrar(char d) {                     // trazabilidad: una línea por planta
  bt.printf("REG,%d,%c,%.6f,%.6f,%lu\n", planta + 1, d,
            gps.location.lat(), gps.location.lng(), millis() / 1000);
}
void detener(const char* motivo) { alUno("STOP"); estado = REPOSO; mostrar(motivo); }

// ---------- 4. Órdenes del celular ----------
void ordenes() {
  if (!bt.available()) return;
  String s = bt.readStringUntil('\n'); s.trim();
  if (s == "S") detener("PARADA");
  else if (s == "M") { alUno("MODE,M"); estado = MANUAL; mostrar("Manual"); }
  else if (s == "A") {
    if (bateria() < V_MIN_BAT) { bt.println("ALARMA,BATERIA"); return; }
    alUno("MODE,A"); planta = 0; x = 0; estado = AVANZAR; tPrev = millis(); pwmEnviado = -1;
  }
  else if (s.startsWith("PLAN:")) { plan = s.substring(5); bt.println("OK,PLAN," + plan); }
}

// ---------- 5. Máquina de estados del modo automático ----------
void automatico() {
  unsigned long ahora = millis();
  float dt = (ahora - tPrev) / 1000.0; tPrev = ahora;
  switch (estado) {
    case AVANZAR: {
      float meta = SEPARACION * (planta + 1);
      float falta = meta - x;
      float v = (falta < 0.05) ? V_CRUCERO * 0.4 : V_CRUCERO;  // frena antes de llegar
      int pwm = (falta < 0.05) ? PWM_CRUCERO * 0.4 : PWM_CRUCERO;
      if (pwm != pwmEnviado || ahora - tEnvio > 200) {   // no satura el enlace a 9600 baudios
        alUno("MOV," + String(pwm) + "," + String(pwm));
        pwmEnviado = pwm; tEnvio = ahora;
      }
      x += v * dt;                           // estimación por tiempo (sin encoder)
      if (x >= meta) { alUno("STOP"); estado = ASPERJAR; }
      break;
    }
    case ASPERJAR:
      alUno("SPRAY," + String(plan[planta])); estado = ESPERAR; t0 = ahora;
      mostrar("Asperjando");
      break;
    case ESPERAR:                            // espera "DONE" del Uno (máx. 4 s)
      if (uno.available()) {
        String r = uno.readStringUntil('\n');
        if (r.startsWith("DONE")) {
          registrar(plan[planta]);
          planta++;
          estado = (planta >= (int)plan.length()) ? FIN : AVANZAR;
          tPrev = millis(); pwmEnviado = -1;     // el tiempo quieto no cuenta como avance
        }
      } else if (ahora - t0 > 4000) detener("ERROR: UNO");
      break;
    case FIN: detener("Fin del surco"); break;
    default: break;
  }
}

void setup() {
  bt.begin("AgroPortico");
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  uno.begin(9600, SERIAL_8N1, UNO_RX, UNO_TX);
  uno.setTimeout(50); bt.setTimeout(50);     // lecturas de línea sin bloquear el ciclo
  pantalla.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  pantalla.setTextColor(SSD1306_WHITE);
  analogReadResolution(12);
  mostrar("Listo");
}

void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());   // NMEA -> lat, lon
  ordenes();
  if (estado == AVANZAR || estado == ASPERJAR || estado == ESPERAR || estado == FIN) automatico();
  static unsigned long tPant = 0;
  if (millis() - tPant > 500) { tPant = millis(); if (estado == AVANZAR) mostrar("Avanzando"); }
  delay(20);                                 // ciclo de control de 50 Hz
}
