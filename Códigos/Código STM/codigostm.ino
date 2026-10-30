#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ArduinoJson.h>

// --- Definición de Pines (BluePill - STM32F103C8T6) ---
// Core: stm32duino:STM32F1 (maple), subida por ST-Link
// Motores (L298N) - ENA/ENB con jumper (siempre habilitados)
const int IN1 = PA1;   // PWM (TIM2 CH2) - motor izquierdo
const int IN2 = PA2;   // dirección motor izquierdo (LOW adelante / HIGH reversa)
const int IN3 = PA3;   // PWM (TIM2 CH4) - motor derecho
const int IN4 = PA4;   // dirección motor derecho (LOW adelante / HIGH reversa)

// Seguidor de Pista (5 Canales: S1 izq. extremo ... S5 der. extremo)
const int IR_S1 = PB0;
const int IR_S2 = PB1;
const int IR_S3 = PB12;
const int IR_S4 = PB13;
const int IR_S5 = PB14;

// Nivel que entrega el sensor cuando está SOBRE la línea
const int LINEA = LOW;

// --- Velocidades (PWM en IN1 e IN3, 0 a PWM_MAX) ---
const uint16_t PWM_MAX  = 75;
const uint16_t VEL_BASE = 37;   // velocidad normal de ambos lados
const uint16_t VEL_CORR = 25;   // lado lento en corrección suave (S2/S4)
const uint16_t VEL_GIRO = 25;   // reversa de la rueda interior en giro (S1/S5)

const int SDA_RASP = PB6;
const int SCL_RASP = PB7;

const int SDA_COLOR = PB11;
const int SCL_COLOR = PB10;

// Variables de Control y Estado
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
TwoWire WireColor(2);   // I2C2 por hardware: PB10 = SCL, PB11 = SDA

volatile bool is_running = false;
int count_red = 0;
int count_green = 0;
int count_blue = 0;

String fork_direction = "center";
String action_red = "stop";
String action_green = "forward";
String action_blue = "turn_left";

unsigned long last_serial_print = 0;

// Estado actual de las salidas (lo escribe el ISR, lo imprime loop)
volatile int16_t vel_izq = 0, vel_der = 0;   // + adelante, - reversa
volatile const char* motor_state = "DETENIDO";

// Prototipos
void Timer_ISR();
void process_uart_command(String json_str);
void send_telemetry();
void stop_motors();
void set_motors(int16_t izq, int16_t der, const char* state);
void drive_motor(int pin_pwm, int pin_dir, int16_t v);

void setup() {
  // Inicialización de Puertos Serie
  Serial.begin(115200);
  while (!Serial);

  // Motores: IN1 e IN3 con PWM (TIM2 CH2 y CH4), IN2 e IN4 como dirección
  pinMode(IN1, PWM);
  pinMode(IN3, PWM);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);

  // PWM: 72 MHz / 72 = 1 MHz -> PWM_MAX cuentas por periodo (150 -> ~6.7 kHz)
  Timer2.setPrescaleFactor(72);
  Timer2.setOverflow(PWM_MAX);
  Timer2.refresh();
  Timer2.resume();

  stop_motors();

  // Configuración de Pines de Sensores IR (Entrada)
  pinMode(IR_S1, INPUT);
  pinMode(IR_S2, INPUT);
  pinMode(IR_S3, INPUT);
  pinMode(IR_S4, INPUT);
  pinMode(IR_S5, INPUT);

  // Inicialización Sensor de Color TCS34725 (I2C2 en PB10/PB11)
  WireColor.begin();

  if (tcs.begin(TCS34725_ADDRESS, &WireColor)) {
    Serial.println(F("[INFO] Sensor TCS34725 detectado correctamente."));
  } else {
    Serial.println(F("[ERROR] No se encontró el sensor TCS34725. Revise las conexiones I2C."));
  }

  // --- Interrupción de control por Timer3 (50 Hz / 20 ms) ---
  // Timer2 queda dedicado al PWM de los motores
  Timer3.pause();
  Timer3.setPeriod(20000);                       // en microsegundos
  Timer3.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer3.setCompare(TIMER_CH1, 1);
  Timer3.attachCompare1Interrupt(Timer_ISR);
  Timer3.refresh();
  Timer3.resume();

  Serial.println(F("[INFO] Sistema inicializado y listo. Esperando comandos..."));
}

void loop() {
  // Procesamiento de comandos entrantes por UART
  if (Serial.available() > 0) {
    String incomingString = Serial.readStringUntil('\n');
    incomingString.trim();
    if (incomingString.length() > 0) {
      process_uart_command(incomingString);
    }
  }

  // Enviar telemetría y lecturas de sensores al Monitor Serial cada 200 ms
  if (millis() - last_serial_print >= 200) {
    last_serial_print = millis();

    // Leer estado actual de los sensores IR
    int s1 = digitalRead(IR_S1);
    int s2 = digitalRead(IR_S2);
    int s3 = digitalRead(IR_S3);
    int s4 = digitalRead(IR_S4);
    int s5 = digitalRead(IR_S5);

    // Leer valores de color RGB del TCS34725
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    // Mostrar datos legibles en el Monitor Serial
    Serial.print(F("[SENSORES IR] S1: ")); Serial.print(s1);
    Serial.print(F(" | S2: ")); Serial.print(s2);
    Serial.print(F(" | S3 (Centro): ")); Serial.print(s3);
    Serial.print(F(" | S4: ")); Serial.print(s4);
    Serial.print(F(" | S5: ")); Serial.print(s5);
    Serial.print(F(" || RGB -> R: ")); Serial.print(r);
    Serial.print(F(" G: ")); Serial.print(g);
    Serial.print(F(" B: ")); Serial.println(b);

    // Copia consistente del estado de salidas
    noInterrupts();
    int16_t vi = vel_izq, vd = vel_der;
    const char* st = (const char*)motor_state;
    bool run = is_running;
    interrupts();

    Serial.print(F("[SALIDAS] Izq (IN1): ")); Serial.print(vi);
    Serial.print(F("/")); Serial.print(PWM_MAX);
    Serial.print(F(" | Der (IN3): ")); Serial.print(vd);
    Serial.print(F("/")); Serial.print(PWM_MAX);
    Serial.print(F(" || Estado: ")); Serial.print(st);
    Serial.print(F(" | Run: ")); Serial.println(run ? "SI" : "NO");
  }
}

// Interrupción a 50 Hz (Seguimiento de Línea con 5 sensores)
// Disposición: S1 (izq. extremo) - S2 - S3 (centro) - S4 - S5 (der. extremo)
// Motor A = izquierdo (IN1/IN2), Motor B = derecho (IN3/IN4)
// Correcciones (S2/S4): bajan la velocidad de un lado
// Giros (S1/S5): la rueda interior va en reversa a VEL_GIRO
void Timer_ISR() {
  if (!is_running) return;

  int s1 = digitalRead(IR_S1);
  int s2 = digitalRead(IR_S2);
  int s3 = digitalRead(IR_S3);
  int s4 = digitalRead(IR_S4);
  int s5 = digitalRead(IR_S5);

  if (s3 == LINEA) {
    // Centrado: ambos a velocidad base
    set_motors(VEL_BASE, VEL_BASE, "AVANZA");
  }
  else if (s2 == LINEA) {
    // Línea un poco a la izquierda: baja el izquierdo
    set_motors(VEL_CORR, VEL_BASE, "CORRIGE_IZQ");
  }
  else if (s4 == LINEA) {
    // Línea un poco a la derecha: baja el derecho
    set_motors(VEL_BASE, VEL_CORR, "CORRIGE_DER");
  }
  else if (s1 == LINEA) {
    // Línea en el extremo izquierdo: izquierdo en reversa, derecho adelante
    set_motors(-(int16_t)VEL_GIRO, VEL_BASE, "GIRO_IZQ");
  }
  else if (s5 == LINEA) {
    // Línea en el extremo derecho: izquierdo adelante, derecho en reversa
    set_motors(VEL_BASE, -(int16_t)VEL_GIRO, "GIRO_DER");
  }
  // Ningún sensor: mantiene la última salida
}

void process_uart_command(String json_str) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json_str);

  if (error) {
    Serial.print(F("[JSON ERROR] Falló al parsear: "));
    Serial.println(error.c_str());
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "start") == 0) {
    is_running = true;
    Serial.println(F("[CMD] Motor iniciado"));
    send_telemetry();
  }
  else if (strcmp(cmd, "stop") == 0) {
    is_running = false;
    stop_motors();
    Serial.println(F("[CMD] Motor detenido"));
    send_telemetry();
  }
  else if (strcmp(cmd, "config") == 0) {
    if (is_running) {
      JsonDocument errDoc;
      errDoc["error"] = "Cannot reconfigure while running";
      serializeJson(errDoc, Serial);
      Serial.println();
    } else {
      if (doc.containsKey("R")) action_red = doc["R"].as<String>();
      if (doc.containsKey("G")) action_green = doc["G"].as<String>();
      if (doc.containsKey("B")) action_blue = doc["B"].as<String>();
      Serial.println(F("[CMD] Configuración actualizada"));
      send_telemetry();
    }
  }
  else if (strcmp(cmd, "fork_dir") == 0) {
    if (doc.containsKey("dir")) {
      fork_direction = doc["dir"].as<String>();
      Serial.print(F("[CMD] Dirección de bifurcación fijada a: "));
      Serial.println(fork_direction);
    }
  }
}

void send_telemetry() {
  JsonDocument doc;
  doc["status"] = is_running ? "run" : "stop";

  JsonObject counts = doc["counts"].to<JsonObject>();
  counts["R"] = count_red;
  counts["G"] = count_green;
  counts["B"] = count_blue;

  serializeJson(doc, Serial);
  Serial.println();
}

void stop_motors() {
  set_motors(0, 0, "DETENIDO");
}

void set_motors(int16_t izq, int16_t der, const char* state) {
  drive_motor(IN1, IN2, izq);   // motor izquierdo
  drive_motor(IN3, IN4, der);   // motor derecho
  vel_izq = izq;
  vel_der = der;
  motor_state = state;
}

// Maneja un motor con un pin PWM y un pin digital de dirección.
// v > 0: adelante -> dir LOW,  PWM = v
// v < 0: reversa  -> dir HIGH, PWM = PWM_MAX - |v|
//        (en la parte alta del PWM ambos pines quedan en HIGH = freno)
// v = 0: ambos en LOW -> freno
void drive_motor(int pin_pwm, int pin_dir, int16_t v) {
  uint16_t m = (v >= 0) ? (uint16_t)v : (uint16_t)(-v);
  if (m > PWM_MAX) m = PWM_MAX;

  if (v >= 0) {
    digitalWrite(pin_dir, LOW);
    pwmWrite(pin_pwm, m);
  } else {
    digitalWrite(pin_dir, HIGH);
    pwmWrite(pin_pwm, PWM_MAX - m);
  }
}
