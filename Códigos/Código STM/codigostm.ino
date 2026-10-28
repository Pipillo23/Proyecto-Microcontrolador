#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ArduinoJson.h>
#include <HardwareTimer.h>

// --- Definición de Pines (BluePill - STM32F103C8T6) ---
// Motores (L298N)
const int ENA = PA0;
const int IN1 = PA1;
const int IN2 = PA2;
const int ENB = PA3;
const int IN3 = PA4;
const int IN4 = PA5;

// Seguidor de Pista (5 Canales: S1, S2, S3, S4, S5)
const int IR_S1 = PB0;
const int IR_S2 = PB1;
const int IR_S3 = PB12;
const int IR_S4 = PB13;
const int IR_S5 = PB14;

const int SDA_RASP = PB6;
const int SCL_RASP = PB7;

const int SDA_COLOR = PB11;
const int SCL_COLOR = PB10;

// Variables de Control y Estado
HardwareTimer *MyTim = NULL;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

volatile bool is_running = false;
int count_red = 0;
int count_green = 0;
int count_blue = 0;

String fork_direction = "center";
String action_red = "stop";
String action_green = "forward";
String action_blue = "turn_left";

unsigned long last_serial_print = 0;

// Prototipos
void Timer_ISR();
void process_uart_command(String json_str);
void send_telemetry();
void stop_motors();
void parse_action_string(String action);

void setup() {
  // Inicialización de Puertos Serie
  Serial.begin(115200); // UART con Raspberry Pi / Monitor Serial
  while(!Serial);

  // Configuración de Pines de Motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuración de Pines de Sensores IR (Entrada)
  pinMode(IR_S1, INPUT);
  pinMode(IR_S2, INPUT);
  pinMode(IR_S3, INPUT);
  pinMode(IR_S4, INPUT);
  pinMode(IR_S5, INPUT);

  // Inicialización Sensor de Color TCS34725 (I2C1 en PB6/PB7)
  Wire.setSCL(SCL_COLOR);
  Wire.setSDA(SDA_COLOR);
  Wire.begin();
  
  if (tcs.begin()) {
    Serial.println(F("[INFO] Sensor TCS34725 detectado correctamente."));
  } else {
    Serial.println(F("[ERROR] No se encontró el sensor TCS34725. Revise las conexiones I2C."));
  }

  // --- Configuración de Interrupción por Timer (50 Hz / 20 ms) ---
  MyTim = new HardwareTimer(TIM2);
  MyTim->setOverflow(50, HERTZ_FORMAT);
  MyTim->attachInterrupt(Timer_ISR);
  MyTim->resume();

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
  }
}

// Interrupción de Hardware a 50Hz (Lógica de Centrado y Seguimiento de Línea)
void Timer_ISR() {
  if (!is_running) return;

  // Lectura de los 5 canales del seguidor de pista
  int s1 = digitalRead(IR_S1);
  int s2 = digitalRead(IR_S2);
  int s3 = digitalRead(IR_S3);
  int s4 = digitalRead(IR_S4);
  int s5 = digitalRead(IR_S5);

  // Lógica de control básica de ejemplo basada en el sensor central (S3)
  if (s3 == HIGH) {
    // Centrado perfecto: Avanzar recto
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } 
  else if (s2 == HIGH) {
    // Corrección leve hacia la izquierda
    analogWrite(ENA, 120);
    analogWrite(ENB, 160);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } 
  else if (s4 == HIGH) {
    // Corrección leve hacia la derecha
    analogWrite(ENA, 160);
    analogWrite(ENB, 120);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
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
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
