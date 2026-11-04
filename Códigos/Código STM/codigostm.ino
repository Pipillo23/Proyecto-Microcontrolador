// 1 = modo de prueba sin Raspberry Pi: arranca solo al encender, no usa el watchdog
//     y no activa el I2C esclavo de la Pi. El sensor de color SÍ funciona, con la
//     librería Wire normal (solo maestro), igual que en el código original.
// PONERLO EN 0 PARA LA ENTREGA FINAL.
#define MODO_SIN_PI 1

#include <Arduino.h>
#if MODO_SIN_PI
#include <Wire.h>                // I2C solo maestro (el sensor de color no necesita más)
#else
#include <Wire_slave.h>          // I2C maestro + esclavo del núcleo Maple. Debe ir ANTES de
                                 // cualquier librería que incluya <Wire.h> (Adafruit_TCS34725)
#endif
#include <Adafruit_TCS34725.h>
#include "protocolo.h"           // mismo archivo que usa la Raspberry Pi (va en la carpeta del sketch)
const unsigned long T_ARRANQUE_MS = 3000;   // tiempo para poner el carro en la pista

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
const uint16_t PWM_MAX  = 65535;   
const uint16_t VEL_BASE = 25000;   
const uint16_t VEL_CORR = 20000;   
const uint16_t VEL_GIRO = 20000;   

// I2C1 por hardware, ESCLAVO 0x42 hacia la Raspberry Pi (pines fijos del periférico):
//   PB6 = SCL -> GPIO3 (pin 5) de la Pi      PB7 = SDA -> GPIO2 (pin 3) de la Pi      GND común
const int SCL_RASP = PB6;
const int SDA_RASP = PB7;

// I2C2 por hardware, MAESTRO hacia el TCS34725
const int SCL_COLOR = PB10;
const int SDA_COLOR = PB11;

// LED de la placa (activo en bajo): encendido mientras corre setup(). Si se queda
// encendido, el programa se trabó en la inicialización.
const int LED_PLACA = PC13;

// --- Sensor de color ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
#if MODO_SIN_PI
TwoWire WireColor(2);            // I2C2 por hardware con la librería Wire (como el código original)
#define BUS_COLOR WireColor
#else
#define BUS_COLOR Wire1          // I2C2 de la librería Wire_slave
#endif
bool tcs_ok = false;
uint16_t col_r = 0, col_g = 0, col_b = 0, col_c = 0;
unsigned long t_ult_color = 0;
const unsigned long T_LECTURA_COLOR_MS = 60;    // > 50 ms de integración

// --- Estado del carro según protocolo.h (lo maneja loop) ---
volatile uint8_t estado = EST_IDLE;             // EST_*; lo lee también ISR_Linea
uint8_t  banderas_fijas = 0;                    // BAN_COM_PERDIDA (se borra con START)
uint8_t  cfg_color = 0;                         // CMD_COL_* activo (0 = sin configurar)
uint8_t  cfg_bif   = 0;                         // CMD_BIF_* activo (0 = sin configurar)
uint8_t  color_actual = COLOR_NINGUNO;          // COLOR_* bajo el sensor
uint16_t count_red = 0;
uint16_t count_green = 0;
uint16_t count_blue = 0;
uint8_t  ult_seq = 0;                           // SEQ del último comando ejecutado
uint8_t  ult_resultado = RES_OK;
uint8_t  pub_cnt = 0;

// --- Recepción I2C: la llena la ISR de I2C, la vacía loop ---
const uint8_t COLA_LEN = 8;
volatile uint8_t cola_seq[COLA_LEN], cola_cmd[COLA_LEN];
volatile uint8_t cola_ini = 0, cola_fin = 0;
volatile unsigned long t_ult_trama = 0;         // última trama válida (incluye HEARTBEAT)

// --- Trama de estado lista para enviar (la arma loop, la envía la ISR de I2C) ---
uint8_t trama_estado[ST_LEN];
unsigned long t_ult_publicacion = 0;

unsigned long last_serial_print = 0;

// Estado actual de las salidas (lo escribe el ISR, lo imprime loop)
volatile int16_t vel_izq = 0, vel_der = 0;   // + adelante, - reversa
volatile const char* motor_state = "DETENIDO";

// Prototipos
void ISR_Linea();
#if !MODO_SIN_PI
void i2c_recibir(int cantidad);
void i2c_responder();
#endif
bool bus_con_pullups(int pin_scl, int pin_sda);
void ejecutar_comando(uint8_t seq, uint8_t cmd);
void publicar_estado();
void arrancar();
void detener(uint8_t nuevo_estado);
bool linea_perdida();
void stop_motors();
void set_motors(int16_t izq, int16_t der, const char* state);
void drive_motor(int pin_pwm, int pin_dir, int16_t v);

void setup() {
  // Puerto serie USB: solo para depurar con la PC (la Pi se comunica únicamente por I2C).
  // Sin "while (!Serial);": con el carro en batería no hay USB y el programa se quedaría ahí.
  Serial.begin(115200);

  pinMode(LED_PLACA, OUTPUT);
  digitalWrite(LED_PLACA, LOW);    // LED encendido: inicializando

  // Motores: IN1 e IN3 con PWM (TIM2 CH2 y CH4), IN2 e IN4 como dirección
  pinMode(IN1, PWM);
  pinMode(IN3, PWM);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);

  stop_motors();

  // Configuración de Pines de Sensores IR (Entrada)
  // Con pull-up interno: si el módulo seguidor está sin alimentar (solo USB, sin batería)
  // los pines no quedan al aire. Al aire, el ruido dispara las interrupciones CHANGE sin
  // parar y el USB deja de atenderse. Con el módulo alimentado, su salida manda sobre el pull-up.
  pinMode(IR_S1, INPUT_PULLUP);
  pinMode(IR_S2, INPUT_PULLUP);
  pinMode(IR_S3, INPUT_PULLUP);
  pinMode(IR_S4, INPUT_PULLUP);
  pinMode(IR_S5, INPUT_PULLUP);

  // Sensor de color TCS34725 en I2C2 (PB10/PB11), como maestro
  // Sin pull-ups en el bus, el primer begin() del I2C se queda trabado en el reset del
  // bus del núcleo Maple (i2c_bus_reset). Por eso se revisan las líneas antes.
  if (bus_con_pullups(SCL_COLOR, SDA_COLOR)) {
    BUS_COLOR.begin();
    tcs_ok = tcs.begin(TCS34725_ADDRESS, &BUS_COLOR);
  }
  if (tcs_ok) {
    Serial.println(F("[INFO] Sensor TCS34725 detectado correctamente."));
  } else {
    Serial.println(F("[ERROR] No se encontró el sensor TCS34725. Revise las conexiones I2C."));
  }

#if MODO_SIN_PI
  // Modo de prueba: el I2C esclavo de la Pi no se activa
  Serial.println(F("[AVISO] MODO_SIN_PI: I2C de la Raspberry Pi deshabilitado."));
#else
  // I2C1 como esclavo en 0x42. La trama de estado queda armada ANTES de habilitar el
  // esclavo, para que la primera lectura de la Pi ya reciba una trama válida.
  publicar_estado();
  t_ult_trama = millis();
  Wire.onReceive(i2c_recibir);     // la Pi escribe una trama de comando (4 bytes)
  Wire.onRequest(i2c_responder);   // la Pi lee la trama de estado (17 bytes)
  // Solo con pull-ups en PB6/PB7 (Pi conectada o resistencias de 4.7k a 3.3 V).
  // Si la Pi se conecta después de encender, hay que reiniciar la Blue Pill.
  if (bus_con_pullups(SCL_RASP, SDA_RASP)) {
    Wire.begin(I2C_DIR_STM32);
    Serial.println(F("[INFO] Bus I2C de la Pi detectado: esclavo 0x42 activo."));
  } else {
    Serial.println(F("[AVISO] Sin pull-ups en PB6/PB7 (Pi desconectada): I2C esclavo deshabilitado."));
  }
#endif

  // --- Interrupciones externas por cambio en los pines del seguidor ---
  // Cada sensor usa su propia línea EXTI (0, 1, 12, 13, 14), así que no chocan.
  // CHANGE: entra tanto cuando un sensor ve la línea como cuando la deja de ver.
  // Timer2 queda dedicado al PWM de los motores.
  attachInterrupt(IR_S1, ISR_Linea, CHANGE);
  attachInterrupt(IR_S2, ISR_Linea, CHANGE);
  attachInterrupt(IR_S3, ISR_Linea, CHANGE);
  attachInterrupt(IR_S4, ISR_Linea, CHANGE);
  attachInterrupt(IR_S5, ISR_Linea, CHANGE);

  digitalWrite(LED_PLACA, HIGH);   // LED apagado: setup() terminó
  Serial.println(F("[INFO] Sistema inicializado."));
}

void loop() {
  #if MODO_SIN_PI
  // Arranque automático, una sola vez, al pasar T_ARRANQUE_MS desde que se encendió
  static bool arranco = false;
  if (!arranco && millis() >= T_ARRANQUE_MS) {
    arranco = true;
    arrancar();
  }
  // Con el USB conectado: 's' arranca, 'x' detiene (desde el Monitor Serie)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' && estado == EST_IDLE) arrancar();
    else if (c == 'x') detener(EST_IDLE);
  }
#endif

  // 1) Ejecutar los comandos que llegaron por I2C, en orden
  while (cola_ini != cola_fin) {
    uint8_t seq = cola_seq[cola_ini];
    uint8_t cmd = cola_cmd[cola_ini];
    cola_ini = (uint8_t)((cola_ini + 1) % COLA_LEN);
    ejecutar_comando(seq, cmd);
  }

  // 2) Watchdog de enlace: en movimiento y sin tramas de la Pi por 500 ms -> parar.
  //    t_ult_trama se lee ANTES que millis(): si la ISR lo actualiza en medio, la resta no da negativa.
  unsigned long t_trama = t_ult_trama;
  if (!MODO_SIN_PI && estado_en_movimiento(estado) && millis() - t_trama > T_WATCHDOG_STM_MS) {
    banderas_fijas |= BAN_COM_PERDIDA;
    detener(EST_FALLA_COM);
    Serial.println(F("[WATCHDOG] Sin comandos de la Pi por 500 ms: carro detenido."));
  }

  // Avisar por serial apenas cambia el estado (una intersección puede durar menos de 200 ms)
  static uint8_t estado_anterior = 0xFF;
  uint8_t est = estado;
  if (est != estado_anterior) {
    estado_anterior = est;
    Serial.print(F("[ESTADO] "));
    Serial.print(millis());
    Serial.print(F(" ms -> "));
    Serial.println(estado_nombre(est));
  }

  // 3) Leer el sensor de color sin bloquear (getRawData() se queda ~50 ms en un delay)
  if (tcs_ok && millis() - t_ult_color >= T_LECTURA_COLOR_MS) {
    t_ult_color = millis();
    col_c = tcs.read16(TCS34725_CDATAL);
    col_r = tcs.read16(TCS34725_RDATAL);
    col_g = tcs.read16(TCS34725_GDATAL);
    col_b = tcs.read16(TCS34725_BDATAL);
  }

  // 4) Refrescar la trama de estado cada 10 ms
  if (millis() - t_ult_publicacion >= T_PUBLICACION_STM_MS) {
    t_ult_publicacion = millis();
    publicar_estado();
  }

  // 5) Depuración por USB cada 200 ms
  if (millis() - last_serial_print >= 200) {
    last_serial_print = millis();

    // Leer estado actual de los sensores IR
    int s1 = digitalRead(IR_S1);
    int s2 = digitalRead(IR_S2);
    int s3 = digitalRead(IR_S3);
    int s4 = digitalRead(IR_S4);
    int s5 = digitalRead(IR_S5);

    // Mostrar datos legibles en el Monitor Serial
    Serial.print(F("[SENSORES IR] S1: ")); Serial.print(s1);
    Serial.print(F(" | S2: ")); Serial.print(s2);
    Serial.print(F(" | S3 (Centro): ")); Serial.print(s3);
    Serial.print(F(" | S4: ")); Serial.print(s4);
    Serial.print(F(" | S5: ")); Serial.print(s5);
    Serial.print(F(" || RGB -> R: ")); Serial.print(col_r);
    Serial.print(F(" G: ")); Serial.print(col_g);
    Serial.print(F(" B: ")); Serial.println(col_b);

    // Copia consistente del estado de salidas
    noInterrupts();
    int16_t vi = vel_izq, vd = vel_der;
    const char* st = (const char*)motor_state;
    interrupts();

    Serial.print(F("[SALIDAS] Izq (IN1): ")); Serial.print(vi);
    Serial.print(F("/")); Serial.print(PWM_MAX);
    Serial.print(F(" | Der (IN3): ")); Serial.print(vd);
    Serial.print(F("/")); Serial.print(PWM_MAX);
    Serial.print(F(" || Motores: ")); Serial.print(st);
    Serial.print(F(" | Estado: ")); Serial.println(estado_nombre(estado));

    Serial.print(F("[PROTOCOLO] ult_seq: ")); Serial.print(ult_seq);
    Serial.print(F(" (")); Serial.print(resultado_nombre(ult_resultado));
    Serial.print(F(") | colores: ")); Serial.print(cmd_nombre(cfg_color));
    Serial.print(F(" | bif: ")); Serial.print(cmd_nombre(cfg_bif));
    Serial.print(F(" | ms desde la ultima trama: ")); Serial.println(millis() - t_ult_trama);
  }
}

// Interrupción externa por cambio en cualquiera de los 5 sensores de línea
// Disposición: S1 (izq. extremo) - S2 - S3 (centro) - S4 - S5 (der. extremo)
// Motor A = izquierdo (IN1/IN2), Motor B = derecho (IN3/IN4)
// Seguimiento: solo S2, S3 y S4. Los extremos S1 y S5 solo detectan intersecciones.
// Correcciones (S2/S4): bajan la velocidad de un lado
void ISR_Linea() {
  if (estado != EST_RUN && estado != EST_BIFURCACION) return;

  int s1 = digitalRead(IR_S1);
  int s2 = digitalRead(IR_S2);
  int s3 = digitalRead(IR_S3);
  int s4 = digitalRead(IR_S4);
  int s5 = digitalRead(IR_S5);

  // Intersección: los dos extremos ven línea a la vez. Se avisa a la Pi con
  // EST_BIFURCACION y se vuelve a RUN cuando los extremos dejan de verla.
  if (s1 == LINEA && s5 == LINEA) estado = EST_BIFURCACION;
  else if (estado == EST_BIFURCACION) estado = EST_RUN;

  if (s3 == LINEA) {
    // Centrado: ambos a velocidad base
    set_motors(VEL_BASE, VEL_BASE, "AVANZA");
  }
  else if (s2 == LINEA) {
    // Línea un poco a la izquierda: izquierdo en reversa, derecho adelante
    set_motors(-(int16_t)VEL_CORR, VEL_BASE, "CORRIGE_IZQ");
  }
  else if (s4 == LINEA) {
    // Línea un poco a la derecha: izquierdo adelante, derecho en reversa
    set_motors(VEL_BASE, -(int16_t)VEL_CORR, "CORRIGE_DER");
  }
  // Ningún sensor del medio: mantiene la última salida
}

// ============================================================================
// COMUNICACIÓN I2C CON LA RASPBERRY PI (protocolo.h v0x02)
// ============================================================================

#if !MODO_SIN_PI
// ISR de I2C: la Pi escribió una trama. Corre con prioridad máxima, así que solo
// valida y encola; el comando se ejecuta en loop.
void i2c_recibir(int cantidad) {
  (void)cantidad;
  uint8_t t[CMD_LEN];
  uint32_t n = 0;
  while (Wire.available()) {
    int b = Wire.read();
    if (n < CMD_LEN) t[n] = (uint8_t)b;
    n++;
  }
  uint8_t seq, cmd;
  if (!cmd_leer(t, n, &seq, &cmd)) return;     // largo o CRC incorrecto: se descarta
  t_ult_trama = millis();                       // cualquier trama válida alimenta el watchdog
  if (cmd == CMD_HEARTBEAT) return;             // no cambia nada más
  uint8_t sig = (uint8_t)((cola_fin + 1) % COLA_LEN);
  if (sig == cola_ini) return;                  // cola llena: la Pi lo reenviará
  cola_seq[cola_fin] = seq;
  cola_cmd[cola_fin] = cmd;
  cola_fin = sig;
}

// ISR de I2C: la Pi lee. Se entrega la última trama de estado ya armada.
void i2c_responder() {
  Wire.write(trama_estado, ST_LEN);
}
#endif

// Misma lógica de comandos que el STM32 simulado de carro_pi.c
void ejecutar_comando(uint8_t seq, uint8_t cmd) {
  // La Pi reenvía con la MISMA SEQ si no vio la confirmación a tiempo: no se ejecuta dos veces
  if (seq == ult_seq) return;

  uint8_t r = RES_OK;
  if (cmd == CMD_STOP) {
    detener(EST_IDLE);                          // siempre se ejecuta, en cualquier estado
  }
  else if (cmd == CMD_START) {
    if (estado_en_movimiento(estado))  r = RES_OK;            // ya estaba en marcha
    else if (estado != EST_IDLE)       r = RES_ERR_ESTADO;    // FALLA_COM: requiere STOP
    else if (!cfg_color || !cfg_bif)   r = RES_ERR_SIN_CONFIG;
    else {
      count_red = count_green = count_blue = 0;
      banderas_fijas &= (uint8_t)~BAN_COM_PERDIDA;
      arrancar();
    }
  }
  else if (cmd_es_color(cmd)) {
    if (estado != EST_IDLE) r = RES_ERR_ESTADO;   // configurar solo detenido
    else cfg_color = cmd;
  }
  else if (cmd_es_bif(cmd)) {
    if (estado != EST_IDLE) r = RES_ERR_ESTADO;
    else cfg_bif = cmd;
  }
  else {
    r = RES_ERR_DESCONOCIDO;
  }

  ult_seq = seq;
  ult_resultado = r;
  publicar_estado();                            // la Pi ve la confirmación en su siguiente lectura
}

// Arma la trama de estado de 17 bytes y la deja lista para la ISR de I2C
void publicar_estado() {
  estado_t e;
  e.version       = PROTO_VERSION;
  e.pub_cnt       = ++pub_cnt;
  e.ult_seq       = ult_seq;
  e.ult_resultado = ult_resultado;
  e.estado        = estado;
  e.banderas      = (uint8_t)(banderas_fijas
                    | (linea_perdida() ? BAN_LINEA_PERDIDA : 0)
                    | (tcs_ok ? 0 : BAN_SENSOR_COLOR_FALLA));
  e.cfg_color     = cfg_color;
  e.cfg_bif       = cfg_bif;
  e.color         = color_actual;
  e.cuenta_r      = count_red;
  e.cuenta_g      = count_green;
  e.cuenta_b      = count_blue;

  uint8_t b[ST_LEN];
  estado_codificar(&e, b);
  noInterrupts();                               // que la Pi no lea una trama a medio copiar
  for (uint8_t i = 0; i < ST_LEN; i++) trama_estado[i] = b[i];
  interrupts();
}

// Pasa a RUN y calcula la primera salida: si el carro está quieto sobre la línea
// ningún pin cambia, así que no habría interrupción que lo arranque.
void arrancar() {
  noInterrupts();
  estado = EST_RUN;
  ISR_Linea();
  interrupts();
}

// Cambia de estado ANTES de parar los motores: si entra ISR_Linea en medio, ya no los mueve
void detener(uint8_t nuevo_estado) {
  estado = nuevo_estado;
  stop_motors();
}

bool linea_perdida() {
  return digitalRead(IR_S1) != LINEA && digitalRead(IR_S2) != LINEA &&
         digitalRead(IR_S3) != LINEA && digitalRead(IR_S4) != LINEA &&
         digitalRead(IR_S5) != LINEA;
}

// true si las dos líneas del bus están en alto con pull-down interno activo, es decir,
// si hay resistencias pull-up externas. Sin ellas no se debe llamar a begin().
bool bus_con_pullups(int pin_scl, int pin_sda) {
  pinMode(pin_scl, INPUT_PULLDOWN);
  pinMode(pin_sda, INPUT_PULLDOWN);
  delay(1);
  bool ok = digitalRead(pin_scl) == HIGH && digitalRead(pin_sda) == HIGH;
  pinMode(pin_scl, INPUT);
  pinMode(pin_sda, INPUT);
  return ok;
}

// ============================================================================
// MOTORES
// ============================================================================

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
