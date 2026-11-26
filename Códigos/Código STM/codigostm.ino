// 1 = modo de prueba sin Raspberry Pi: arranca solo al encender, no usa el watchdog
//     y no activa el I2C esclavo de la Pi. El sensor de color SÍ funciona, con la
//     librería Wire normal (solo maestro), igual que en el código original.
// PONERLO EN 0 PARA LA ENTREGA FINAL.
#define MODO_SIN_PI 0

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
const int IR_S1 = PB14;
const int IR_S2 = PB0;
const int IR_S3 = PB12;
const int IR_S4 = PB13;
const int IR_S5 = PA7;

// Nivel que entrega el sensor cuando está SOBRE la línea
const int LINEA = HIGH;

// --- Velocidades (PWM en IN1 e IN3, 0 a PWM_MAX) ---
const uint16_t PWM_MAX     = 65535;
const int32_t  VEL_BASE    = 25000;   // velocidad en recta
const int32_t  VEL_REV_MAX = 15000;   // reversa máxima de la rueda interior en curvas cerradas

// --- Control PD de posición (ISR_Control, cada 2 ms por Timer3) ---
// Error = posición de la línea medida en "sensores":
//   -2 bajo S1 ... -1 bajo S2 ... 0 bajo S3 ... +1 bajo S4 ... +2 bajo S5
//   (dos sensores vecinos a la vez dan el punto medio: -1.5, -0.5, +0.5, +1.5)
// Se pueden cambiar por el Monitor Serie: p8000  d500  f0.3
float Kp          = 8000.0f;  // cuentas de PWM por sensor de error
float Kd          = 500.0f;   // cuentas de PWM por (sensor/s) de cambio del error
float FRENO_CURVA = 0.30f;    // con el error máximo la velocidad base baja 30 %
const uint32_t T_CONTROL_US = 2000;                  // 500 Hz
const float    DT_CONTROL   = T_CONTROL_US * 1e-6f;
const float    E_PERDIDA    = 2.5f;   // error cuando la línea se salió por un extremo
volatile float error_linea  = 0;      // lo escribe ISR_Linea, lo lee ISR_Control
float e_anterior = 0, d_filtrada = 0; // solo ISR_Control (y arrancar)

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
// Integración de 24 ms (antes 50 ms). Un segmento de 5 cm a 0.5 m/s pasa bajo el
// sensor en 100 ms: caben ~4 mediciones y el color se confirma con 2 seguidas.
// Máximo de cuenta con 24 ms: (256 - 0xF6) * 1024 = 10240.
#define COLOR_T_INTEGRACION TCS34725_INTEGRATIONTIME_24MS
Adafruit_TCS34725 tcs = Adafruit_TCS34725(COLOR_T_INTEGRACION, TCS34725_GAIN_4X);
const uint32_t CUENTA_MAX_COLOR = (256u - (uint8_t)COLOR_T_INTEGRACION) * 1024u;
#if MODO_SIN_PI
TwoWire WireColor(2);            // I2C2 por hardware con la librería Wire (como el código original)
#define BUS_COLOR WireColor
#else
#define BUS_COLOR Wire1          // I2C2 de la librería Wire_slave
#endif

// Cómo se entera el STM32 de que hay una medición nueva:
//   1 = POR INTERRUPCIÓN: el pin INT del TCS34725 baja al terminar cada medición
//       (registro PERS = 0 + AIEN). Cablear INT -> PA8. La ISR solo levanta una
//       bandera; la lectura I2C se hace en loop. Si se pierde un flanco, cada
//       T_RESPALDO_COLOR_MS se consulta el registro STATUS para recuperarse.
//   0 = POR SONDEO (polling): sin cable INT, se consulta el bit AINT de STATUS
//       cada T_SONDEO_COLOR_MS.
#define COLOR_POR_INTERRUPCION 1
const int COLOR_INT = PA8;       // INT del TCS34725: open-drain, activo en bajo. EXTI8 no
                                 // choca con los IR (EXTI 0, 1, 12, 13 y 14)
const unsigned long T_RESPALDO_COLOR_MS = 100;
const unsigned long T_SONDEO_COLOR_MS   = 5;
volatile bool color_listo = false;               // la levanta ISR_Color
bool tcs_ok = false;
bool color_falla = false;                        // 5 lecturas I2C fallidas seguidas
uint8_t fallas_color = 0;
uint16_t col_r = 0, col_g = 0, col_b = 0, col_c = 0;   // última lectura cruda
unsigned long t_ult_consulta_color = 0;

// --- Calibración del color (ver clasificar_color) ---
// Referencia de BLANCO: valores normalizados (por mil de C) y C crudo sobre papel
// blanco. Con el USB conectado, poner papel blanco bajo el sensor y enviar 'b' por
// el Monitor Serie: se toman los valores actuales y se imprimen para dejarlos fijos aquí.
uint16_t blanco_r = 333, blanco_g = 333, blanco_b = 333;
uint16_t blanco_c = 2000;
const uint16_t OSCURO_PCT  = 15;     // C < 15 % del C del blanco -> negro (tape) o nada
const uint16_t DOMINIO_MIN = 1150;   // el canal ganador debe superar al blanco en 15 %
const uint16_t MARGEN_MIN  = 150;    // y ganarle al segundo canal por 0.15 (relativo al blanco)
const uint8_t  N_CONFIRMA  = 2;      // mediciones seguidas iguales para aceptar un color
uint8_t color_candidato = COLOR_NINGUNO, n_iguales = 0;
uint8_t ultimo_contado  = COLOR_NINGUNO; // evita contar dos veces el mismo segmento

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
volatile int32_t vel_izq = 0, vel_der = 0;   // + adelante, - reversa
volatile const char* motor_state = "DETENIDO";

// Prototipos
void ISR_Linea();
void ISR_Control();
void imprimir_pd();
void ISR_Color();
void atender_sensor_color();
bool tcs_leer(uint8_t reg, uint8_t *buf, uint8_t n);
void registrar_falla_color();
uint8_t clasificar_color(uint16_t c, uint16_t r, uint16_t g, uint16_t b);
void procesar_color();
void calibrar_blanco();
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
void set_motors(int32_t izq, int32_t der, const char* state);
void drive_motor(int pin_pwm, int pin_dir, int32_t v);

void setup() {
  // Puerto serie USB: solo para depurar con la PC (la Pi se comunica únicamente por I2C).
  // Sin "while (!Serial);": con el carro en batería no hay USB y el programa se quedaría ahí.
  Serial.begin(115200);
  Serial.setTimeout(50);           // para leer las ganancias (p, d, f) sin trabar el loop

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
    // Interrupción en CADA fin de medición ("dato listo"), sin umbrales: con PERS = 0
    // el sensor activa AINT (y baja INT) cada ciclo, sin importar AILT/AIHT.
    tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE);
    tcs.setInterrupt(true);          // AIEN = 1
    tcs.clearInterrupt();            // arranca con INT en alto
#if COLOR_POR_INTERRUPCION
    pinMode(COLOR_INT, INPUT_PULLUP);              // INT es open-drain: necesita pull-up
    attachInterrupt(COLOR_INT, ISR_Color, FALLING);
    Serial.println(F("[INFO] Color por interrupción (INT del TCS34725 en PA8)."));
#else
    Serial.println(F("[INFO] Color por sondeo del registro STATUS."));
#endif
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
  ISR_Linea();                     // posición inicial de la línea

  // --- Interrupción de control por Timer3 (500 Hz / 2 ms): el PD ---
  // Timer2 genera el PWM; Timer3 solo marca el tiempo (canal 1 sin salida a pin).
  Timer3.pause();
  Timer3.setPeriod(T_CONTROL_US);                // en microsegundos
  Timer3.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  Timer3.setCompare(TIMER_CH1, 1);
  Timer3.attachCompare1Interrupt(ISR_Control);
  Timer3.refresh();
  Timer3.resume();

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
#endif
  // Con el USB conectado (Monitor Serie): 'b' toma la referencia de blanco del sensor
  // de color. En MODO_SIN_PI además 's' arranca y 'x' detiene.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'b') calibrar_blanco();
    else if (c == 'p') { Kp = Serial.parseFloat(); imprimir_pd(); }
    else if (c == 'd') { Kd = Serial.parseFloat(); imprimir_pd(); }
    else if (c == 'f') { FRENO_CURVA = Serial.parseFloat(); imprimir_pd(); }
#if MODO_SIN_PI
    else if (c == 's' && estado == EST_IDLE) arrancar();
    else if (c == 'x') detener(EST_IDLE);
#endif
  }

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

  // 3) Sensor de color: si hay medición nueva, leer, normalizar, clasificar y contar
  atender_sensor_color();

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
    Serial.println();

    // Sensor de color: crudo, normalizado (por mil de C) y color confirmado
    Serial.print(F("[COLOR] C: ")); Serial.print(col_c);
    Serial.print(F(" R: ")); Serial.print(col_r);
    Serial.print(F(" G: ")); Serial.print(col_g);
    Serial.print(F(" B: ")); Serial.print(col_b);
    Serial.print(F(" | norm r: ")); Serial.print(color_normalizar(col_r, col_c));
    Serial.print(F(" g: ")); Serial.print(color_normalizar(col_g, col_c));
    Serial.print(F(" b: ")); Serial.print(color_normalizar(col_b, col_c));
    Serial.print(F(" | color: ")); Serial.print(color_nombre(color_actual));
    Serial.print(F(" | conteo R/G/B: ")); Serial.print(count_red);
    Serial.print(F("/")); Serial.print(count_green);
    Serial.print(F("/")); Serial.print(count_blue);
    if (color_falla) Serial.print(F(" | SENSOR EN FALLA"));
    Serial.println();

    // Copia consistente del estado de salidas
    noInterrupts();
    int32_t vi = vel_izq, vd = vel_der;
    float err = error_linea;
    const char* st = (const char*)motor_state;
    interrupts();

    Serial.print(F("[SALIDAS] Izq (IN1): ")); Serial.print(vi);
    Serial.print(F("/")); Serial.print(PWM_MAX);
    Serial.print(F(" | Der (IN3): ")); Serial.print(vd);
    Serial.print(F("/")); Serial.print(PWM_MAX);
    Serial.print(F(" || error: ")); Serial.print(err);
    Serial.print(F(" | Motores: ")); Serial.print(st);
    Serial.print(F(" | Estado: ")); Serial.println(estado_nombre(estado));

    Serial.print(F("[PROTOCOLO] ult_seq: ")); Serial.print(ult_seq);
    Serial.print(F(" (")); Serial.print(resultado_nombre(ult_resultado));
    Serial.print(F(") | colores: ")); Serial.print(cmd_nombre(cfg_color));
    Serial.print(F(" | bif: ")); Serial.print(cmd_nombre(cfg_bif));
    Serial.print(F(" | ms desde la ultima trama: ")); Serial.println(millis() - t_ult_trama);
  }
}

// Interrupción externa por cambio en cualquiera de los 5 sensores de línea.
// Ya NO mueve los motores: solo calcula dónde está la línea (error_linea) y detecta
// intersecciones. Los motores los maneja ISR_Control a frecuencia fija.
// Disposición: S1 (izq. extremo) - S2 - S3 (centro) - S4 - S5 (der. extremo)
void ISR_Linea() {
  bool v1 = digitalRead(IR_S1) == LINEA;
  bool v2 = digitalRead(IR_S2) == LINEA;
  bool v3 = digitalRead(IR_S3) == LINEA;
  bool v4 = digitalRead(IR_S4) == LINEA;
  bool v5 = digitalRead(IR_S5) == LINEA;

  // Intersección: los dos extremos ven línea a la vez. Se avisa a la Pi con
  // EST_BIFURCACION y se vuelve a RUN cuando los extremos dejan de verla.
  bool interseccion = v1 && v5;
  if (estado == EST_RUN || estado == EST_BIFURCACION) {
    if (interseccion) estado = EST_BIFURCACION;
    else if (estado == EST_BIFURCACION) estado = EST_RUN;
  }

  // Posición = promedio de las posiciones de los sensores que ven la línea.
  // En una intersección los extremos no cuentan: ven la otra rama, no la nuestra.
  int suma = 0, n = 0;
  if (!interseccion) {
    if (v1) { suma -= 2; n++; }
    if (v5) { suma += 2; n++; }
  }
  if (v2) { suma -= 1; n++; }
  if (v3) { n++; }
  if (v4) { suma += 1; n++; }

  static float e_valido = 0;           // última posición en la que sí se vio la línea
  if (n > 0) {
    e_valido = (float)suma / n;
    error_linea = e_valido;
  }
  else if (e_valido <= -1.0f) error_linea = -E_PERDIDA;   // se salió por la izquierda: girar fuerte
  else if (e_valido >=  1.0f) error_linea =  E_PERDIDA;   // se salió por la derecha
  else                        error_linea = e_valido;     // hueco entre sensores: mantener
}

// Interrupción de Timer3 cada 2 ms: control PD.
//   w = Kp*e + Kd*de/dt      izquierda = v + w      derecha = v - w
// e < 0 (línea a la izquierda) -> w < 0 -> la rueda izquierda va más lento -> gira a la izquierda.
// La D frena el giro cuando la línea ya viene volviendo al centro: eso quita el zigzag.
void ISR_Control() {
  if (estado != EST_RUN && estado != EST_BIFURCACION) return;

  float e = error_linea;
  float d = (e - e_anterior) / DT_CONTROL;
  e_anterior = e;
  d_filtrada += 0.1f * (d - d_filtrada);  // pasa-bajos (~20 ms): el error cambia en escalones

  float w = Kp * e + Kd * d_filtrada;

  // Más lento cuanto más lejos está la línea del centro (curvas)
  float a = fabsf(e);
  if (a > 2.0f) a = 2.0f;
  float v = VEL_BASE * (1.0f - FRENO_CURVA * a / 2.0f);

  int32_t izq = (int32_t)(v + w);
  int32_t der = (int32_t)(v - w);
  izq = constrain(izq, -VEL_REV_MAX, (int32_t)PWM_MAX);
  der = constrain(der, -VEL_REV_MAX, (int32_t)PWM_MAX);
  set_motors(izq, der, "PD");
}

void imprimir_pd() {
  Serial.print(F("[PD] Kp = ")); Serial.print(Kp);
  Serial.print(F("  Kd = ")); Serial.print(Kd);
  Serial.print(F("  freno en curva = ")); Serial.println(FRENO_CURVA);
}

// ============================================================================
// SENSOR DE COLOR TCS34725 (I2C2, el STM32 es MAESTRO de este bus)
// ============================================================================

// ISR del pin INT del TCS34725: terminó una medición. No se hace I2C aquí: la
// transferencia maestra de I2C2 usa sus propias interrupciones y espera, y no debe
// bloquear al esclavo I2C1 de la Pi. Solo se avisa a loop.
void ISR_Color() {
  color_listo = true;
}

// Lee n registros seguidos del TCS34725 en UNA transacción (auto-incremento).
// Leer los 8 bytes juntos asegura que C, R, G y B son de la misma medición.
bool tcs_leer(uint8_t reg, uint8_t *buf, uint8_t n) {
  BUS_COLOR.beginTransmission(TCS34725_ADDRESS);
  BUS_COLOR.write((uint8_t)(TCS34725_COMMAND_BIT | 0x20 | reg));  // 0x20 = auto-incremento
  if (BUS_COLOR.endTransmission() != 0) return false;
  if (BUS_COLOR.requestFrom(TCS34725_ADDRESS, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)BUS_COLOR.read();
  return true;
}

void registrar_falla_color() {
  if (fallas_color < 255) fallas_color++;
  if (fallas_color >= 5) color_falla = true;     // la Pi lo ve como BAN_SENSOR_COLOR_FALLA
}

// Llamada en cada vuelta de loop. Solo hace I2C cuando hay (o puede haber) dato nuevo.
void atender_sensor_color() {
  if (!tcs_ok) return;
#if COLOR_POR_INTERRUPCION
  const unsigned long t_consulta = T_RESPALDO_COLOR_MS;  // respaldo si se pierde un flanco
#else
  const unsigned long t_consulta = T_SONDEO_COLOR_MS;    // sondeo normal
#endif
  unsigned long ahora = millis();
  bool hay_dato = false;

  if (color_listo) {                    // aviso del pin INT
    color_listo = false;
    hay_dato = true;
  } else if (ahora - t_ult_consulta_color >= t_consulta) {
    t_ult_consulta_color = ahora;
    uint8_t st;
    if (!tcs_leer(TCS34725_STATUS, &st, 1)) { registrar_falla_color(); return; }
    hay_dato = (st & TCS34725_STATUS_AINT) != 0;      // AINT = medición nueva sin leer
  }
  if (!hay_dato) return;
  t_ult_consulta_color = ahora;

  uint8_t b[8];                         // CDATAL, CDATAH, RDATAL, ... BDATAH (0x14..0x1B)
  if (!tcs_leer(TCS34725_CDATAL, b, 8)) { registrar_falla_color(); return; }
  tcs.clearInterrupt();                 // borra AINT y suelta INT: la próxima medición da un flanco nuevo
  fallas_color = 0;
  color_falla = false;

  col_c = (uint16_t)(b[0] | (b[1] << 8));
  col_r = (uint16_t)(b[2] | (b[3] << 8));
  col_g = (uint16_t)(b[4] | (b[5] << 8));
  col_b = (uint16_t)(b[6] | (b[7] << 8));
  procesar_color();
}

// Decide qué color hay bajo el sensor a partir de una medición cruda.
//  1) Normalizar por iluminación: n = canal * 1000 / C (protocolo.h). Quita el
//     efecto de la altura, del LED y de la luz ambiente.
//  2) Comparar con el blanco: rel = n * 1000 / n_blanco. Sobre blanco los tres
//     canales dan 1000; así se corrige que el sensor no ve igual los tres colores.
//  3) Un canal debe sobresalir: rel >= DOMINIO_MIN y ganarle al segundo por MARGEN_MIN.
uint8_t clasificar_color(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
  if (c >= CUENTA_MAX_COLOR) return COLOR_INDETERMINADO;              // saturado
  if ((uint32_t)c * 100u < (uint32_t)blanco_c * OSCURO_PCT) return COLOR_NINGUNO;  // negro

  const uint16_t n[3]   = { color_normalizar(r, c), color_normalizar(g, c), color_normalizar(b, c) };
  const uint16_t ref[3] = { blanco_r, blanco_g, blanco_b };
  uint32_t rel[3];
  for (uint8_t i = 0; i < 3; i++)
    rel[i] = ref[i] ? (uint32_t)n[i] * 1000u / ref[i] : 0;

  uint8_t gana = 0;
  for (uint8_t i = 1; i < 3; i++)
    if (rel[i] > rel[gana]) gana = i;
  uint32_t segundo = 0;
  for (uint8_t i = 0; i < 3; i++)
    if (i != gana && rel[i] > segundo) segundo = rel[i];

  if (rel[gana] < DOMINIO_MIN || rel[gana] - segundo < MARGEN_MIN)
    return COLOR_NINGUNO;                  // blanco, gris o un color que no es R, G ni B
  return (uint8_t)(COLOR_ROJO + gana);     // 0 -> ROJO, 1 -> VERDE, 2 -> AZUL
}

// Antirrebote y conteo. Un color se acepta tras N_CONFIRMA mediciones iguales
// seguidas, y un segmento se cuenta una sola vez: para volver a contar hay que
// salir del segmento (NINGUNO confirmado) o pasar a otro color.
void procesar_color() {
  uint8_t c = clasificar_color(col_c, col_r, col_g, col_b);
  if (c == color_candidato) {
    if (n_iguales < 255) n_iguales++;
  } else {
    color_candidato = c;
    n_iguales = 1;
  }
  if (n_iguales < N_CONFIRMA) return;

  color_actual = c;                                  // lo que ve la Pi en ST_COLOR
  if (c == COLOR_NINGUNO) { ultimo_contado = COLOR_NINGUNO; return; }
  if (c == COLOR_INDETERMINADO || c == ultimo_contado) return;
  ultimo_contado = c;

  // Solo se cuenta avanzando por la pista (no detenido ni en una acción)
  uint8_t est = estado;
  if (est != EST_RUN && est != EST_BIFURCACION) return;

  uint16_t *cuenta = (c == COLOR_ROJO) ? &count_red : (c == COLOR_VERDE) ? &count_green : &count_blue;
  if (*cuenta < 0xFFFFu) (*cuenta)++;
  Serial.print(F("[COLOR] Segmento ")); Serial.print(color_nombre(c));
  Serial.print(F(" #")); Serial.println(*cuenta);
}

// 'b' por el Monitor Serie: toma la medición actual como referencia de blanco
void calibrar_blanco() {
  if (!tcs_ok || col_c == 0) {
    Serial.println(F("[CAL] No hay una lectura válida del sensor de color."));
    return;
  }
  uint16_t r = color_normalizar(col_r, col_c);
  uint16_t g = color_normalizar(col_g, col_c);
  uint16_t b = color_normalizar(col_b, col_c);
  if (r == 0 || g == 0 || b == 0) {
    Serial.println(F("[CAL] Un canal vale 0: ¿el sensor está sobre papel blanco?"));
    return;
  }
  blanco_c = col_c; blanco_r = r; blanco_g = g; blanco_b = b;
  Serial.print(F("[CAL] Blanco tomado. Para dejarlo fijo, copie en el código: blanco_r = "));
  Serial.print(r); Serial.print(F(", blanco_g = ")); Serial.print(g);
  Serial.print(F(", blanco_b = ")); Serial.print(b);
  Serial.print(F("; blanco_c = ")); Serial.println(col_c);
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
                    | ((tcs_ok && !color_falla) ? 0 : BAN_SENSOR_COLOR_FALLA));
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

// Pasa a RUN. El PD arranca desde la posición actual de la línea.
void arrancar() {
  noInterrupts();
  ISR_Linea();                 // posición actual de la línea
  e_anterior = error_linea;    // sin salto de derivada al arrancar
  d_filtrada = 0;
  estado = EST_RUN;            // ISR_Control mueve los motores desde el siguiente tick (≤ 2 ms)
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

void set_motors(int32_t izq, int32_t der, const char* state) {
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
// int32_t: con PWM_MAX = 65535 los valores pasan de 32767 y en int16_t se volverían negativos.
void drive_motor(int pin_pwm, int pin_dir, int32_t v) {
  uint32_t m = (v >= 0) ? (uint32_t)v : (uint32_t)(-v);
  if (m > PWM_MAX) m = PWM_MAX;

  if (v >= 0) {
    digitalWrite(pin_dir, LOW);
    pwmWrite(pin_pwm, (uint16_t)m);
  } else {
    digitalWrite(pin_dir, HIGH);
    pwmWrite(pin_pwm, (uint16_t)(PWM_MAX - m));
  }
}
