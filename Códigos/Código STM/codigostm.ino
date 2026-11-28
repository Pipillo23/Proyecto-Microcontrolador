// 1 = modo de prueba sin Raspberry Pi: arranca solo al encender, no usa el watchdog
//     y no activa el I2C esclavo de la Pi. El sensor de color SÍ funciona, con la
//     librería Wire normal (solo maestro), igual que en el código original.
// PONERLO EN 0 PARA LA ENTREGA FINAL.
#define MODO_SIN_PI 1

#include <Arduino.h>
#if MODO_SIN_PI
#include <Wire.h>                // I2C solo maestro (el sensor de color no necesita más)
#else
#include <Wire_slave.h>          // I2C maestro + esclavo del núcleo Maple
#endif
#include <EEPROM.h>              // emulación de EEPROM en flash del núcleo Maple (guarda la calibración)
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
const int IR_S5 = PA9;

// Nivel que entrega el sensor cuando está SOBRE la línea
const int LINEA = HIGH;

// --- Velocidades (PWM en IN1 e IN3, 0 a PWM_MAX) ---
const uint16_t PWM_MAX  = 65535;
const uint16_t VEL_BASE = 18750;
const uint16_t VEL_CORR = 0;
const uint16_t VEL_GIRO = 0;
// Al corregir (S2/S4) el motor que sigue encendido baja a este porcentaje de la
// velocidad actual (normal, rápida o reversa). 100 = sin bajar, como antes.
const uint8_t  PCT_GIRO = 80;
// Sensores IR que tienen que ver "línea" a la vez para tomarlo como mancha ancha y no como la cinta
const uint8_t  N_MANCHA = 4;
const uint16_t VEL_RAPIDA  = VEL_BASE*1.5;   // velocidad de la acción "cambio de velocidad"
const unsigned long T_RAPIDO_MS = 3000;       // cuánto dura la velocidad rápida; después vuelve a VEL_BASE
const uint16_t VEL_REVERSA = 15000;   // velocidad del retroceso de 3 s

// --- Acciones por color y bifurcación (se temporizan con TIM3 y TIM4, por interrupción) ---
// Cada color (R, G, B) tiene una acción según cfg_color (CMD_COL_* de protocolo.h):
//   FRENAR: detenido T_FRENO_MS y sigue solo.   RETROCEDER: reversa T_REVERSA_MS y sigue.
//   VELOCIDAD: VEL_RAPIDA durante T_RAPIDO_MS y vuelve sola a VEL_BASE.
// MODO_SIN_PI 1: la configuración arranca con los valores de abajo y se cambia por el
// Monitor Serie. MODO_SIN_PI 0: la manda la Pi (hasta entonces START se rechaza).
const unsigned long T_FRENO_MS       = 10000;
const unsigned long T_REVERSA_MS     = 3000;
const unsigned long T_BIFURCACION_MS = 800;   // tiempo que se prefiere la rama elegida
const unsigned long T_MARCA_BIF_MS   = 1500;  // SENSADO: el extremo que vio línea en este lapso
const uint8_t CFG_COLOR_PRUEBA = CMD_COL_FRE_RET_VEL;   // R=Frenar G=Retroceder B=Velocidad
const uint8_t CFG_BIF_PRUEBA   = CMD_BIF_SENSADO;       // los mismos que pone la Pi por defecto

// Puente sobre los segmentos de color: los segmentos tienen el ancho de la cinta negra y
// los sensores IR van DELANTE del sensor de color, así que los IR pasan primero sobre el
// color. En el borde del tape, S2 o S4 puede ver algo un instante (el carro empieza a
// corregir) y después ningún sensor ve nada. Para no quedarse girando:
//   - Si ningún sensor ve la línea, S3 la vio hace menos de T_CENTRO_MS y desde entonces
//     ningún extremo (S1/S5) vio nada, es un segmento de color: sigue RECTO, como mucho
//     T_PUENTE_MS. Apenas S2, S3 o S4 ve la línea, la sigue normalmente.
//   - En una curva la línea se va por S1/S5 antes de perderse (o hace rato que S3 no la
//     ve): se mantiene la corrección como siempre, para volver a encontrarla.
// PUENTE_COLOR 0 = desactivado: en curvas cerradas se confundía con el tape y no corregía.
#define PUENTE_COLOR 0
const unsigned long T_PUENTE_MS = 600;
const unsigned long T_CENTRO_MS = 300;
// Recto sobre el tape: mientras el SENSOR DE COLOR ve rojo, verde o azul (cada medición,
// aunque el color no se cuente por "un color por vuelta") y hasta T_SEGMENTO_MS después,
// el carro no corrige: va recto, salvo que S3 vea la línea. Así los IR, que pasan por el
// tape un poco antes, no lo hacen girar al ver el borde del tape. 0 = desactivado.
#define RECTO_EN_COLOR 1
const unsigned long T_SEGMENTO_MS = 150;
volatile unsigned long t_ult_color_visto = 0;   // última medición clasificada como R, G o B
// Búsqueda de la línea:
//   - S1 o S5 solos también corrigen (la línea quedó muy a un lado, p. ej. al salir de un tape).
//   - Si ningún sensor ve la línea por más de T_BUSQUEDA_MS, gira hacia el lado donde la vio
//     por última vez (ultimo_lado). Si la perdió yendo centrado, sigue recto.
const unsigned long T_BUSQUEDA_MS = 400;
volatile int8_t  ultimo_lado = 0;               // -1 la vio a la izquierda, +1 a la derecha, 0 centro
volatile unsigned long t_sin_linea = 0;         // desde cuándo ningún sensor ve la línea (0 = la ve)
// 1 = el Monitor muestra cada cambio de los motores con lo que ven los 5 IR (para depurar)
#define LOG_MOTORES 1

// Retroceso: recto, los dos motores en reversa a VEL_REVERSA durante T_REVERSA_MS.
// NO sigue la línea: mientras retrocede, ISR_Linea no toca los motores (con los sensores
// adelante, corregir en reversa era inestable).

// Temporizadores de un disparo: TIM3 = fin de la acción, TIM4 = fin de la bifurcación.
// TIM2 sigue dedicado al PWM. Reloj de 72 MHz / 36000 = 2 kHz (0.5 ms), máximo 32 s.
#define TIM_ACCION TIMER3
#define TIM_BIF    TIMER4
const uint16_t TEMP_PRESCALER = 35999;
const uint32_t TEMP_TICKS_MS  = 2;

// I2C1 por hardware, ESCLAVO 0x42 hacia la Raspberry Pi (pines fijos del periférico):
//   PB6 = SCL -> GPIO3 (pin 5) de la Pi      PB7 = SDA -> GPIO2 (pin 3) de la Pi      GND común
const int SCL_RASP = PB6;
const int SDA_RASP = PB7;

// I2C2 por hardware, MAESTRO hacia el TCS34725
const int SCL_COLOR = PB10;
const int SDA_COLOR = PB11;

// USB: si al encender no hay una compu (powerbank o cargador de pared), se apaga el USB.
// Esos cargadores suelen unir D+ y D-, y el USB del STM32 queda generando interrupciones
// sin parar, que no dejan trabajar al I2C del sensor de color. Con la compu el USB queda
// como siempre (aunque el Monitor Serie se abra después). Para usar el Monitor Serie hay
// que conectar la compu ANTES de encender (o reiniciar la placa con la compu conectada).
// Cómo se sabe si hay compu: cuando una compu reconoce la placa, le asigna una dirección
// USB, que queda en el registro DADDR del USB (bits 0..6). Un cargador o una powerbank
// nunca la asignan. Se espera hasta T_ESPERA_USB_MS (Windows tarda ~1 s después del reset).
const unsigned long T_ESPERA_USB_MS = 3000;
#define USB_DADDR (*(volatile uint32_t *)0x40005C4Cu)   // registro DADDR del USB del STM32F103
// 1 = detecta la compu y, si no hay, apaga el USB (para la powerbank).
// 0 = el USB queda SIEMPRE encendido (para probar el Monitor Serie si la detección falla).
#define DETECTAR_COMPU 0
// Al encender (tarda hasta 3 s sin compu), el LED dice qué se decidió: 3 parpadeos cortos = compu detectada (USB
// encendido); 1 parpadeo largo = sin compu (USB apagado).
bool hay_compu = false;

// Todos los mensajes pasan por Consola: sin compu no se toca el Serial (con el USB apagado,
// Serial.print podría quedarse esperando para siempre). Junta los caracteres y los envía
// por línea, para no mandar un paquete USB por cada letra.
class ConsolaSegura : public Print {
public:
  size_t write(uint8_t c) {
    if (!hay_compu) return 1;
    buf[n++] = c;
    if (c == '\n' || n >= sizeof(buf)) enviar();
    return 1;
  }
  int available() { return hay_compu ? Serial.available() : 0; }
  int read()      { return hay_compu ? Serial.read() : -1; }
private:
  uint8_t buf[64];
  uint8_t n = 0;
  void enviar() { if (n) { Serial.write((const uint8 *)buf, (uint32)n); n = 0; } }
};
ConsolaSegura Consola;

// LED de la placa (activo en bajo): encendido mientras corre setup(). Si se queda
// encendido, el programa se trabó en la inicialización.
const int LED_PLACA = PC13;

// --- Sensor de color ---
// Integración de 24 ms (antes 50 ms). Un segmento de 5 cm a 0.5 m/s pasa bajo el
// sensor en 100 ms: caben ~4 mediciones y el color se confirma con 2 seguidas.
// Máximo de cuenta con 24 ms: (256 - 0xF6) * 1024 = 10240.
//
// El sensor se maneja con escrituras/lecturas I2C directas, SIN la librería de Adafruit:
// su begin() (con Adafruit_BusIO) se quedaba trabado con la Wire del núcleo Maple, que
// no tiene tiempo límite. Las mismas transferencias hechas a mano sí funcionan.
#define TCS34725_ADDRESS               0x29
#define TCS34725_COMMAND_BIT           0x80
#define TCS34725_AUTOINCREMENTO        0x20
#define TCS34725_LIMPIAR_INT           0xE6   // comando especial: borra la interrupción (AINT)
#define TCS34725_ENABLE                0x00
#define TCS34725_ENABLE_PON            0x01   // encendido del oscilador interno
#define TCS34725_ENABLE_AEN            0x02   // mediciones RGBC
#define TCS34725_ENABLE_AIEN           0x10   // interrupción en el pin INT
#define TCS34725_ATIME                 0x01
#define TCS34725_PERS                  0x0C
#define TCS34725_PERS_NONE             0x00   // interrupción en cada fin de medición
#define TCS34725_CONTROL               0x0F
#define TCS34725_GAIN_4X               0x01
#define TCS34725_ID                    0x12   // 0x44 (TCS34721/5) o 0x4D (TCS34723/7)
#define TCS34725_STATUS                0x13
#define TCS34725_STATUS_AINT           0x10
#define TCS34725_CDATAL                0x14
#define TCS34725_INTEGRATIONTIME_24MS  0xF6
#define COLOR_T_INTEGRACION TCS34725_INTEGRATIONTIME_24MS
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
//   0 = POR TIEMPO: sin cable INT. La interrupción de TIM1 (cada T_TICK_MS)
//       levanta una bandera y loop consulta el bit AINT del registro STATUS; si hay
//       medición nueva, la lee. Así no se lee dos veces la misma medición.
// TIM1 corre SIEMPRE como base de tiempo de 5 ms (ISR_Tick): además del sondeo del color,
// termina la velocidad rápida y el puente, porque no queda otro temporizador libre.
#define COLOR_POR_INTERRUPCION 0
const int COLOR_INT = PA8;       // INT del TCS34725 (solo con COLOR_POR_INTERRUPCION 1): open-drain,
                                 // activo en bajo. EXTI8 no choca con los IR (EXTI 0, 9, 12, 13 y 14)
const unsigned long T_RESPALDO_COLOR_MS = 100;
const unsigned long T_TICK_MS           = 5;     // período de TIM1 (base de tiempo)
#define TIM_TICK TIMER1                          // TIM1 (TIM2 PWM, TIM3 acciones, TIM4 bifurcación)
volatile bool color_listo = false;               // la levanta ISR_Color
volatile bool color_consultar = false;           // la levanta ISR_Tick (TIM1)
bool tcs_ok = false;
bool color_falla = false;                        // 5 lecturas I2C fallidas seguidas
uint8_t fallas_color = 0;
uint16_t col_r = 0, col_g = 0, col_b = 0, col_c = 0;   // última lectura cruda
unsigned long t_ult_consulta_color = 0;
// Recuperación del sensor: si deja de entregar mediciones, loop lo vuelve a configurar
const unsigned long T_SIN_DATO_COLOR_MS  = 300;  // sin medición nueva por este tiempo -> reiniciar
const unsigned long T_REINTENTO_COLOR_MS = 500;  // entre intentos de reconexión
unsigned long t_ult_dato_color     = 0;          // última medición leída
unsigned long t_ult_reintento_color = 0;
uint16_t      n_reinicios_color    = 0;          // veces que hubo que reiniciar el sensor

// LED de la placa como indicador cuando no hay Monitor Serie (con la powerbank):
//   parpadeo lento (1 por s)   = todo bien, el sensor de color está midiendo
//   encendido 0.3 s seguidos   = se contó un segmento de color
//   fijo, sin parpadear        = el programa se trabó (loop detenido)
//   GRUPOS de parpadeos cortos y una pausa de 1.5 s = el sensor no mide. La cantidad
//   de parpadeos de cada grupo dice por qué (causa_color):
//     1 = SCL/SDA sin pull-up: el módulo del sensor no tiene alimentación o un cable está suelto
//     2 = hay pull-ups pero el sensor no contesta por I2C (dirección, SDA/SCL cruzados, sensor reiniciándose)
//     3 = contesta, pero no entrega mediciones nuevas o las lecturas fallan
const uint8_t CAUSA_NINGUNA = 0, CAUSA_SIN_PULLUPS = 1, CAUSA_NO_RESPONDE = 2, CAUSA_SIN_MEDICIONES = 3;
uint8_t causa_color = CAUSA_NINGUNA;             // se borra recién cuando llega una medición
const unsigned long T_LED_COLOR_MS = 300;
unsigned long t_led_color = 0;                   // momento en que se contó el último color

// --- Calibración del sensor de color (se hace una vez y queda guardada en la flash) ---
// Al encender, el STM32 busca la calibración guardada en la flash. Si la encuentra (y es
// válida) la usa y no pide nada. Si no hay ninguna, arranca en EST_CALIBRACION y pide,
// en orden, NEGRO, BLANCO, AZUL, ROJO y VERDE (cal_color_paso() en protocolo.h). En cada
// paso promedia N_MUESTRAS_CAL mediciones y las guarda como referencia. Al terminar los
// 5 pasos, la calibración se graba en la flash y sirve para los próximos encendidos.
// Para cambiarla: conectar el STM32 a la compu por USB, abrir el Monitor Serie (115200)
// y enviar 'c'. Después 'm' en cada paso. Si se desconecta antes de terminar los 5
// pasos, la calibración anterior sigue guardada y se vuelve a cargar al encender.
// Quién indica cuándo medir: 'm' por el Monitor Serie (en los dos modos); con
// MODO_SIN_PI 0 también la interfaz de la Pi (CMD_CAL_MEDIR, CMD_CAL_REINICIAR).
// Después, cada medición se clasifica como el color de referencia más parecido.
const uint8_t  N_MUESTRAS_CAL     = 8;    // ~200 ms con 24 ms de integración
const uint8_t  N_CONFIRMA         = 2;    // mediciones seguidas iguales para aceptar un color
const uint16_t MARGEN_AMBIGUO_PCT = 150;  // la 2ª referencia debe estar >= 1.5 veces más lejos
const uint16_t N_MAX              = 2000; // tope de los valores normalizados (por mil de C)
const uint16_t BRILLO_MAX         = 4000; // tope del brillo relativo al blanco (por mil)
uint8_t  cal_paso      = 0;               // 0..CAL_PASOS-1
uint8_t  cal_restantes = 0;               // > 0: tomando las muestras del paso actual
uint32_t cal_suma[4]   = {0, 0, 0, 0};    // C crudo y r, g, b normalizados
uint16_t ref_c[CAL_PASOS];                // C promedio de cada color de referencia
uint16_t ref_n[CAL_PASOS][3];             // r, g, b normalizados (por mil de C)
bool     cal_completa  = false;
bool     cal_valida    = false;           // hay referencias (cargadas o recién calibradas)
unsigned long t_fin_cal = 0;
uint8_t color_candidato = COLOR_NINGUNO, n_iguales = 0;
uint8_t ultimo_contado  = COLOR_NINGUNO; // evita contar dos veces el mismo segmento

// --- Calibración guardada en la flash (librería EEPROM del núcleo Maple) ---
// Usa las 2 últimas páginas de 1 KB de los 64 KB de la C8T6 (0x0800F800 y 0x0800FC00).
// El programa debe ocupar menos de 62 KB (el IDE muestra el tamaño al compilar).
// Volver a subir el programa NO borra la calibración, salvo que crezca hasta esas páginas.
// Cada dato es un uint16 con una "dirección virtual"; se guarda también un número mágico,
// la versión del formato y una suma de verificación: si algo no coincide, se recalibra.
const uint32_t FLASH_CAL_PAG0  = 0x0800F800;
const uint32_t FLASH_CAL_PAG1  = 0x0800FC00;
const uint32_t FLASH_CAL_TAM   = 0x400;                    // página de 1 KB
const uint16_t CAL_MAGICO      = 0xCA1B;
const uint16_t CAL_VERSION     = 0x0100 | CAL_PASOS;       // si cambia el formato, cambiarla
const uint16_t DIR_CAL_MAGICO  = 0x10;
const uint16_t DIR_CAL_VERSION = 0x11;
const uint16_t DIR_CAL_DATOS   = 0x12;                     // ref_c[0..4] y luego ref_n[0..4][0..2]
const uint16_t N_DATOS_CAL     = CAL_PASOS * 4;
const uint16_t DIR_CAL_SUMA    = DIR_CAL_DATOS + N_DATOS_CAL;
bool flash_ok = false;                                     // EEPROM.init() funcionó

// --- Estado del carro según protocolo.h (lo maneja loop) ---
volatile uint8_t estado = EST_IDLE;             // EST_*; lo lee también ISR_Linea
uint8_t  banderas_fijas = 0;                    // BAN_COM_PERDIDA (se borra con START)
#if MODO_SIN_PI
bool     arranco   = false;                     // ya hizo el arranque automático
uint8_t  cfg_color = CFG_COLOR_PRUEBA;          // modo de prueba: ya configurado
uint8_t  cfg_bif   = CFG_BIF_PRUEBA;
#else
uint8_t  cfg_color = 0;                         // CMD_COL_* activo (0 = sin configurar)
uint8_t  cfg_bif   = 0;                         // CMD_BIF_* activo (0 = sin configurar)
#endif

// --- Acciones y bifurcación (las usan ISR_Linea y las ISR de los temporizadores) ---
volatile uint16_t vel_avance = VEL_BASE;        // VEL_BASE o VEL_RAPIDA
volatile int8_t  lado_bif = 0;                  // rama elegida: -1 izquierda, +1 derecha
volatile bool    bif_armada = true;             // false hasta que los extremos dejan de ver línea
volatile int8_t  ultimo_extremo = 0;            // SENSADO: -1 S1 vio línea solo, +1 S5, 0 ninguno
volatile unsigned long t_ultimo_extremo = 0;
volatile bool    fin_reversa = false;           // lo levanta ISR_FinAccion; lo atiende loop
volatile uint16_t ticks_rapido = 0;             // > 0: velocidad rápida; ISR_Tick la descuenta
volatile bool    fin_rapido = false;            // lo levanta ISR_Tick; loop lo avisa por serial
volatile bool    puente = false;                // cruzando un segmento de color sin ver la línea
volatile unsigned long t_puente = 0;
volatile unsigned long t_ult_s3 = 0;            // última vez que S3 vio la línea
volatile bool    ext_desde_centro = false;      // S1 o S5 vieron línea después de S3
uint8_t ignorar_segmentos = 0;                  // segmentos que se vuelven a pasar tras retroceder

// --- Un color por vuelta ---
// Un color ya hecho no se vuelve a ejecutar hasta que en la vuelta se hayan hecho los 3
// (rojo, verde y azul, en cualquier orden); entonces empieza una vuelta nueva.
// CONTAR_REPETIDOS false: el color repetido tampoco se cuenta (se toma como lectura falsa).
// CONTAR_REPETIDOS true : se cuenta, pero no se ejecuta su acción.
const bool CONTAR_REPETIDOS = false;
uint8_t colores_vuelta = 0;                     // bit 0 = ROJO, bit 1 = VERDE, bit 2 = AZUL
const uint8_t TODOS_LOS_COLORES = 0x07;
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
volatile int32_t vel_izq = 0, vel_der = 0;   // + adelante, - reversa (int32: VEL_RAPIDA no cabe en int16)
volatile const char* motor_state = "DETENIDO";

// Prototipos
void ISR_Linea();
void ISR_Color();
void ISR_Tick();
void temporizador_periodico(timer_dev *t, uint32_t ms, voidFuncPtr isr);
void atender_sensor_color();
bool tcs_conectar();
bool color_sin_datos();
const __FlashStringHelper* causa_nombre(uint8_t c);
void actualizar_led();
bool tcs_leer(uint8_t reg, uint8_t *buf, uint8_t n);
bool tcs_escribir(uint8_t reg, uint8_t valor);
bool tcs_limpiar_interrupcion();
bool tcs_iniciar();
void registrar_falla_color();
uint8_t clasificar_color(uint16_t c, uint16_t r, uint16_t g, uint16_t b);
void procesar_color();
void segmento_nuevo(uint8_t c);
void iniciar_accion(uint8_t acc);
void reanudar();
void ISR_FinAccion();
void ISR_FinBifurcacion();
int8_t elegir_lado_bif();
void imprimir_config();
void temporizador_config(timer_dev *t, voidFuncPtr isr);
void temporizador_iniciar(timer_dev *t, uint32_t ms);
void temporizador_parar(timer_dev *t);
void cal_iniciar();
bool cal_pedir_medicion();
void cal_muestra();
void cal_avisar_paso();
uint16_t* cal_dato(uint16_t i);
uint16_t cal_suma_verif();
bool flash_escribir(uint16_t dir, uint16_t valor);
bool cal_guardar();
bool cal_cargar();
void cal_imprimir_referencias();
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

  // ¿Hay una compu en el USB? (los motores ya están detenidos mientras se espera)
  unsigned long t_usb = millis();
  while ((USB_DADDR & 0x7Fu) == 0 && millis() - t_usb < T_ESPERA_USB_MS) { }
  hay_compu = (USB_DADDR & 0x7Fu) != 0;   // la compu le asignó una dirección a la placa
#if !DETECTAR_COMPU
  hay_compu = true;                      // USB siempre encendido
#endif
  // Aviso con el LED (activo en bajo): 3 cortos = compu, 1 largo = sin compu
  digitalWrite(LED_PLACA, HIGH); delay(300);
  if (hay_compu) {
    for (uint8_t i = 0; i < 3; i++) { digitalWrite(LED_PLACA, LOW); delay(120); digitalWrite(LED_PLACA, HIGH); delay(120); }
  } else {
    digitalWrite(LED_PLACA, LOW); delay(800); digitalWrite(LED_PLACA, HIGH);
  }
  delay(300);
  digitalWrite(LED_PLACA, LOW);          // vuelve a encendido: sigue el setup()
  if (hay_compu) {
    Consola.println(F("[INFO] Compu detectada por USB: Monitor Serie activo."));
  } else {
    // Sin compu: se apagan las interrupciones del USB (no se llama a Serial.end())
    nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
    nvic_irq_disable(NVIC_USB_HP_CAN_TX);
  }

  // Sensor de color TCS34725 en I2C2 (PB10/PB11), como maestro
  // Sin pull-ups en el bus, el primer begin() del I2C se queda trabado en el reset del
  // bus del núcleo Maple (i2c_bus_reset). Por eso se revisan las líneas antes.
  // Si no responde al encender (por ejemplo, la powerbank tarda en dar tensión estable),
  // loop lo vuelve a intentar cada T_REINTENTO_COLOR_MS (ver atender_sensor_color).
  tcs_ok = tcs_conectar();         // ID, 24 ms, ganancia 4x, INT en cada medición
  t_ult_dato_color = millis();
  if (tcs_ok) Consola.println(F("[INFO] Sensor TCS34725 detectado correctamente."));
  else        Consola.println(F("[ERROR] No se encontró el sensor TCS34725: se reintenta cada 0.5 s. Revise las conexiones I2C."));
#if COLOR_POR_INTERRUPCION
  pinMode(COLOR_INT, INPUT_PULLUP);                // INT es open-drain: necesita pull-up
  attachInterrupt(COLOR_INT, ISR_Color, FALLING);
  Consola.println(F("[INFO] Color por interrupción (INT del TCS34725 en PA8)."));
#else
  Consola.print(F("[INFO] Color por tiempo: TIM1 consulta el sensor cada "));
  Consola.print(T_TICK_MS); Consola.println(F(" ms."));
#endif
  temporizador_periodico(TIM_TICK, T_TICK_MS, ISR_Tick);   // base de tiempo de 5 ms
  // Memoria flash para la calibración (emulación de EEPROM en las 2 últimas páginas)
  EEPROM.PageBase0 = FLASH_CAL_PAG0;
  EEPROM.PageBase1 = FLASH_CAL_PAG1;
  EEPROM.PageSize  = FLASH_CAL_TAM;
  flash_ok = (EEPROM.init() == EEPROM_OK);

  // Calibración del color: se carga la guardada (aunque el sensor todavía no haya
  // respondido: se usará cuando se recupere). Si no hay, se calibra y se guarda al final.
  if (cal_cargar()) {
    cal_valida = true;
    cal_completa = true;
    t_fin_cal = millis();
    estado = EST_IDLE;
    Consola.println(F("[CAL] Calibración cargada de la flash:"));
    cal_imprimir_referencias();
    Consola.println(F("[CAL] Para calibrar de nuevo envíe 'c'; para borrarla, 'b' (Monitor Serie)."));
  } else if (tcs_ok) {
    if (flash_ok) Consola.println(F("[CAL] No hay calibración guardada: hay que calibrar."));
    else          Consola.println(F("[CAL] ERROR: no se pudo iniciar la flash; la calibración no se guardará."));
    cal_iniciar();
  } else {
    cal_completa = true;
    t_fin_cal = millis();
    Consola.println(F("[AVISO] Sin sensor y sin calibración guardada: no se detectarán colores."));
  }

#if MODO_SIN_PI
  // Modo de prueba: el I2C esclavo de la Pi no se activa
  Consola.println(F("[AVISO] MODO_SIN_PI: I2C de la Raspberry Pi deshabilitado."));
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
    Consola.println(F("[INFO] Bus I2C de la Pi detectado: esclavo 0x42 activo."));
  } else {
    Consola.println(F("[AVISO] Sin pull-ups en PB6/PB7 (Pi desconectada): I2C esclavo deshabilitado."));
  }
#endif

  // Temporizadores de un disparo para las acciones (freno/retroceso) y la bifurcación
  temporizador_config(TIM_ACCION, ISR_FinAccion);
  temporizador_config(TIM_BIF,    ISR_FinBifurcacion);
#if MODO_SIN_PI
  imprimir_config();
#endif

  // --- Interrupciones externas por cambio en los pines del seguidor ---
  // Cada sensor usa su propia línea EXTI (14, 0, 12, 13 y 9), así que no chocan.
  // CHANGE: entra tanto cuando un sensor ve la línea como cuando la deja de ver.
  // Timer2 queda dedicado al PWM de los motores.
  attachInterrupt(IR_S1, ISR_Linea, CHANGE);
  attachInterrupt(IR_S2, ISR_Linea, CHANGE);
  attachInterrupt(IR_S3, ISR_Linea, CHANGE);
  attachInterrupt(IR_S4, ISR_Linea, CHANGE);
  attachInterrupt(IR_S5, ISR_Linea, CHANGE);

  digitalWrite(LED_PLACA, HIGH);   // LED apagado: setup() terminó
  Consola.println(F("[INFO] Sistema inicializado."));
}

void loop() {
  #if MODO_SIN_PI
  // Arranque automático, una sola vez, T_ARRANQUE_MS después de terminar (o cargar) la calibración
  if (!arranco && cal_completa && estado == EST_IDLE && millis() - t_fin_cal >= T_ARRANQUE_MS) {
    arranco = true;
    arrancar();
  }
#endif
  // Monitor Serie (USB, 115200). En los dos modos:
  //   'c' vuelve a calibrar desde NEGRO (detiene el carro). La nueva se guarda en la
  //       flash al terminar los 5 pasos; si se apaga antes, sigue la anterior.
  //   'm' toma la medición del paso de calibración
  //   'b' BORRA la calibración guardada y empieza a calibrar (si se apaga sin terminar,
  //       al encender pedirá calibrar otra vez)
  //   'v' muestra la calibración en uso
  // Solo con MODO_SIN_PI 1:
  //   's' arranca, 'x' detiene
  //   '1'..'6' acciones de los colores (CMD_COL_FRE_RET_VEL ... CMD_COL_VEL_RET_FRE)
  //   'i' bifurcación a la izquierda, 'd' a la derecha, 'n' por sensado (solo detenido)
  //   'R', 'G', 'B' simulan pasar sobre un segmento de ese color (prueba sin pista)
  //   'p' muestra la configuración
  if (Consola.available()) {
    char c = Consola.read();
    if (c == 'm') {
      if (!cal_pedir_medicion())
        Consola.println(F("[CAL] No se puede medir ahora (no está calibrando o ya está midiendo)."));
    }
    else if (c == 'c') {
      if (!tcs_ok) Consola.println(F("[CAL] Sin sensor de color: no se puede calibrar."));
      else {
        detener(EST_IDLE);
#if MODO_SIN_PI
        arranco = false;           // al terminar la calibración vuelve a arrancar solo
#endif
        cal_iniciar();
      }
    }
    else if (c == 'b') {
      // Borra la calibración guardada (anula el número mágico) y empieza a calibrar
      if (!flash_ok || !flash_escribir(DIR_CAL_MAGICO, 0))
        Consola.println(F("[CAL] ERROR: no se pudo borrar la calibración de la flash."));
      else {
        Consola.println(F("[CAL] Calibración guardada BORRADA."));
        if (!tcs_ok) Consola.println(F("[CAL] Sin sensor de color: no se puede calibrar ahora."));
        else {
          detener(EST_IDLE);
#if MODO_SIN_PI
          arranco = false;
#endif
          cal_iniciar();
        }
      }
    }
    else if (c == 'v') {
      if (estado == EST_CALIBRACION) cal_avisar_paso();
      else if (!tcs_ok) Consola.println(F("[CAL] Sin sensor de color."));
      else { Consola.println(F("[CAL] Calibración en uso:")); cal_imprimir_referencias(); }
    }
#if MODO_SIN_PI
    else {
    bool detenido = (estado == EST_IDLE || estado == EST_CALIBRACION);
    if (c == 's' && estado == EST_IDLE) arrancar();
    else if (c == 'x' && estado != EST_CALIBRACION) detener(EST_IDLE);
    else if ((c >= '1' && c <= '6') || c == 'i' || c == 'd' || c == 'n') {
      if (!detenido) Consola.println(F("[CONFIG] ERROR: solo se puede configurar con el carro detenido ('x')."));
      else {
        if (c >= '1' && c <= '6') cfg_color = (uint8_t)(CMD_COL_PRIMERO + (c - '1'));
        else cfg_bif = (c == 'i') ? CMD_BIF_IZQUIERDA : (c == 'd') ? CMD_BIF_DERECHA : CMD_BIF_SENSADO;
        c = 'p';
      }
    }
    else if (c == 'R' || c == 'G' || c == 'B') {
      Consola.print(F("[PRUEBA] Segmento simulado: ")); Consola.println(c);
      segmento_nuevo(c == 'R' ? COLOR_ROJO : c == 'G' ? COLOR_VERDE : COLOR_AZUL);
    }
    if (c == 'p') imprimir_config();
    }
#endif
  }

  // Terminó un retroceso. Si el carro quedó dentro de un segmento, al avanzar ya no
  // "entra" a él, así que hay uno menos que ignorar (ver segmento_nuevo)
  if (fin_reversa) {
    fin_reversa = false;
    if (ultimo_contado != COLOR_NINGUNO && ignorar_segmentos) ignorar_segmentos--;
  }

#if LOG_MOTORES
  // Cada cambio de los motores, con lo que ven los IR en ese momento (S1..S5, 1 = línea)
  {
    static const char* st_ant = 0;
    const char* st_ahora = (const char*)motor_state;
    if (st_ahora != st_ant) {
      st_ant = st_ahora;
      Consola.print(F("[MOTOR] ")); Consola.print(millis()); Consola.print(F(" ms -> "));
      Consola.print(st_ahora);
      Consola.print(F(" | IR "));
      Consola.print(digitalRead(IR_S1) == LINEA ? 1 : 0); Consola.print(digitalRead(IR_S2) == LINEA ? 1 : 0);
      Consola.print(digitalRead(IR_S3) == LINEA ? 1 : 0); Consola.print(digitalRead(IR_S4) == LINEA ? 1 : 0);
      Consola.print(digitalRead(IR_S5) == LINEA ? 1 : 0);
      Consola.print(F(" | color: ")); Consola.println(color_nombre(color_actual));
    }
  }
#endif
  if (fin_rapido) {
    fin_rapido = false;
    Consola.println(F("[ACCION] Fin de la velocidad rápida: vuelve a NORMAL"));
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
    Consola.println(F("[WATCHDOG] Sin comandos de la Pi por 500 ms: carro detenido."));
  }

  // Avisar por serial apenas cambia el estado (una intersección puede durar menos de 200 ms)
  static uint8_t estado_anterior = 0xFF;
  uint8_t est = estado;
  if (est != estado_anterior) {
    estado_anterior = est;
    Consola.print(F("[ESTADO] "));
    Consola.print(millis());
    Consola.print(F(" ms -> "));
    Consola.println(estado_nombre(est));
  }

  // 3) Sensor de color: si hay medición nueva, leer, normalizar, clasificar y contar
  atender_sensor_color();

  // LED de la placa: indicador para probar con la powerbank, sin Monitor Serie
  actualizar_led();

  // 4) Refrescar la trama de estado cada 10 ms
  if (millis() - t_ult_publicacion >= T_PUBLICACION_STM_MS) {
    t_ult_publicacion = millis();
    publicar_estado();
  }

  // 5) Depuración por USB cada 200 ms
  if (millis() - last_serial_print >= 200) {
    last_serial_print = millis();

    // Calibrando: en vez de todo el bloque, repetir cada 1 s qué color hay que medir
    if (estado == EST_CALIBRACION) {
      static uint8_t n_aviso = 0;
      if (++n_aviso >= 5) { n_aviso = 0; cal_avisar_paso(); }
      return;                      // este bloque es lo último del loop
    }

    // Leer estado actual de los sensores IR
    int s1 = digitalRead(IR_S1);
    int s2 = digitalRead(IR_S2);
    int s3 = digitalRead(IR_S3);
    int s4 = digitalRead(IR_S4);
    int s5 = digitalRead(IR_S5);

    // Mostrar datos legibles en el Monitor Serial
    Consola.print(F("[SENSORES IR] S1: ")); Consola.print(s1);
    Consola.print(F(" | S2: ")); Consola.print(s2);
    Consola.print(F(" | S3 (Centro): ")); Consola.print(s3);
    Consola.print(F(" | S4: ")); Consola.print(s4);
    Consola.print(F(" | S5: ")); Consola.print(s5);
    Consola.println();

    // Sensor de color: crudo, normalizado (por mil de C) y color confirmado
    Consola.print(F("[COLOR] C: ")); Consola.print(col_c);
    Consola.print(F(" R: ")); Consola.print(col_r);
    Consola.print(F(" G: ")); Consola.print(col_g);
    Consola.print(F(" B: ")); Consola.print(col_b);
    Consola.print(F(" | norm r: ")); Consola.print(color_normalizar(col_r, col_c));
    Consola.print(F(" g: ")); Consola.print(color_normalizar(col_g, col_c));
    Consola.print(F(" b: ")); Consola.print(color_normalizar(col_b, col_c));
    Consola.print(F(" | color: ")); Consola.print(color_nombre(color_actual));
    Consola.print(F(" | conteo R/G/B: ")); Consola.print(count_red);
    Consola.print(F("/")); Consola.print(count_green);
    Consola.print(F("/")); Consola.print(count_blue);
    if (color_sin_datos()) { Consola.print(F(" | SENSOR SIN DATOS: ")); Consola.print(causa_nombre(causa_color)); }
    Consola.print(F(" | reinicios: ")); Consola.print(n_reinicios_color);
    Consola.println();

    // Copia consistente del estado de salidas
    noInterrupts();
    int16_t vi = vel_izq, vd = vel_der;
    const char* st = (const char*)motor_state;
    interrupts();

    Consola.print(F("[SALIDAS] Izq (IN1): ")); Consola.print(vi);
    Consola.print(F("/")); Consola.print(PWM_MAX);
    Consola.print(F(" | Der (IN3): ")); Consola.print(vd);
    Consola.print(F("/")); Consola.print(PWM_MAX);
    Consola.print(F(" || Motores: ")); Consola.print(st);
    Consola.print(F(" | Estado: ")); Consola.println(estado_nombre(estado));

    Consola.print(F("[PROTOCOLO] ult_seq: ")); Consola.print(ult_seq);
    Consola.print(F(" (")); Consola.print(resultado_nombre(ult_resultado));
    Consola.print(F(") | colores: ")); Consola.print(cmd_nombre(cfg_color));
    Consola.print(F(" | bif: ")); Consola.print(cmd_nombre(cfg_bif));
    Consola.print(F(" | ms desde la ultima trama: ")); Consola.println(millis() - t_ult_trama);
  }
}

// Interrupción externa por cambio en cualquiera de los 5 sensores de línea
// Disposición: S1 (izq. extremo) - S2 - S3 (centro) - S4 - S5 (der. extremo)
// Motor A = izquierdo (IN1/IN2), Motor B = derecho (IN3/IN4)
// Seguimiento: solo S2, S3 y S4. Los extremos S1 y S5 solo detectan intersecciones.
// Correcciones (S2/S4): bajan la velocidad de un lado
// También la llaman (con interrupciones apagadas) arrancar, reanudar e iniciar_accion,
// y las ISR de los temporizadores, para recalcular la salida sin esperar un cambio de pin.
void ISR_Linea() {
  uint8_t est = estado;
  // Solo sigue la línea avanzando. En el retroceso de 3 s (EST_ACT_REVERSA) no hace nada:
  // los motores quedaron en reversa recta en iniciar_accion y los sensores no los cambian.
  if (est != EST_RUN && est != EST_BIFURCACION) return;

  int s1 = digitalRead(IR_S1);
  int s2 = digitalRead(IR_S2);
  int s3 = digitalRead(IR_S3);
  int s4 = digitalRead(IR_S4);
  int s5 = digitalRead(IR_S5);

  // SENSADO: recordar qué extremo vio la línea solo (marca antes de la bifurcación)
  if (s1 == LINEA && s5 != LINEA)      { ultimo_extremo = -1; t_ultimo_extremo = millis(); }
  else if (s5 == LINEA && s1 != LINEA) { ultimo_extremo = +1; t_ultimo_extremo = millis(); }
  if (s1 != LINEA || s5 != LINEA) bif_armada = true;

  // Para el puente: cuándo se vio la línea al centro y si después pasó por un extremo
  if (s3 == LINEA) { t_ult_s3 = millis(); ext_desde_centro = false; }
  if (s1 == LINEA || s5 == LINEA) ext_desde_centro = true;

  // Mancha ancha: la cinta negra tapa como mucho 2 sensores vecinos. Si N_MANCHA o más
  // ven "línea" a la vez, es algo más ancho (p. ej. un segmento de color que el IR ve
  // oscuro): no se corrige ni se detecta bifurcación, se sigue recto.
  uint8_t n_activos = (s1 == LINEA) + (s2 == LINEA) + (s3 == LINEA) + (s4 == LINEA) + (s5 == LINEA);
  if (n_activos >= N_MANCHA) {
    set_motors(vel_avance, vel_avance, "MANCHA_RECTO");
    return;
  }

  const uint16_t v  = vel_avance;
  const uint16_t vg = (uint16_t)((uint32_t)v * PCT_GIRO / 100u);   // motor encendido al corregir

  // Puente sobre un segmento de color (ver T_PUENTE_MS)
#if PUENTE_COLOR
  if (est == EST_RUN) {
    if (n_activos == 0 && !puente && !ext_desde_centro && millis() - t_ult_s3 < T_CENTRO_MS) {
      puente = true;                    // se perdió todo poco después de ir centrado: segmento de color
      t_puente = millis();
    }
    if (puente) {
      if (s2 == LINEA || s3 == LINEA || s4 == LINEA) {
        puente = false;                 // un sensor del medio volvió a ver la línea: seguirla ya
      } else if (millis() - t_puente >= T_PUENTE_MS) {
        puente = false;                 // se acabó el tiempo sin línea (no vuelve a entrar:
                                        // S3 no la ve hace más de T_CENTRO_MS)
      } else {
        set_motors(v, v, "PUENTE_COLOR");
        return;
      }
    }
  }
#endif

#if RECTO_EN_COLOR
  // Sobre un tape de color (lo dice el sensor de color): recto, sin corregir
  if (est == EST_RUN && s3 != LINEA && t_ult_color_visto != 0 &&
      millis() - t_ult_color_visto < T_SEGMENTO_MS) {
    set_motors(v, v, "RECTO_COLOR");
    return;
  }
#endif

  // Bifurcación: los dos extremos ven línea a la vez y el centro NO (en una "Y" las dos
  // ramas ya están separadas). Se elige la rama (cfg_bif) y durante T_BIFURCACION_MS se
  // sigue esa rama; ISR_FinBifurcacion vuelve a RUN.
  if (est == EST_RUN && bif_armada && s1 == LINEA && s5 == LINEA && s3 != LINEA) {
    bif_armada = false;                 // no volver a detectar la misma bifurcación
    lado_bif = elegir_lado_bif();
    est = EST_BIFURCACION;
    estado = est;
    temporizador_iniciar(TIM_BIF, T_BIFURCACION_MS);
  }

  if (est == EST_BIFURCACION) {
    // La rama elegida tiene prioridad: se gira hacia el primer sensor de ese lado que vea línea
    bool lado_elegido = (lado_bif < 0) ? (s1 == LINEA || s2 == LINEA) : (s4 == LINEA || s5 == LINEA);
    bool lado_otro    = (lado_bif < 0) ? (s4 == LINEA || s5 == LINEA) : (s1 == LINEA || s2 == LINEA);
    if (lado_elegido) {
      if (lado_bif < 0) set_motors(VEL_CORR, vg, "BIF_IZQ");
      else              set_motors(vg, VEL_CORR, "BIF_DER");
    }
    else if (s3 == LINEA) set_motors(v, v, "AVANZA");
    else if (lado_otro) {
      if (lado_bif < 0) set_motors(vg, VEL_CORR, "CORRIGE_DER");
      else              set_motors(VEL_CORR, vg, "CORRIGE_IZQ");
    }
    return;
  }

  if (n_activos) t_sin_linea = 0;
  if (s3 == LINEA) {
    // Centrado: ambos a la velocidad de avance (VEL_BASE o VEL_RAPIDA)
    ultimo_lado = 0;
    set_motors(v, v, "AVANZA");
  }
  else if (s2 == LINEA) {
    // Línea un poco a la izquierda: gira a la izquierda
    ultimo_lado = -1;
    set_motors(VEL_CORR, vg, "CORRIGE_IZQ");
  }
  else if (s4 == LINEA) {
    // Línea un poco a la derecha: gira a la derecha
    ultimo_lado = +1;
    set_motors(vg, VEL_CORR, "CORRIGE_DER");
  }
  else if (s1 == LINEA) {
    // Línea muy a la izquierda (solo S1): gira a la izquierda
    ultimo_lado = -1;
    set_motors(VEL_CORR, vg, "EXTREMO_IZQ");
  }
  else if (s5 == LINEA) {
    // Línea muy a la derecha (solo S5): gira a la derecha
    ultimo_lado = +1;
    set_motors(vg, VEL_CORR, "EXTREMO_DER");
  }
  else {
    // Ningún sensor ve la línea
    unsigned long ahora = millis();
    if (t_sin_linea == 0) t_sin_linea = ahora ? ahora : 1;
    if (ahora - t_sin_linea >= T_BUSQUEDA_MS && ultimo_lado != 0) {
      // Hace rato que no la ve: la busca girando hacia donde la vio por última vez
      if (ultimo_lado < 0) set_motors(VEL_CORR, vg, "BUSCA_IZQ");
      else                 set_motors(vg, VEL_CORR, "BUSCA_DER");
    }
    else if (vel_izq > 0 && vel_der > 0 && vel_izq == vel_der) {
      // Iba recto: sigue recto, pero con la velocidad actual (p. ej. al terminar la rápida)
      set_motors(v, v, (const char*)motor_state);
    }
    // Si iba corrigiendo, mantiene la corrección (curva)
  }
}

// Muestra qué acción tiene cada color y cómo se resuelve la bifurcación
void imprimir_config() {
  Consola.print(F("[CONFIG] R: ")); Consola.print(accion_nombre(color_accion(cfg_color, COLOR_ROJO)));
  Consola.print(F(" | G: ")); Consola.print(accion_nombre(color_accion(cfg_color, COLOR_VERDE)));
  Consola.print(F(" | B: ")); Consola.print(accion_nombre(color_accion(cfg_color, COLOR_AZUL)));
  Consola.print(F(" | bifurcación: ")); Consola.println(bif_nombre(cfg_bif));
}

// Lado de la bifurcación según cfg_bif. SENSADO: hacia el extremo que vio la línea solo
// en los últimos T_MARCA_BIF_MS (p. ej. una marca de tape a un lado antes de la "Y");
// si ninguno, izquierda.
int8_t elegir_lado_bif() {
  if (cfg_bif == CMD_BIF_DERECHA)   return +1;
  if (cfg_bif == CMD_BIF_IZQUIERDA) return -1;
  if (ultimo_extremo != 0 && millis() - t_ultimo_extremo <= T_MARCA_BIF_MS) return ultimo_extremo;
  return -1;
}

// ============================================================================
// ACCIONES POR COLOR (TIM3) Y FIN DE LA BIFURCACIÓN (TIM4)
// ============================================================================

// TIM3 (un disparo): terminó el freno de 10 s o el retroceso de 3 s -> sigue avanzando
void ISR_FinAccion() {
  uint8_t est = estado;
  if (est != EST_ACT_FRENO && est != EST_ACT_REVERSA) return;   // se detuvo antes (STOP)
  if (est == EST_ACT_REVERSA) fin_reversa = true;
  reanudar();
}

// TIM4 (un disparo): pasó la bifurcación -> vuelve al seguimiento normal
void ISR_FinBifurcacion() {
  if (estado != EST_BIFURCACION) return;
  estado = EST_RUN;
  ISR_Linea();
}

// Vuelve a RUN después de una acción. Si ningún sensor del medio ve la línea en ese
// momento, ISR_Linea mantendría la salida anterior (detenido o en reversa): se avanza recto.
void reanudar() {
  noInterrupts();
  estado = EST_RUN;
  ISR_Linea();
  if (vel_izq <= 0 && vel_der <= 0) set_motors(vel_avance, vel_avance, "AVANZA");
  interrupts();
}

// Ejecuta la acción del color (lo llama loop, desde segmento_nuevo)
void iniciar_accion(uint8_t acc) {
  if (acc == ACC_NINGUNA) return;
  noInterrupts();
  if (acc == ACC_VELOCIDAD) {
    vel_avance = VEL_RAPIDA;                       // rápida por T_RAPIDO_MS (ISR_Tick la termina)
    ticks_rapido = (uint16_t)(T_RAPIDO_MS / T_TICK_MS);
    ISR_Linea();                                   // aplica la nueva velocidad ya
    interrupts();
    Consola.print(F("[ACCION] Velocidad RAPIDA por ")); Consola.print(T_RAPIDO_MS / 1000);
    Consola.println(F(" s"));
    return;
  }
  temporizador_parar(TIM_BIF);                     // una acción termina la bifurcación
  if (acc == ACC_FRENAR) {
    estado = EST_ACT_FRENO;
    stop_motors();
    temporizador_iniciar(TIM_ACCION, T_FRENO_MS);
  } else {                                         // ACC_RETROCEDER
    estado = EST_ACT_REVERSA;
    ignorar_segmentos = 1;                         // al volver se pasa otra vez por este segmento
    set_motors(-(int32_t)VEL_REVERSA, -(int32_t)VEL_REVERSA, "RETROCEDE");   // recto, sin seguir la línea
    temporizador_iniciar(TIM_ACCION, T_REVERSA_MS);
  }
  interrupts();
  Consola.print(F("[ACCION] ")); Consola.println(accion_nombre(acc));
}

// Configura un temporizador de 16 bits como "un disparo": cuenta hasta el valor pedido,
// genera UNA interrupción de actualización y se detiene solo (OPM).
void temporizador_config(timer_dev *t, voidFuncPtr isr) {
  timer_pause(t);
  t->regs.gen->CR1  = TIMER_CR1_URS | TIMER_CR1_OPM;  // URS: el UG de abajo no interrumpe
  t->regs.gen->DIER = 0;
  t->regs.gen->CCER = 0;                             // sin salidas en los pines del timer
  timer_set_prescaler(t, TEMP_PRESCALER);
  timer_set_reload(t, 1);
  timer_set_count(t, 0);
  timer_generate_update(t);                          // carga el prescaler
  t->regs.gen->SR = 0;
  timer_attach_interrupt(t, TIMER_UPDATE_INTERRUPT, isr);
}

void temporizador_iniciar(timer_dev *t, uint32_t ms) {
  timer_pause(t);
  timer_set_reload(t, (uint16_t)(ms * TEMP_TICKS_MS - 1));
  timer_set_count(t, 0);
  t->regs.gen->SR = 0;
  timer_resume(t);
}

// Temporizador repetitivo: una interrupción de actualización cada "ms" (sin OPM)
void temporizador_periodico(timer_dev *t, uint32_t ms, voidFuncPtr isr) {
  timer_pause(t);
  t->regs.gen->CR1  = TIMER_CR1_URS;                 // se repite; el UG de abajo no interrumpe
  t->regs.gen->DIER = 0;
  t->regs.gen->CCER = 0;                             // sin salidas en los pines del timer
  timer_set_prescaler(t, TEMP_PRESCALER);            // 2 kHz (0.5 ms)
  timer_set_reload(t, (uint16_t)(ms * TEMP_TICKS_MS - 1));
  timer_set_count(t, 0);
  timer_generate_update(t);                          // carga el prescaler
  t->regs.gen->SR = 0;
  timer_attach_interrupt(t, TIMER_UPDATE_INTERRUPT, isr);
  timer_resume(t);
}

void temporizador_parar(timer_dev *t) {
  timer_pause(t);
  t->regs.gen->SR = 0;                               // descarta una interrupción pendiente
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

// ISR de TIM1, cada T_TICK_MS (base de tiempo):
//  - COLOR_POR_INTERRUPCION 0: avisa a loop que toca consultar el sensor (sin I2C aquí)
//  - termina la velocidad rápida a los T_RAPIDO_MS
//  - termina el puente si pasó T_PUENTE_MS sin que S3 vuelva a ver la línea
//  - vuelve a revisar los 5 IR: si se perdió una interrupción de cambio de pin (un pulso
//    muy corto cuando la línea cruza un sensor), la salida se corrige en <= 5 ms
void ISR_Tick() {
#if !COLOR_POR_INTERRUPCION
  color_consultar = true;
#endif
  ISR_Linea();                           // solo actúa en RUN o BIFURCACION
  if (ticks_rapido && --ticks_rapido == 0) {
    vel_avance = VEL_BASE;
    fin_rapido = true;
    ISR_Linea();                         // aplica la velocidad normal ya (si está avanzando)
  }
  if (puente && millis() - t_puente >= T_PUENTE_MS) {
    puente = false;
    ISR_Linea();                         // vuelve a las reglas normales con lo que ven los IR
  }
}

// Lee n registros seguidos del TCS34725 en UNA transacción (auto-incremento).
// Leer los 8 bytes juntos asegura que C, R, G y B son de la misma medición.
bool tcs_leer(uint8_t reg, uint8_t *buf, uint8_t n) {
  BUS_COLOR.beginTransmission(TCS34725_ADDRESS);
  BUS_COLOR.write((uint8_t)(TCS34725_COMMAND_BIT | TCS34725_AUTOINCREMENTO | reg));
  if (BUS_COLOR.endTransmission() != 0) return false;
  if (BUS_COLOR.requestFrom(TCS34725_ADDRESS, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)BUS_COLOR.read();
  return true;
}

// Escribe un registro del TCS34725
bool tcs_escribir(uint8_t reg, uint8_t valor) {
  BUS_COLOR.beginTransmission(TCS34725_ADDRESS);
  BUS_COLOR.write((uint8_t)(TCS34725_COMMAND_BIT | reg));
  BUS_COLOR.write(valor);
  return BUS_COLOR.endTransmission() == 0;
}

// Borra la interrupción del sensor (AINT) y suelta el pin INT
bool tcs_limpiar_interrupcion() {
  BUS_COLOR.beginTransmission(TCS34725_ADDRESS);
  BUS_COLOR.write((uint8_t)TCS34725_LIMPIAR_INT);
  return BUS_COLOR.endTransmission() == 0;
}

// Revisa el ID y deja el sensor midiendo, con interrupción en cada fin de medición.
// Hace lo mismo que tcs.begin() + write8(PERS) + setInterrupt(true) + clearInterrupt().
bool tcs_iniciar() {
  uint8_t id;
  if (!tcs_leer(TCS34725_ID, &id, 1)) return false;
  if (id != 0x44 && id != 0x4D && id != 0x10) {
    Consola.print(F("[ERROR] ID del sensor de color inesperado: 0x")); Consola.println(id, HEX);
    return false;
  }
  if (!tcs_escribir(TCS34725_ATIME,   COLOR_T_INTEGRACION))  return false;  // 24 ms
  if (!tcs_escribir(TCS34725_CONTROL, TCS34725_GAIN_4X))     return false;  // ganancia 4x
  if (!tcs_escribir(TCS34725_PERS,    TCS34725_PERS_NONE))   return false;  // INT en cada medición
  if (!tcs_escribir(TCS34725_ENABLE,  TCS34725_ENABLE_PON))  return false;  // encender
  delay(3);                                                                 // 2.4 ms de arranque
  if (!tcs_escribir(TCS34725_ENABLE,  TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN | TCS34725_ENABLE_AIEN))
    return false;                                                           // medir + interrupción
  return tcs_limpiar_interrupcion();                                        // INT arranca en alto
}

void registrar_falla_color() {
  if (fallas_color < 255) fallas_color++;
  if (fallas_color >= 5) color_falla = true;     // la Pi lo ve como BAN_SENSOR_COLOR_FALLA
  if (color_falla && cal_restantes) {            // no se puede terminar la medición
    cal_restantes = 0;
    Consola.println(F("[CAL] Medición cancelada: el sensor de color no responde."));
  }
}

// Conecta el sensor: revisa los pull-ups, inicia el I2C2 y configura el TCS34725.
// Sin pull-ups no se llama a begin(): se quedaría trabado en el reset del bus (núcleo Maple).
bool tcs_conectar() {
  if (!bus_con_pullups(SCL_COLOR, SDA_COLOR)) { causa_color = CAUSA_SIN_PULLUPS; return false; }
  BUS_COLOR.begin();
  if (!tcs_iniciar()) { causa_color = CAUSA_NO_RESPONDE; return false; }
  return true;                       // causa_color se borra al leer la primera medición
}

const __FlashStringHelper* causa_nombre(uint8_t c) {
  switch (c) {
    case CAUSA_SIN_PULLUPS: return F("SIN PULL-UPS (sensor sin alimentación o cable suelto)");
    case CAUSA_NO_RESPONDE: return F("NO CONTESTA POR I2C");
    default:                return F("NO ENTREGA MEDICIONES");
  }
}

// true si el sensor no está entregando mediciones (no se encontró, falla el I2C o pasó
// T_SIN_DATO_COLOR_MS sin una medición nueva: con 24 ms de integración debería haber ~12)
bool color_sin_datos() {
  return !tcs_ok || color_falla || millis() - t_ult_dato_color > T_SIN_DATO_COLOR_MS;
}

// Llamada en cada vuelta de loop. Solo hace I2C cuando hay (o puede haber) dato nuevo.
void atender_sensor_color() {
  unsigned long ahora = millis();

  // Recuperación: si el sensor dejó de medir (por ejemplo, se reinició por una caída de
  // tensión al arrancar los motores y quedó apagado), se vuelve a configurar
  if (color_sin_datos()) {
    if (ahora - t_ult_reintento_color >= T_REINTENTO_COLOR_MS) {
      t_ult_reintento_color = ahora;
      if (tcs_ok) causa_color = CAUSA_SIN_MEDICIONES;   // estaba conectado pero no mide
      bool antes = tcs_ok;
      tcs_ok = tcs_conectar();
      if (tcs_ok) {
        if (n_reinicios_color < 0xFFFF) n_reinicios_color++;
        fallas_color = 0;
        color_falla = false;
        t_ult_dato_color = ahora;
        Consola.print(F("[COLOR] Sensor reiniciado (vez ")); Consola.print(n_reinicios_color);
        Consola.println(F(")."));
        if (!cal_valida) Consola.println(F("[COLOR] No hay calibración: envíe 'c' para calibrar."));
      } else if (antes) {
        Consola.print(F("[COLOR] El sensor no responde (")); Consola.print(causa_nombre(causa_color));
        Consola.println(F("): se reintenta cada 0.5 s."));
      }
    }
    if (!tcs_ok) return;
  }
#if COLOR_POR_INTERRUPCION
  bool consultar = (ahora - t_ult_consulta_color >= T_RESPALDO_COLOR_MS);  // respaldo si se pierde un flanco
#else
  bool consultar = color_consultar;     // lo levanta ISR_Tick (TIM1) cada T_TICK_MS
  color_consultar = false;
#endif
  bool hay_dato = false;

  if (color_listo) {                    // aviso del pin INT
    color_listo = false;
    hay_dato = true;
  } else if (consultar) {
    t_ult_consulta_color = ahora;
    uint8_t st;
    if (!tcs_leer(TCS34725_STATUS, &st, 1)) { registrar_falla_color(); return; }
    hay_dato = (st & TCS34725_STATUS_AINT) != 0;      // AINT = medición nueva sin leer
  }
  if (!hay_dato) return;
  t_ult_consulta_color = ahora;

  uint8_t b[8];                         // CDATAL, CDATAH, RDATAL, ... BDATAH (0x14..0x1B)
  if (!tcs_leer(TCS34725_CDATAL, b, 8)) { registrar_falla_color(); return; }
  tcs_limpiar_interrupcion();           // borra AINT y suelta INT: la próxima medición da un flanco nuevo
  fallas_color = 0;
  color_falla = false;
  t_ult_dato_color = ahora;
  causa_color = CAUSA_NINGUNA;

  col_c = (uint16_t)(b[0] | (b[1] << 8));
  col_r = (uint16_t)(b[2] | (b[3] << 8));
  col_g = (uint16_t)(b[4] | (b[5] << 8));
  col_b = (uint16_t)(b[6] | (b[7] << 8));
  if (estado == EST_CALIBRACION) cal_muestra();   // calibrando: la medición es una muestra
  else                           procesar_color();
}

// Decide qué color hay bajo el sensor comparando con las 5 referencias de la calibración.
// Cada medición se describe con 4 números:
//   r, g, b normalizados (canal * 1000 / C): el color, sin importar cuánta luz llega
//   brillo = C * 1000 / C_blanco: qué tan clara es la superficie (separa negro y blanco)
// Se elige la referencia más cercana (distancia al cuadrado; el brillo pesa la mitad).
// Si la segunda más cercana está casi igual de cerca (borde entre dos colores), la
// medición se descarta como INDETERMINADO. NEGRO y BLANCO cuentan como "ningún color".
uint8_t clasificar_color(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
  if (c >= CUENTA_MAX_COLOR) return COLOR_INDETERMINADO;              // saturado
  uint16_t n[3] = { color_normalizar(r, c), color_normalizar(g, c), color_normalizar(b, c) };
  for (uint8_t i = 0; i < 3; i++) if (n[i] > N_MAX) n[i] = N_MAX;
  const uint32_t c_blanco = ref_c[1] ? ref_c[1] : 1;                   // paso 1 = BLANCO
  uint32_t brillo = (uint32_t)c * 1000u / c_blanco;
  if (brillo > BRILLO_MAX) brillo = BRILLO_MAX;

  uint32_t mejor = 0xFFFFFFFFu, segundo = 0xFFFFFFFFu;
  uint8_t paso_mejor = 0;
  for (uint8_t k = 0; k < CAL_PASOS; k++) {
    uint32_t brillo_ref = (uint32_t)ref_c[k] * 1000u / c_blanco;
    if (brillo_ref > BRILLO_MAX) brillo_ref = BRILLO_MAX;
    int32_t d0 = (int32_t)n[0] - (int32_t)ref_n[k][0];
    int32_t d1 = (int32_t)n[1] - (int32_t)ref_n[k][1];
    int32_t d2 = (int32_t)n[2] - (int32_t)ref_n[k][2];
    int32_t db = ((int32_t)brillo - (int32_t)brillo_ref) / 2;
    uint32_t d = (uint32_t)(d0 * d0) + (uint32_t)(d1 * d1) + (uint32_t)(d2 * d2) + (uint32_t)(db * db);
    if (d < mejor)        { segundo = mejor; mejor = d; paso_mejor = k; }
    else if (d < segundo) { segundo = d; }
  }
  if ((uint64_t)segundo * 100u < (uint64_t)mejor * MARGEN_AMBIGUO_PCT)
    return COLOR_INDETERMINADO;                                        // ambiguo
  uint8_t col = cal_color_paso(paso_mejor);
  return (col == COLOR_NEGRO || col == COLOR_BLANCO) ? COLOR_NINGUNO : col;
}

// Antirrebote y conteo. Un color se acepta tras N_CONFIRMA mediciones iguales
// seguidas, y un segmento se cuenta una sola vez: para volver a contar hay que
// salir del segmento (NINGUNO confirmado) o pasar a otro color.
void procesar_color() {
  if (!cal_valida) return;                           // sin referencias no se puede clasificar
  uint8_t c = clasificar_color(col_c, col_r, col_g, col_b);
  if (c == COLOR_ROJO || c == COLOR_VERDE || c == COLOR_AZUL)
    t_ult_color_visto = millis();                    // para RECTO_EN_COLOR (sin esperar confirmación)
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
  segmento_nuevo(c);
}

// El carro entró a un segmento de color: contar y ejecutar su acción.
// Después de retroceder, el carro vuelve a pasar por los segmentos que cruzó en reversa
// y por el que disparó el retroceso: esos no se cuentan ni repiten la acción.
void segmento_nuevo(uint8_t c) {
  uint8_t est = estado;
  if (est == EST_ACT_REVERSA) {                  // cruzando hacia atrás: se volverá a pasar
    if (ignorar_segmentos < 255) ignorar_segmentos++;
    return;
  }
  // Solo se cuenta avanzando por la pista (no detenido ni frenando)
  if (est != EST_RUN && est != EST_BIFURCACION) return;
  if (ignorar_segmentos) {
    ignorar_segmentos--;
    Consola.print(F("[COLOR] Segmento ")); Consola.print(color_nombre(c));
    Consola.println(F(" ya contado (de vuelta tras retroceder)"));
    return;
  }

  // Un color por vuelta: si este color ya se hizo y faltan otros, no se ejecuta
  const uint8_t bit = (uint8_t)(1u << (c - COLOR_ROJO));     // ROJO=1, VERDE=2, AZUL=3
  const bool repetido = (colores_vuelta & bit) != 0;
  if (repetido && !CONTAR_REPETIDOS) {
    Consola.print(F("[COLOR] Segmento ")); Consola.print(color_nombre(c));
    Consola.println(F(" ignorado: ya se hizo en esta vuelta (faltan otros colores)"));
    return;
  }

  uint16_t *cuenta = (c == COLOR_ROJO) ? &count_red : (c == COLOR_VERDE) ? &count_green : &count_blue;
  if (*cuenta < 0xFFFFu) (*cuenta)++;
  Consola.print(F("[COLOR] Segmento ")); Consola.print(color_nombre(c));
  Consola.print(F(" #")); Consola.println(*cuenta);
  t_led_color = millis();                         // el LED queda encendido T_LED_COLOR_MS
  if (repetido) {                                 // CONTAR_REPETIDOS: contado, sin acción
    Consola.println(F("[COLOR] Acción omitida: ese color ya se hizo en esta vuelta"));
    return;
  }

  colores_vuelta |= bit;
  if (colores_vuelta == TODOS_LOS_COLORES) {
    colores_vuelta = 0;                           // ya se hicieron los 3: vuelta nueva
    Consola.println(F("[COLOR] Vuelta completa (rojo, verde y azul): se habilitan los 3 colores"));
  }
  iniciar_accion(color_accion(cfg_color, c));    // cfg_color = 0 -> ACC_NINGUNA
}

// LED de la placa (activo en bajo). Ver el significado junto a T_LED_COLOR_MS.
void actualizar_led() {
  static unsigned long t_led = 0;
  static bool encendido = false;
  static uint8_t pulso = 0;                        // parpadeos ya hechos en el grupo actual
  unsigned long ahora = millis();

  if (t_led_color != 0 && ahora - t_led_color < T_LED_COLOR_MS) {
    encendido = true;                              // se contó un color: encendido fijo
    pulso = 0;
  } else if (!color_sin_datos()) {
    pulso = 0;
    if (ahora - t_led < 500) return;               // todo bien: parpadeo lento
    t_led = ahora;
    encendido = !encendido;
  } else {
    // Código de falla: N parpadeos de 150 ms y una pausa de 1.5 s
    const uint8_t n = causa_color ? causa_color : CAUSA_SIN_MEDICIONES;
    const unsigned long espera = encendido ? 150 : (pulso == 0 ? 1500 : 150);
    if (ahora - t_led < espera) return;
    t_led = ahora;
    if (encendido) {                               // termina un parpadeo
      encendido = false;
      if (++pulso >= n) pulso = 0;                 // grupo completo: viene la pausa larga
    } else {
      encendido = true;
    }
  }
  digitalWrite(LED_PLACA, encendido ? LOW : HIGH);
}

// ============================================================================
// CALIBRACIÓN DEL SENSOR DE COLOR
// ============================================================================

// Empieza (o reinicia) la calibración desde NEGRO. El carro queda detenido.
void cal_iniciar() {
  cal_paso = 0;
  cal_restantes = 0;
  cal_completa = false;
  cal_valida = false;
  color_candidato = COLOR_NINGUNO;
  n_iguales = 0;
  ultimo_contado = COLOR_NINGUNO;
  color_actual = COLOR_NINGUNO;
  estado = EST_CALIBRACION;
  Consola.println(F("[CAL] Calibración del sensor de color: NEGRO, BLANCO, AZUL, ROJO, VERDE."));
  cal_avisar_paso();
}

// Pide tomar la medición del paso actual. false si no corresponde ahora.
bool cal_pedir_medicion() {
  if (estado != EST_CALIBRACION || cal_restantes > 0 || !tcs_ok || color_falla) return false;
  for (uint8_t i = 0; i < 4; i++) cal_suma[i] = 0;
  cal_restantes = N_MUESTRAS_CAL;
  Consola.print(F("[CAL] Midiendo ")); Consola.print(color_nombre(cal_color_paso(cal_paso)));
  Consola.println(F("... no mueva el sensor."));
  return true;
}

// Una medición nueva durante la calibración: se acumula; al juntar N_MUESTRAS_CAL se
// guarda el promedio como referencia del color del paso y se pasa al siguiente.
void cal_muestra() {
  if (cal_restantes == 0) return;                  // esperando la orden de medir
  cal_suma[0] += col_c;
  cal_suma[1] += color_normalizar(col_r, col_c);
  cal_suma[2] += color_normalizar(col_g, col_c);
  cal_suma[3] += color_normalizar(col_b, col_c);
  if (--cal_restantes > 0) return;

  const uint8_t k = cal_paso;
  ref_c[k] = (uint16_t)(cal_suma[0] / N_MUESTRAS_CAL);
  for (uint8_t i = 0; i < 3; i++) {
    uint32_t v = cal_suma[i + 1] / N_MUESTRAS_CAL;
    ref_n[k][i] = (uint16_t)(v > N_MAX ? N_MAX : v);
  }
  Consola.print(F("[CAL] ")); Consola.print(color_nombre(cal_color_paso(k)));
  Consola.print(F(" guardado: C = ")); Consola.print(ref_c[k]);
  Consola.print(F(" | r = ")); Consola.print(ref_n[k][0]);
  Consola.print(F(" g = ")); Consola.print(ref_n[k][1]);
  Consola.print(F(" b = ")); Consola.println(ref_n[k][2]);
  if ((uint32_t)ref_c[k] * 100u >= CUENTA_MAX_COLOR * 95u)
    Consola.println(F("[CAL] AVISO: el sensor está casi saturado. Súbalo un poco o baje la ganancia."));

  // El blanco tiene que verse más claro que el negro; si no, se repite el blanco
  if (k == 1 && ref_c[1] <= ref_c[0]) {
    Consola.println(F("[CAL] El BLANCO salió igual o más oscuro que el NEGRO: repita la medición del BLANCO."));
    cal_avisar_paso();
    return;
  }

  cal_paso++;
  if (cal_paso >= CAL_PASOS) {
    Consola.println(F("[CAL] Calibración completa."));
    cal_valida = true;
    if (cal_guardar()) Consola.println(F("[CAL] Guardada en la flash: se usará en los próximos encendidos."));
    else               Consola.println(F("[CAL] ERROR: no se pudo guardar en la flash (vale solo hasta apagar)."));
    cal_completa = true;
    t_fin_cal = millis();          // después de grabar: la flash tarda unas decenas de ms
    estado = EST_IDLE;
#if MODO_SIN_PI
    Consola.print(F("[CAL] El carro arranca en ")); Consola.print(T_ARRANQUE_MS / 1000);
    Consola.println(F(" s: póngalo sobre la pista."));
#endif
  } else {
    cal_avisar_paso();
  }
}

// Dice por el Monitor Serie qué color hay que medir, con la lectura actual
void cal_avisar_paso() {
  Consola.print(F("[CAL] Paso ")); Consola.print(cal_paso + 1);
  Consola.print(F("/")); Consola.print(CAL_PASOS);
  Consola.print(F(": coloque el sensor sobre ")); Consola.print(color_nombre(cal_color_paso(cal_paso)));
#if MODO_SIN_PI
  Consola.print(F(" y envíe 'm'"));
#else
  Consola.print(F(" y envíe 'm' (o 'Tomar medicion' en la interfaz)"));
#endif
  if (cal_restantes) Consola.print(F(" (midiendo...)"));
  Consola.print(F(" | ahora: C = ")); Consola.print(col_c);
  Consola.print(F(" r = ")); Consola.print(color_normalizar(col_r, col_c));
  Consola.print(F(" g = ")); Consola.print(color_normalizar(col_g, col_c));
  Consola.print(F(" b = ")); Consola.println(color_normalizar(col_b, col_c));
}

// ============================================================================
// CALIBRACIÓN EN LA FLASH (librería EEPROM del núcleo Maple)
// ============================================================================

// Dato i de la calibración: 0..CAL_PASOS-1 = ref_c[], después ref_n[][] fila por fila
uint16_t* cal_dato(uint16_t i) {
  if (i < CAL_PASOS) return &ref_c[i];
  i -= CAL_PASOS;
  return &ref_n[i / 3][i % 3];
}

// Suma de verificación de las referencias (detecta datos a medio grabar o corruptos)
uint16_t cal_suma_verif() {
  uint16_t s = 0x5A5A;
  for (uint16_t i = 0; i < N_DATOS_CAL; i++)
    s = (uint16_t)(((s << 1) | (s >> 15)) ^ *cal_dato(i));
  return s;
}

// Escribe solo si el valor cambió (cuida la flash: cada página aguanta ~10 000 borrados)
bool flash_escribir(uint16_t dir, uint16_t valor) {
  uint16_t actual;
  if (EEPROM.read(dir, &actual) == EEPROM_OK && actual == valor) return true;
  return EEPROM.write(dir, valor) == EEPROM_OK;
}

// Graba la calibración. El número mágico se borra primero y se escribe al final: si se
// corta la corriente a mitad de la grabación, al encender no se carga nada a medias.
bool cal_guardar() {
  if (!flash_ok) return false;
  bool ok = flash_escribir(DIR_CAL_MAGICO, 0);
  ok = ok && flash_escribir(DIR_CAL_VERSION, CAL_VERSION);
  for (uint16_t i = 0; ok && i < N_DATOS_CAL; i++)
    ok = flash_escribir(DIR_CAL_DATOS + i, *cal_dato(i));
  ok = ok && flash_escribir(DIR_CAL_SUMA, cal_suma_verif());
  ok = ok && flash_escribir(DIR_CAL_MAGICO, CAL_MAGICO);
  return ok;
}

// Lee la calibración guardada. false si no hay o no es válida (entonces hay que calibrar).
bool cal_cargar() {
  if (!flash_ok) return false;
  uint16_t v;
  if (EEPROM.read(DIR_CAL_MAGICO, &v)  != EEPROM_OK || v != CAL_MAGICO)  return false;
  if (EEPROM.read(DIR_CAL_VERSION, &v) != EEPROM_OK || v != CAL_VERSION) return false;
  for (uint16_t i = 0; i < N_DATOS_CAL; i++) {
    if (EEPROM.read(DIR_CAL_DATOS + i, &v) != EEPROM_OK) return false;
    *cal_dato(i) = v;
  }
  if (EEPROM.read(DIR_CAL_SUMA, &v) != EEPROM_OK || v != cal_suma_verif()) return false;
  if (ref_c[1] <= ref_c[0]) return false;          // el BLANCO debe ser más claro que el NEGRO
  return true;
}

// Muestra las 5 referencias por el Monitor Serie
void cal_imprimir_referencias() {
  for (uint8_t k = 0; k < CAL_PASOS; k++) {
    Consola.print(F("[CAL]   ")); Consola.print(color_nombre(cal_color_paso(k)));
    Consola.print(F(": C = ")); Consola.print(ref_c[k]);
    Consola.print(F(" | r = ")); Consola.print(ref_n[k][0]);
    Consola.print(F(" g = ")); Consola.print(ref_n[k][1]);
    Consola.print(F(" b = ")); Consola.println(ref_n[k][2]);
  }
}

// ============================================================================
// COMUNICACIÓN I2C CON LA RASPBERRY PI (protocolo.h v0x03)
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
    if (estado == EST_CALIBRACION) stop_motors();  // detenido ya; la calibración sigue
    else detener(EST_IDLE);                     // siempre se ejecuta, en cualquier estado
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
  else if (cmd_es_color(cmd)) {                 // configurar solo detenido (o calibrando)
    if (estado != EST_IDLE && estado != EST_CALIBRACION) r = RES_ERR_ESTADO;
    else cfg_color = cmd;
  }
  else if (cmd_es_bif(cmd)) {
    if (estado != EST_IDLE && estado != EST_CALIBRACION) r = RES_ERR_ESTADO;
    else cfg_bif = cmd;
  }
  else if (cmd == CMD_CAL_MEDIR) {
    if (!cal_pedir_medicion()) r = RES_ERR_ESTADO;      // no calibrando, ya midiendo o sin sensor
  }
  else if (cmd == CMD_CAL_REINICIAR) {
    if ((estado == EST_IDLE || estado == EST_CALIBRACION) && tcs_ok) cal_iniciar();
    else r = RES_ERR_ESTADO;
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
                    | ((tcs_ok && !color_falla) ? 0 : BAN_SENSOR_COLOR_FALLA)
                    | (cal_restantes ? BAN_CAL_MIDIENDO : 0));
  e.cfg_color     = cfg_color;
  e.cfg_bif       = cfg_bif;
  e.color         = (estado == EST_CALIBRACION) ? cal_color_paso(cal_paso) : color_actual;
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
// Cada recorrido empieza a velocidad normal y sin acciones pendientes.
void arrancar() {
  noInterrupts();
  vel_avance = VEL_BASE;
  ticks_rapido = 0;
  puente = false;
  t_ult_s3 = millis();                            // se arranca sobre la línea
  ultimo_lado = 0;
  t_sin_linea = 0;
  ext_desde_centro = false;
  ignorar_segmentos = 0;
  colores_vuelta = 0;                             // cada recorrido empieza con los 3 colores
  bif_armada = true;
  ultimo_extremo = 0;
  estado = EST_RUN;
  ISR_Linea();
  interrupts();
}

// Cambia de estado ANTES de parar los motores: si entra ISR_Linea en medio, ya no los mueve.
// También cancela el freno, el retroceso o la bifurcación en curso.
void detener(uint8_t nuevo_estado) {
  noInterrupts();
  estado = nuevo_estado;
  ticks_rapido = 0;                              // cancela la velocidad rápida
  vel_avance = VEL_BASE;
  puente = false;
  temporizador_parar(TIM_ACCION);
  temporizador_parar(TIM_BIF);
  stop_motors();
  interrupts();
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
void drive_motor(int pin_pwm, int pin_dir, int32_t v) {
  uint32_t m = (v >= 0) ? (uint32_t)v : (uint32_t)(-v);
  if (m > PWM_MAX) m = PWM_MAX;

  if (v >= 0) {
    digitalWrite(pin_dir, LOW);
    pwmWrite(pin_pwm, m);
  } else {
    digitalWrite(pin_dir, HIGH);
    pwmWrite(pin_pwm, PWM_MAX - m);
  }
}
