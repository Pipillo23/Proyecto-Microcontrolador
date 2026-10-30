/* ============================================================================
 * protocolo.h  -  TABLA DE TRAMAS I2C   Raspberry Pi (maestro) <-> STM32 (esclavo)
 * ----------------------------------------------------------------------------
 * Este MISMO archivo se usa en la Raspberry Pi y en el STM32. Si cambias un
 * codigo aqui, copia el archivo a los dos proyectos.
 *
 * Protocolo v0x02: reduccion de SPEC-I2C-01 v1.3 a lo que pide "Diseno Interfaz".
 *   - Trama de COMANDO (Pi -> STM32): largo fijo de 4 bytes.
 *     Cada configuracion posible tiene su propio codigo, por eso no hay payload.
 *   - Trama de ESTADO (STM32 -> Pi): largo fijo de 17 bytes, la Pi la lee
 *     periodicamente (en I2C el esclavo no puede transmitir por iniciativa propia).
 *
 * Reglas heredadas de la SPEC:
 *   - Multibyte en little-endian (byte bajo primero).
 *   - Las tramas se arman y se leen byte a byte (nunca memcpy de un struct).
 *   - CRC-16/CCITT-FALSE: poli 0x1021, inicial 0xFFFF, sin reflexion, sin XOR.
 * ==========================================================================*/
#ifndef PROTOCOLO_H
#define PROTOCOLO_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Parametros generales
 * ------------------------------------------------------------------------*/
#define PROTO_VERSION          0x02u   /* viaja en la trama de estado */
#define I2C_DIR_STM32          0x42u   /* direccion de 7 bits del STM32 */

#define T_HEARTBEAT_MS         100u    /* la Pi envia algo al menos cada 100 ms */
#define T_WATCHDOG_STM_MS      500u    /* sin comandos 500 ms en movimiento -> STM32 para */
#define T_PUBLICACION_STM_MS   10u     /* el STM32 refresca la trama de estado */

/* ==========================================================================
 * 1) TRAMA DE COMANDO  (Pi -> STM32), siempre 4 bytes
 *
 *    Byte 0   SEQ    numero de secuencia 1..255 (0 solo en HEARTBEAT)
 *    Byte 1   CMD    codigo de la tabla de abajo
 *    Byte 2   CRC_L  CRC-16 de los bytes 0..1, byte bajo
 *    Byte 3   CRC_H  CRC-16 de los bytes 0..1, byte alto
 * ========================================================================*/
#define CMD_LEN                4u

/* ---- TABLA DE COMANDOS: nombre -> hexadecimal ------------------------- */
#define CMD_HEARTBEAT          0x01u   /* "sigo vivo"; no cambia nada en el STM32   */
#define CMD_START              0x10u   /* iniciar operacion (solo desde IDLE)       */
#define CMD_STOP               0x11u   /* detener; siempre se ejecuta, prioridad maxima */

/* Configuracion de colores: accion asociada a  ROJO / VERDE / AZUL
 * Acciones: FRE = frenar 10 s, RET = retroceder 3 s, VEL = cambio de velocidad.
 * 3 colores y 3 acciones distintas = 3! = 6 combinaciones posibles.       */
#define CMD_COL_FRE_RET_VEL    0x30u   /* R=Frenar      G=Retroceder  B=Velocidad  */
#define CMD_COL_FRE_VEL_RET    0x31u   /* R=Frenar      G=Velocidad   B=Retroceder */
#define CMD_COL_RET_FRE_VEL    0x32u   /* R=Retroceder  G=Frenar      B=Velocidad  */
#define CMD_COL_RET_VEL_FRE    0x33u   /* R=Retroceder  G=Velocidad   B=Frenar     */
#define CMD_COL_VEL_FRE_RET    0x34u   /* R=Velocidad   G=Frenar      B=Retroceder */
#define CMD_COL_VEL_RET_FRE    0x35u   /* R=Velocidad   G=Retroceder  B=Frenar     */
#define CMD_COL_PRIMERO        CMD_COL_FRE_RET_VEL
#define CMD_COL_ULTIMO         CMD_COL_VEL_RET_FRE

/* Configuracion de bifurcaciones                                          */
#define CMD_BIF_SENSADO        0x40u   /* gira hacia el sensor que detecto: S0 izq, S4 der */
#define CMD_BIF_IZQUIERDA      0x41u   /* siempre izquierda */
#define CMD_BIF_DERECHA        0x42u   /* siempre derecha   */
#define CMD_BIF_PRIMERO        CMD_BIF_SENSADO
#define CMD_BIF_ULTIMO         CMD_BIF_DERECHA

/* ==========================================================================
 * 2) TRAMA DE ESTADO  (STM32 -> Pi), siempre 17 bytes
 * ========================================================================*/
#define ST_LEN                 17u
#define ST_VERSION             0u   /* u8  = PROTO_VERSION                          */
#define ST_PUB_CNT             1u   /* u8  +1 en cada publicacion (detecta STM32 congelado) */
#define ST_ULT_SEQ             2u   /* u8  SEQ del ultimo comando ejecutado (0 = ninguno) */
#define ST_ULT_RESULTADO       3u   /* u8  resultado de ese comando (RES_*)          */
#define ST_ESTADO              4u   /* u8  EST_*                                     */
#define ST_BANDERAS            5u   /* u8  BAN_*                                     */
#define ST_CFG_COLOR           6u   /* u8  CMD_COL_* activo (0 = sin configurar)     */
#define ST_CFG_BIF             7u   /* u8  CMD_BIF_* activo (0 = sin configurar)     */
#define ST_COLOR               8u   /* u8  color bajo el sensor (COLOR_*)            */
#define ST_CUENTA_R            9u   /* u16 segmentos rojos                           */
#define ST_CUENTA_G            11u  /* u16 segmentos verdes                          */
#define ST_CUENTA_B            13u  /* u16 segmentos azules                          */
#define ST_CRC                 15u  /* u16 CRC-16 de los bytes 0..14                 */

/* Estados del carro */
#define EST_IDLE               0x00u  /* detenido; unico estado que acepta configuracion */
#define EST_RUN                0x01u  /* siguiendo linea                             */
#define EST_ACT_FRENO          0x02u  /* accion: frenado 10 s, luego vuelve a RUN    */
#define EST_ACT_REVERSA        0x03u  /* accion: retroceso 3 s, luego vuelve a RUN   */
#define EST_BIFURCACION        0x04u  /* resolviendo una bifurcacion                 */
#define EST_FALLA_COM          0x05u  /* detenido por perdida de enlace; requiere STOP */

/* Resultados del ultimo comando */
#define RES_OK                 0x00u
#define RES_ERR_DESCONOCIDO    0x03u  /* CMD que no esta en la tabla                 */
#define RES_ERR_ESTADO         0x04u  /* no permitido en el estado actual (p. ej. config en movimiento) */
#define RES_ERR_SIN_CONFIG     0x06u  /* START sin colores o sin bifurcacion configurados */

/* Banderas */
#define BAN_COM_PERDIDA        0x01u  /* el STM32 paro por falta de comandos (se borra con START) */
#define BAN_LINEA_PERDIDA      0x02u  /* la linea no se ve en este momento           */
#define BAN_SENSOR_COLOR_FALLA 0x04u  /* el sensor de color no responde              */

/* Colores */
#define COLOR_NINGUNO          0x00u
#define COLOR_ROJO             0x01u
#define COLOR_VERDE            0x02u
#define COLOR_AZUL             0x03u
#define COLOR_INDETERMINADO    0xFFu

/* Acciones */
#define ACC_NINGUNA            0x00u
#define ACC_FRENAR             0x01u
#define ACC_RETROCEDER         0x02u
#define ACC_VELOCIDAD          0x03u

/* Estado decodificado (solo para usar en C; NUNCA se copia directo a la trama) */
typedef struct {
    uint8_t  version;
    uint8_t  pub_cnt;
    uint8_t  ult_seq;
    uint8_t  ult_resultado;
    uint8_t  estado;
    uint8_t  banderas;
    uint8_t  cfg_color;
    uint8_t  cfg_bif;
    uint8_t  color;
    uint16_t cuenta_r;
    uint16_t cuenta_g;
    uint16_t cuenta_b;
} estado_t;

#define DEC_OK                 0
#define DEC_ERR_CRC           -1
#define DEC_ERR_VERSION       -2

/* ==========================================================================
 * 3) FUNCIONES COMUNES (iguales en la Pi y en el STM32)
 * ========================================================================*/
static inline uint16_t crc16(const uint8_t *d, uint16_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint16_t)((uint16_t)(*d++) << 8);
        for (uint8_t i = 0; i < 8u; i++)
            c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
    return c;
}

static inline int cmd_es_color(uint8_t c) { return c >= CMD_COL_PRIMERO && c <= CMD_COL_ULTIMO; }
static inline int cmd_es_bif(uint8_t c)   { return c >= CMD_BIF_PRIMERO && c <= CMD_BIF_ULTIMO; }

static inline int estado_en_movimiento(uint8_t e)
{
    return e == EST_RUN || e == EST_ACT_FRENO || e == EST_ACT_REVERSA || e == EST_BIFURCACION;
}

/* Accion asociada a un color (COLOR_ROJO..AZUL) segun el codigo CMD_COL_* activo */
static inline uint8_t color_accion(uint8_t cmd_color, uint8_t color)
{
    static const uint8_t tabla[6][3] = {
        /*  ROJO            VERDE           AZUL          */
        { ACC_FRENAR,     ACC_RETROCEDER, ACC_VELOCIDAD  },  /* 0x30 */
        { ACC_FRENAR,     ACC_VELOCIDAD,  ACC_RETROCEDER },  /* 0x31 */
        { ACC_RETROCEDER, ACC_FRENAR,     ACC_VELOCIDAD  },  /* 0x32 */
        { ACC_RETROCEDER, ACC_VELOCIDAD,  ACC_FRENAR     },  /* 0x33 */
        { ACC_VELOCIDAD,  ACC_FRENAR,     ACC_RETROCEDER },  /* 0x34 */
        { ACC_VELOCIDAD,  ACC_RETROCEDER, ACC_FRENAR     },  /* 0x35 */
    };
    if (!cmd_es_color(cmd_color) || color < COLOR_ROJO || color > COLOR_AZUL)
        return ACC_NINGUNA;
    return tabla[cmd_color - CMD_COL_PRIMERO][color - COLOR_ROJO];
}

/* Inverso: a partir de la accion de cada color devuelve el CMD_COL_* (0 si no existe) */
static inline uint8_t color_cmd_desde_acciones(uint8_t acc_r, uint8_t acc_g, uint8_t acc_b)
{
    for (uint8_t c = CMD_COL_PRIMERO; c <= CMD_COL_ULTIMO; c++)
        if (color_accion(c, COLOR_ROJO) == acc_r && color_accion(c, COLOR_VERDE) == acc_g &&
            color_accion(c, COLOR_AZUL) == acc_b)
            return c;
    return 0;
}

/* Arma una trama de comando de 4 bytes */
static inline void cmd_armar(uint8_t t[CMD_LEN], uint8_t seq, uint8_t cmd)
{
    t[0] = seq;
    t[1] = cmd;
    uint16_t c = crc16(t, 2);
    t[2] = (uint8_t)(c & 0xFFu);
    t[3] = (uint8_t)(c >> 8);
}

/* Valida una trama de comando recibida. Devuelve 1 si es valida. */
static inline int cmd_leer(const uint8_t *t, uint32_t n, uint8_t *seq, uint8_t *cmd)
{
    if (n != CMD_LEN) return 0;
    uint16_t c = crc16(t, 2);
    if (t[2] != (uint8_t)(c & 0xFFu) || t[3] != (uint8_t)(c >> 8)) return 0;
    *seq = t[0];
    *cmd = t[1];
    return 1;
}

static inline void estado_codificar(const estado_t *e, uint8_t b[ST_LEN])
{
    b[ST_VERSION]       = PROTO_VERSION;
    b[ST_PUB_CNT]       = e->pub_cnt;
    b[ST_ULT_SEQ]       = e->ult_seq;
    b[ST_ULT_RESULTADO] = e->ult_resultado;
    b[ST_ESTADO]        = e->estado;
    b[ST_BANDERAS]      = e->banderas;
    b[ST_CFG_COLOR]     = e->cfg_color;
    b[ST_CFG_BIF]       = e->cfg_bif;
    b[ST_COLOR]         = e->color;
    b[ST_CUENTA_R]      = (uint8_t)(e->cuenta_r & 0xFFu);
    b[ST_CUENTA_R + 1]  = (uint8_t)(e->cuenta_r >> 8);
    b[ST_CUENTA_G]      = (uint8_t)(e->cuenta_g & 0xFFu);
    b[ST_CUENTA_G + 1]  = (uint8_t)(e->cuenta_g >> 8);
    b[ST_CUENTA_B]      = (uint8_t)(e->cuenta_b & 0xFFu);
    b[ST_CUENTA_B + 1]  = (uint8_t)(e->cuenta_b >> 8);
    uint16_t c = crc16(b, ST_CRC);
    b[ST_CRC]     = (uint8_t)(c & 0xFFu);
    b[ST_CRC + 1] = (uint8_t)(c >> 8);
}

/* Devuelve DEC_OK, DEC_ERR_CRC o DEC_ERR_VERSION */
static inline int estado_decodificar(const uint8_t b[ST_LEN], estado_t *e)
{
    uint16_t c = crc16(b, ST_CRC);
    if (b[ST_CRC] != (uint8_t)(c & 0xFFu) || b[ST_CRC + 1] != (uint8_t)(c >> 8))
        return DEC_ERR_CRC;
    if (b[ST_VERSION] != PROTO_VERSION)
        return DEC_ERR_VERSION;
    e->version       = b[ST_VERSION];
    e->pub_cnt       = b[ST_PUB_CNT];
    e->ult_seq       = b[ST_ULT_SEQ];
    e->ult_resultado = b[ST_ULT_RESULTADO];
    e->estado        = b[ST_ESTADO];
    e->banderas      = b[ST_BANDERAS];
    e->cfg_color     = b[ST_CFG_COLOR];
    e->cfg_bif       = b[ST_CFG_BIF];
    e->color         = b[ST_COLOR];
    e->cuenta_r      = (uint16_t)(b[ST_CUENTA_R] | (b[ST_CUENTA_R + 1] << 8));
    e->cuenta_g      = (uint16_t)(b[ST_CUENTA_G] | (b[ST_CUENTA_G + 1] << 8));
    e->cuenta_b      = (uint16_t)(b[ST_CUENTA_B] | (b[ST_CUENTA_B + 1] << 8));
    return DEC_OK;
}

/* ---- Nombres legibles (los usa la Pi; en el STM32 no ocupan memoria si no se llaman) */
static inline const char *cmd_nombre(uint8_t c)
{
    switch (c) {
    case CMD_HEARTBEAT:       return "HEARTBEAT";
    case CMD_START:           return "START";
    case CMD_STOP:            return "STOP";
    case CMD_COL_FRE_RET_VEL: return "COL_FRE_RET_VEL";
    case CMD_COL_FRE_VEL_RET: return "COL_FRE_VEL_RET";
    case CMD_COL_RET_FRE_VEL: return "COL_RET_FRE_VEL";
    case CMD_COL_RET_VEL_FRE: return "COL_RET_VEL_FRE";
    case CMD_COL_VEL_FRE_RET: return "COL_VEL_FRE_RET";
    case CMD_COL_VEL_RET_FRE: return "COL_VEL_RET_FRE";
    case CMD_BIF_SENSADO:     return "BIF_SENSADO";
    case CMD_BIF_IZQUIERDA:   return "BIF_IZQUIERDA";
    case CMD_BIF_DERECHA:     return "BIF_DERECHA";
    default:                  return "DESCONOCIDO";
    }
}

static inline const char *estado_nombre(uint8_t e)
{
    switch (e) {
    case EST_IDLE:        return "IDLE";
    case EST_RUN:         return "RUN";
    case EST_ACT_FRENO:   return "FRENANDO 10 s";
    case EST_ACT_REVERSA: return "RETROCEDIENDO 3 s";
    case EST_BIFURCACION: return "BIFURCACION";
    case EST_FALLA_COM:   return "FALLA DE ENLACE";
    default:              return "?";
    }
}

static inline const char *resultado_nombre(uint8_t r)
{
    switch (r) {
    case RES_OK:              return "OK";
    case RES_ERR_DESCONOCIDO: return "ERR_DESCONOCIDO";
    case RES_ERR_ESTADO:      return "ERR_ESTADO";
    case RES_ERR_SIN_CONFIG:  return "ERR_SIN_CONFIG";
    default:                  return "?";
    }
}

static inline const char *accion_nombre(uint8_t a)
{
    switch (a) {
    case ACC_FRENAR:     return "Frenar 10 s";
    case ACC_RETROCEDER: return "Retroceder 3 s";
    case ACC_VELOCIDAD:  return "Cambio de velocidad";
    default:             return "-";
    }
}

static inline const char *color_nombre(uint8_t c)
{
    switch (c) {
    case COLOR_NINGUNO:       return "ninguno";
    case COLOR_ROJO:          return "ROJO";
    case COLOR_VERDE:         return "VERDE";
    case COLOR_AZUL:          return "AZUL";
    case COLOR_INDETERMINADO: return "indeterminado";
    default:                  return "?";
    }
}

static inline const char *bif_nombre(uint8_t c)
{
    switch (c) {
    case CMD_BIF_SENSADO:   return "Por sensado (S0 -> izquierda, S4 -> derecha)";
    case CMD_BIF_IZQUIERDA: return "Solo izquierda";
    case CMD_BIF_DERECHA:   return "Solo derecha";
    default:                return "(sin configurar)";
    }
}

#endif /* PROTOCOLO_H */
