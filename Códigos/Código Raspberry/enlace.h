#ifndef ENLACE_H
#define ENLACE_H

/*
 * Acceso al bus I2C desde Linux con i2c-dev (REQ-80).
 * Cada write() es una transacción T-W completa y cada read() una T-R,
 * ambas terminadas en STOP (REQ-24).
 *
 * Un enlace_t NO es seguro entre hilos: lo usa un solo hilo (REQ-81).
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "trama.h"

/* Parámetros de confirmación (REQ-63) */
#define ENLACE_REENVIOS     3u   /* además del primer envío */
#define ENLACE_LECTURAS     4u   /* lecturas de confirmación por envío */
#define ENLACE_ESPERA_MS    5u   /* espera antes de cada lectura */

typedef enum {
    ENLACE_OK = 0,
    ENLACE_ERR_IO,       /* el bus falló: ver ultimo_errno */
    ENLACE_ERR_TRAMA     /* llegaron 32 bytes, pero mal formados: ver ultima_dec */
} enlace_res_t;

typedef struct {
    int       fd;
    uint8_t   dir;
    int       ultimo_errno;
    dec_res_t ultima_dec;
    /* Estadísticas, útiles para la prueba P-10 y para la bitácora */
    uint32_t  tx_ok, tx_err;
    uint32_t  rx_ok, rx_err;
    uint32_t  trama_err;
} enlace_t;

/* Devuelven 0 si todo bien; -1 y ultimo_errno si falla. */
int  enlace_abrir(enlace_t *en, const char *dispositivo, uint8_t dir);
int  enlace_cambiar_dir(enlace_t *en, uint8_t dir);
void enlace_cerrar(enlace_t *en);

/* Transacciones crudas: true solo si se transfirieron exactamente n bytes (REQ-64). */
bool enlace_escribir(enlace_t *en, const uint8_t *datos, size_t n);
bool enlace_leer(enlace_t *en, uint8_t *buf, size_t n);

/* Lee y valida la trama de estado. crudo puede ser NULL. */
enlace_res_t enlace_leer_estado(enlace_t *en, estado_t *e,
                                uint8_t crudo[PROTO_LARGO_ESTADO]);

/*
 * Envía un comando y espera su confirmación por LAST_SEQ (REQ-63).
 * true si el STM32 confirmó esa SEQ; *resultado = LAST_RESULT.
 * Los reenvíos usan la MISMA trama y la MISMA SEQ (REQ-62).
 * Si abortar no es NULL y se pone en true, deja de reintentar y devuelve
 * false enseguida: así un STOP pendiente no espera a otro comando (REQ-65).
 */
bool enlace_enviar(enlace_t *en, uint8_t seq, uint8_t cmd,
                   const uint8_t *datos, uint8_t len, uint8_t *resultado,
                   atomic_bool *abortar);

/* Explicación en español de un errno de I2C, con sugerencia de qué revisar. */
const char *enlace_explicar(int err);

#endif /* ENLACE_H */
