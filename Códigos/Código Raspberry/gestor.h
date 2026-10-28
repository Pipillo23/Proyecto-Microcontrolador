#ifndef GESTOR_H
#define GESTOR_H

/*
 * Hilo gestor I2C: único dueño del bus (REQ-81).
 *  - lee el estado cada 50 ms (REQ-82) y lo publica para la interfaz;
 *  - envía los comandos de la cola con confirmación (REQ-63);
 *  - envía HEARTBEAT si no hubo comandos en 100 ms y la interfaz está
 *    viva (REQ-70, REQ-73), sin esperar confirmación (excepción de REQ-63);
 *  - detecta resets del STM32 y conserva el conteo (REQ-75);
 *  - detecta estado congelado por PUB_CNT (REQ-76);
 *  - al terminar envía STOP (REQ-77).
 */

#include <stdbool.h>
#include <stdint.h>

#include "enlace.h"
#include "trama.h"

typedef struct {
    bool     hay_estado;        /* se ha leído al menos un estado válido */
    estado_t estado;            /* último estado válido */
    bool     enlace_ok;
    bool     reset_detectado;   /* latcheado hasta gestor_limpiar_reset() */
    bool     version_incompatible; /* el STM32 habla otra VER (REQ-83 paso 2) */

    /* Conteo mostrado = acumulado antes de resets + conteo actual (REQ-75) */
    uint32_t total_r, total_g, total_b;

    /* Última orden completada (no incluye HEARTBEAT) */
    unsigned ult_id;            /* cambia cada vez que termina una orden */
    uint8_t  ult_cmd;
    bool     ult_confirmado;
    bool     ult_cancelado;     /* se interrumpió porque se pidió STOP */
    uint8_t  ult_resultado;

    uint32_t heartbeats_fallidos;
} gestor_info_t;

bool gestor_iniciar(enlace_t *en);
void gestor_detener(void);          /* envía STOP y espera al hilo */

/* Encola un comando (no STOP). false si la cola está llena. */
bool gestor_encolar(uint8_t cmd, const uint8_t *datos, uint8_t len);

/* STOP no pasa por la cola: bandera atómica con prioridad máxima. */
void gestor_pedir_stop(void);

/* La interfaz debe llamarla en cada vuelta de su ciclo (REQ-73). */
void gestor_senal_de_vida(void);

void gestor_info(gestor_info_t *copia);
void gestor_limpiar_reset(void);

#endif /* GESTOR_H */
