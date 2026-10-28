#ifndef TIEMPO_H
#define TIEMPO_H

/*
 * Tiempo monotónico (REQ-87): nunca usar gettimeofday() ni time() para
 * timeouts, porque la hora de pared salta cuando la Pi sincroniza por NTP.
 */

#include <stdint.h>
#include <time.h>

uint64_t ahora_ms(void);
uint64_t ahora_us(void);

/* Duerme ms milisegundos (vuelve antes si llega una señal). */
void espera_ms(unsigned ms);

/* Periodo fijo sin deriva: clock_nanosleep con instante absoluto. */
typedef struct {
    struct timespec siguiente;
    unsigned        ms;
} periodo_t;

void periodo_iniciar(periodo_t *p, unsigned ms);
void periodo_esperar(periodo_t *p);

#endif /* TIEMPO_H */
