#ifndef INTERFAZ_H
#define INTERFAZ_H

/*
 * Interfaz de texto: terminal en modo crudo con termios y lectura de
 * teclas con poll() y timeout, nunca bloqueante sin límite (REQ-89).
 */

#include <stdbool.h>

#include "gestor.h"

typedef enum {
    TECLA_NINGUNA = 0,
    TECLA_ARRIBA,
    TECLA_ABAJO,
    TECLA_IZQUIERDA,
    TECLA_DERECHA,
    TECLA_ENTER,
    TECLA_SALIR,      /* 'q' */
    TECLA_OTRA
} tecla_t;

bool    interfaz_iniciar(void);      /* false si stdin no es una terminal */
void    interfaz_restaurar(void);    /* se puede llamar varias veces */
tecla_t interfaz_leer_tecla(int timeout_ms);

void interfaz_dibujar(const gestor_info_t *g,
                      const char *const *opciones, int n_opciones, int sel,
                      const char *mensaje);

#endif /* INTERFAZ_H */
