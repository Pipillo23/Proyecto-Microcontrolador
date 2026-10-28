/*
 * Programa principal de la Raspberry Pi (SPEC-I2C-01 v1.3).
 *
 * Esqueleto funcional: menú con flechas y Enter (Inicio, Fin, config de
 * prueba, Salir), estado del STM32 en pantalla y supervisión del enlace.
 * El menú completo de configuración color-acción queda para la spec de
 * interfaz.
 *
 * Uso: ./seguidor [dispositivo]      (por defecto /dev/i2c-1)
 * Salir: opción "Salir", tecla q o Ctrl+C. Siempre envía STOP al salir.
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "enlace.h"
#include "gestor.h"
#include "interfaz.h"
#include "tiempo.h"

#define DISPOSITIVO_POR_DEFECTO "/dev/i2c-1"
#define REDIBUJO_MS             100u

/* REQ-77: el manejador solo levanta la bandera */
static volatile sig_atomic_t g_senal = 0;

static void al_recibir_senal(int s)
{
    (void)s;
    g_senal = 1;
}

static void instalar_senales(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = al_recibir_senal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;               /* sin SA_RESTART: poll() vuelve con EINTR */
    sigaction(SIGINT, &sa, NULL);  /* Ctrl+C */
    sigaction(SIGTERM, &sa, NULL); /* kill */
    sigaction(SIGHUP, &sa, NULL);  /* se cerró la sesión remota */
}

static bool en_movimiento(uint8_t estado)
{
    return estado == ESTADO_RUN || estado == ESTADO_ACT_BRAKE ||
           estado == ESTADO_ACT_REVERSE || estado == ESTADO_FORK;
}

/* Config de prueba: rojo=frenar, verde=retroceder, azul=cambio de
 * velocidad, bifurcación a la izquierda, 40 % y 80 % (Anexo A). */
static const uint8_t CONFIG_PRUEBA[CONFIG_LARGO] = {
    ACCION_FRENAR, ACCION_RETROCEDER, ACCION_CAMBIO_VEL,
    FORK_IZQUIERDA, 40u, 80u
};

enum { OP_INICIO, OP_FIN, OP_CONFIG, OP_SALIR, N_OPCIONES };

static const char *const OPCIONES[N_OPCIONES] = {
    "Inicio",
    "Fin",
    "Enviar configuración de prueba",
    "Salir"
};

static void ejecutar_opcion(int op, const gestor_info_t *info,
                            bool *salir, char *msj, size_t tam)
{
    switch (op) {
    case OP_INICIO:
        if (info->version_incompatible) {             /* REQ-83 paso 2 */
            snprintf(msj, tam, "Inicio bloqueado: el STM32 usa otra versión del "
                               "protocolo. Actualice el firmware o el programa.");
        } else if (gestor_encolar(CMD_START, NULL, 0)) {
            snprintf(msj, tam, "Inicio enviado; esperando confirmación...");
        } else {
            snprintf(msj, tam, "Cola llena; intente de nuevo.");
        }
        break;
    case OP_FIN:
        gestor_pedir_stop();
        snprintf(msj, tam, "Fin enviado.");
        break;
    case OP_CONFIG:
        /* REQ-84: primera barrera, sin enviar nada */
        if (info->hay_estado && en_movimiento(info->estado.estado)) {
            snprintf(msj, tam, "ERROR: no se puede cambiar la configuración "
                               "con el carro en movimiento. Presione Fin primero.");
        } else if (gestor_encolar(CMD_SET_CONFIG, CONFIG_PRUEBA, CONFIG_LARGO)) {
            snprintf(msj, tam, "Configuración de prueba enviada; esperando confirmación...");
        } else {
            snprintf(msj, tam, "Cola llena; intente de nuevo.");
        }
        break;
    case OP_SALIR:
        *salir = true;
        break;
    default:
        break;
    }
}

static void mensaje_resultado(const gestor_info_t *info, char *msj, size_t tam)
{
    if (info->ult_cancelado) {
        snprintf(msj, tam, "%s cancelado porque se pidió Fin.", texto_cmd(info->ult_cmd));
    } else if (!info->ult_confirmado) {
        snprintf(msj, tam, "%s: sin confirmación del STM32 (revise el enlace).",
                 texto_cmd(info->ult_cmd));
    } else if (info->ult_cmd == CMD_SET_CONFIG && info->ult_resultado == RES_ERR_STATE) {
        /* REQ-84: segunda barrera, el STM32 la rechazó */
        snprintf(msj, tam, "ERROR: no se puede cambiar la configuración "
                           "con el carro en movimiento.");
    } else if (info->ult_cmd == CMD_START && info->ult_resultado == RES_ERR_NO_CONFIG) {
        snprintf(msj, tam, "START rechazado: primero envíe una configuración.");
    } else {
        snprintf(msj, tam, "%s: %s", texto_cmd(info->ult_cmd),
                 texto_resultado(info->ult_resultado));
    }
}

int main(int argc, char **argv)
{
    const char *dispositivo = (argc > 1) ? argv[1] : DISPOSITIVO_POR_DEFECTO;

    instalar_senales();   /* antes de crear hilos */

    enlace_t en;
    if (enlace_abrir(&en, dispositivo, PROTO_DIR_STM32) != 0) {
        fprintf(stderr, "No se pudo abrir %s: %s\n", dispositivo,
                enlace_explicar(en.ultimo_errno));
        return EXIT_FAILURE;
    }
    if (!interfaz_iniciar()) {
        fprintf(stderr, "Este programa necesita una terminal interactiva.\n");
        enlace_cerrar(&en);
        return EXIT_FAILURE;
    }
    atexit(interfaz_restaurar);   /* restaurar la terminal pase lo que pase */

    if (!gestor_iniciar(&en)) {
        interfaz_restaurar();
        fprintf(stderr, "No se pudo crear el hilo gestor.\n");
        enlace_cerrar(&en);
        return EXIT_FAILURE;
    }

    int      sel = 0;
    bool     salir = false;
    char     msj[160] = "Listo.";
    unsigned ult_id_visto = 0;
    uint64_t proximo_dibujo = 0;

    while (!salir && !g_senal) {
        gestor_senal_de_vida();                    /* REQ-73 */
        tecla_t t = interfaz_leer_tecla(50);        /* nunca más de 50 ms */

        gestor_info_t info;
        gestor_info(&info);

        switch (t) {
        case TECLA_ARRIBA: sel = (sel + N_OPCIONES - 1) % N_OPCIONES; break;
        case TECLA_ABAJO:  sel = (sel + 1) % N_OPCIONES;              break;
        case TECLA_ENTER:  ejecutar_opcion(sel, &info, &salir, msj, sizeof msj); break;
        case TECLA_SALIR:  salir = true;                               break;
        default:                                                       break;
        }

        if (info.ult_id != ult_id_visto) {
            ult_id_visto = info.ult_id;
            mensaje_resultado(&info, msj, sizeof msj);
        }

        if (t != TECLA_NINGUNA || ahora_ms() >= proximo_dibujo) {
            interfaz_dibujar(&info, OPCIONES, N_OPCIONES, sel, msj);
            proximo_dibujo = ahora_ms() + REDIBUJO_MS;
        }
    }

    gestor_detener();          /* envía STOP y espera al hilo (REQ-77) */
    interfaz_restaurar();
    printf("Fin del programa. Transacciones: escrituras %lu ok / %lu error, "
           "lecturas %lu ok / %lu error, tramas inválidas %lu.\n",
           (unsigned long)en.tx_ok, (unsigned long)en.tx_err,
           (unsigned long)en.rx_ok, (unsigned long)en.rx_err,
           (unsigned long)en.trama_err);
    enlace_cerrar(&en);
    return EXIT_SUCCESS;
}
