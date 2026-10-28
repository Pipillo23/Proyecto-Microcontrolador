#include "interfaz.h"

#include <poll.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static struct termios g_original;
static bool           g_cruda;

bool interfaz_iniciar(void)
{
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &g_original) != 0) {
        return false;
    }
    struct termios cruda = g_original;
    cruda.c_lflag &= (tcflag_t)~(ICANON | ECHO);   /* sin Enter obligatorio, sin eco */
    cruda.c_cc[VMIN]  = 0;
    cruda.c_cc[VTIME] = 0;                          /* ISIG sigue activo: Ctrl+C funciona */
    if (tcsetattr(STDIN_FILENO, TCSANOW, &cruda) != 0) {
        return false;
    }
    g_cruda = true;
    printf("\033[?25l");                            /* ocultar cursor */
    fflush(stdout);
    return true;
}

void interfaz_restaurar(void)
{
    if (g_cruda) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_original);
        g_cruda = false;
        printf("\033[?25h\n");                      /* mostrar cursor */
        fflush(stdout);
    }
}

/*
 * Buffer de teclas: varias teclas pueden llegar en un solo read() (escritura
 * rápida, auto-repetición, retardo de la sesión remota). Se devuelven de a
 * una por llamada, sin perder ninguna (hallazgo V-27).
 */
static unsigned char g_buf[64];
static size_t        g_ini, g_fin;

static bool leer_mas(int timeout_ms)
{
    if (g_fin >= sizeof g_buf) {
        return false;
    }
    struct pollfd p = { .fd = STDIN_FILENO, .events = POLLIN };
    if (poll(&p, 1, timeout_ms) <= 0) {
        return false;                               /* timeout o señal */
    }
    ssize_t n = read(STDIN_FILENO, &g_buf[g_fin], sizeof g_buf - g_fin);
    if (n <= 0) {
        return false;
    }
    g_fin += (size_t)n;
    return true;
}

tecla_t interfaz_leer_tecla(int timeout_ms)
{
    if (g_ini == g_fin) {
        g_ini = g_fin = 0;
        if (!leer_mas(timeout_ms)) {
            return TECLA_NINGUNA;
        }
    }

    /* Una flecha (ESC [ A/B/C/D) puede llegar partida en dos lecturas:
     * si falta el resto, se espera hasta 30 ms a que llegue. */
    if (g_buf[g_ini] == 27u && g_fin - g_ini < 3u) {
        size_t quedan = g_fin - g_ini;
        for (size_t i = 0; i < quedan; i++) {
            g_buf[i] = g_buf[g_ini + i];
        }
        g_ini = 0;
        g_fin = quedan;
        (void)leer_mas(30);
    }

    const unsigned char *k = &g_buf[g_ini];
    size_t disponibles = g_fin - g_ini;

    if (k[0] == 27u) {
        if (disponibles >= 3u && k[1] == '[') {
            g_ini += 3u;
            switch (k[2]) {
            case 'A': return TECLA_ARRIBA;
            case 'B': return TECLA_ABAJO;
            case 'C': return TECLA_DERECHA;
            case 'D': return TECLA_IZQUIERDA;
            default:  return TECLA_OTRA;
            }
        }
        g_ini += 1u;                                /* ESC suelta: se ignora */
        return TECLA_OTRA;
    }

    g_ini += 1u;
    if (k[0] == '\n' || k[0] == '\r') return TECLA_ENTER;
    if (k[0] == 'q' || k[0] == 'Q')   return TECLA_SALIR;
    return TECLA_OTRA;
}

void interfaz_dibujar(const gestor_info_t *g,
                      const char *const *opciones, int n_opciones, int sel,
                      const char *mensaje)
{
    printf("\033[H\033[J");                         /* limpiar pantalla */
    printf("Carro seguidor de línea · interfaz de prueba (SPEC-I2C-01 v1.3)\n");
    printf("--------------------------------------------------------------\n");

    const char *enlace = !g->hay_estado ? "esperando datos del STM32"
                       : g->enlace_ok   ? "OK"
                       :                  "CAÍDO o degradado";
    printf("Enlace I2C : %s\n", enlace);

    if (g->hay_estado) {
        const estado_t *e = &g->estado;
        printf("Estado     : %s", texto_estado(e->estado));
        if (e->action_remain > 0u) {
            printf("  (quedan %u.%u s)", e->action_remain / 10u, e->action_remain % 10u);
        }
        printf("\n");
        printf("Color      : %s\n", texto_color(e->color));
        printf("Conteo     : rojo %lu   verde %lu   azul %lu\n",
               (unsigned long)g->total_r, (unsigned long)g->total_g,
               (unsigned long)g->total_b);
        printf("Motores    : izq %+d %%   der %+d %%   velocidad %s\n",
               e->motor_l, e->motor_r, e->speed_mode == 2u ? "alta" : "baja");
        printf("Config     : %s (hash 0x%04X)\n",
               (e->flags & FLAG_CONFIG_VALID) ? "cargada" : "SIN CONFIGURAR",
               e->config_hash);
        printf("STM32      : encendido hace %lu.%lu s, errores I2C %u\n",
               (unsigned long)(e->uptime_ms / 1000u),
               (unsigned long)((e->uptime_ms % 1000u) / 100u), e->err_cnt);
        if (e->flags & FLAG_COMM_TIMEOUT) {
            printf("AVISO      : el carro se detuvo por pérdida de enlace\n");
        }
        if (e->flags & FLAG_COLOR_SENSOR_FAULT) {
            printf("AVISO      : el sensor de color no responde\n");
        }
    }
    if (g->version_incompatible) {
        printf("ERROR      : el STM32 usa otra versión del protocolo; Inicio bloqueado\n");
    }
    if (g->reset_detectado) {
        printf("AVISO      : el STM32 se reinició (causa 0x%02X); "
               "se reenvió la última configuración\n", g->estado.reset_cause);
    }

    printf("\n");
    for (int i = 0; i < n_opciones; i++) {
        printf("  %s %s\n", (i == sel) ? ">" : " ", opciones[i]);
    }
    printf("\n%s\n", mensaje);
    printf("\nFlechas: mover   Enter: elegir   q o Ctrl+C: salir\n");
    fflush(stdout);
}
