/* =============================================================================
 * carro_pi.c  -  Interfaz de consola + maestro I2C   (Raspberry Pi -> STM32)
 * -----------------------------------------------------------------------------
 * Archivos necesarios en la Pi:  carro_pi.c  y  protocolo.h  (misma carpeta)
 *
 * Preparar la Pi (una sola vez):
 *   sudo raspi-config  -> Interface Options -> I2C -> Enable
 *   En /boot/firmware/config.txt:  dtparam=i2c_arm_baudrate=100000
 *   sudo usermod -aG i2c $USER   (cerrar sesion y volver a entrar)
 *   Comprobar cableado:  i2cdetect -y 1   (debe aparecer 42)
 *
 * Compilar:
 *   gcc -std=c11 -Wall -Wextra -O2 -o carro_pi carro_pi.c
 *
 * Ejecutar:
 *   ./carro_pi                  interfaz con el STM32 real (/dev/i2c-1, 0x42)
 *   ./carro_pi --sim            interfaz con un STM32 SIMULADO (sin hardware)
 *   ./carro_pi --prueba         SECCION DE PRUEBAS sobre el bus real
 *   ./carro_pi --prueba --sim   SECCION DE PRUEBAS con el STM32 simulado
 *   Opciones:  --bus /dev/i2c-N   -n <lecturas de prueba>
 *
 * Teclas: flechas = mover, Enter = elegir, Esc = volver, q = salir (envia STOP)
 *
 * Diseno: un solo hilo. Cada 20 ms se lee la trama de estado, se envian los
 * comandos pendientes y, si no se envio nada en 100 ms, un HEARTBEAT. Si la
 * interfaz se cuelga, los heartbeats se detienen y el STM32 para el carro en
 * 500 ms (no hace falta un watchdog aparte entre hilos).
 * ===========================================================================*/
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#include "protocolo.h"

/* ---- Parametros de la Pi ------------------------------------------------ */
#define PERIODO_TICK_MS     20u    /* lectura de estado + envios               */
#define T_CONFIRMACION_MS   60u    /* espera de LAST_SEQ antes de reenviar      */
#define MAX_ENVIOS          4      /* 1 envio + 3 reenvios                      */
#define T_ENLACE_CAIDO_MS   500u   /* sin lecturas validas -> SIN CONEXION      */
#define T_ESC_MS            30u    /* espera para distinguir Esc de una flecha  */
#define PUB_IGUALES_MAX     5      /* PUB_CNT congelado 5 lecturas -> degradado */

/* ============================================================================
 * 1. TIEMPO (CLOCK_MONOTONIC: no salta si la Pi ajusta la hora por red)
 * ==========================================================================*/
static uint64_t ahora_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000u + (uint64_t)t.tv_nsec / 1000000u;
}

static void dormir_ms(unsigned ms)
{
    struct timespec t = { (time_t)(ms / 1000u), (long)(ms % 1000u) * 1000000L };
    while (nanosleep(&t, &t) == -1 && errno == EINTR) { }
}

/* ============================================================================
 * 2. BUS I2C  (real con i2c-dev, o simulado)
 *    write() = una transaccion START|0x42+W|datos|STOP
 *    read()  = una transaccion START|0x42+R|datos|STOP   (sin repeated-start)
 * ==========================================================================*/
static int bus_fd = -1;
static int modo_sim = 0;

static void sim_iniciar(void);
static int  sim_escribir(const uint8_t *b, size_t n);
static int  sim_leer(uint8_t *b, size_t n);

static int bus_abrir(const char *dev)
{
    if (modo_sim) { sim_iniciar(); return 0; }
    bus_fd = open(dev, O_RDWR);
    if (bus_fd < 0) return -1;
    if (ioctl(bus_fd, I2C_SLAVE, I2C_DIR_STM32) < 0) {
        int e = errno;
        close(bus_fd);
        bus_fd = -1;
        errno = e;
        return -1;
    }
    return 0;
}

static void bus_cerrar(void)
{
    if (bus_fd >= 0) close(bus_fd);
    bus_fd = -1;
}

/* Devuelven 0 si se transfirio exactamente n bytes, -1 si no (errno indica la causa) */
static int bus_escribir(const uint8_t *b, size_t n)
{
    if (modo_sim) return sim_escribir(b, n);
    ssize_t r = write(bus_fd, b, n);
    if (r == (ssize_t)n) return 0;
    if (r >= 0) errno = EIO;
    return -1;
}

static int bus_leer(uint8_t *b, size_t n)
{
    if (modo_sim) return sim_leer(b, n);
    ssize_t r = read(bus_fd, b, n);
    if (r == (ssize_t)n) return 0;
    if (r >= 0) errno = EIO;
    return -1;
}

static const char *explicar_errno(int e)
{
    switch (e) {
    case ENOENT:    return "no existe el bus: habilita I2C con raspi-config";
    case EACCES:    return "sin permiso: agrega tu usuario al grupo i2c o usa sudo";
    case ENXIO:
    case EREMOTEIO: return "el STM32 no respondio (NACK): revisa 0x42, SDA/SCL/GND y que el firmware corra";
    case ETIMEDOUT: return "tiempo agotado: bus bloqueado o clock stretching muy largo";
    case EIO:       return "error de E/S en el bus";
    case EAGAIN:    return "bus ocupado, reintentar";
    default:        return strerror(e);
    }
}

/* ============================================================================
 * 3. ENLACE: lectura de estado, envio con confirmacion, heartbeat
 * ==========================================================================*/
typedef struct {
    estado_t st;                 /* ultima trama de estado valida              */
    int      sincronizado;       /* ya se leyo al menos una trama valida       */
    int      version_mala;
    uint64_t t_ult_ok, t_ult_err, t_ult_envio;
    uint32_t lecturas_ok, errores;
    int      pub_iguales;
    uint8_t  sig_seq;

    int      pend_activo;        /* comando esperando confirmacion             */
    uint8_t  pend_cmd, pend_seq, pend_trama[CMD_LEN];
    uint64_t pend_t;
    int      pend_envios;

    uint8_t  cola[8];
    int      cola_n;
    int      stop_pedido;        /* STOP no pasa por la cola: tiene prioridad   */

    uint8_t  cfg_color_deseada;  /* lo que eligio el usuario                    */
    uint8_t  cfg_bif_deseada;
    int      config_reenviar;
    int      start_bloqueado;    /* START elegido: no se puede repetir hasta STOP */
} enlace_t;

static enlace_t L;

/* ---- Mensajes para la interfaz ------------------------------------------ */
static char msg[200];
static int  msg_error = 0;
static int  redibujar = 1;

static void mensaje(int es_error, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    msg_error = es_error;
    redibujar = 1;
}

#define MSG_ERR_MOVIMIENTO \
    "ERROR: no se puede configurar con el carro en movimiento. Presione STOP primero."

static uint8_t seq_siguiente(void)
{
    uint8_t s = L.sig_seq;
    L.sig_seq = (uint8_t)(s >= 255u ? 1u : s + 1u);   /* 1..255, el 0 se salta */
    return s;
}

static void encolar(uint8_t cmd)
{
    for (int i = 0; i < L.cola_n; i++)
        if (L.cola[i] == cmd) return;                  /* ya estaba en cola */
    if (L.cola_n < (int)sizeof L.cola) L.cola[L.cola_n++] = cmd;
}

static void quitar_de_cola(uint8_t cmd)
{
    int j = 0;
    for (int i = 0; i < L.cola_n; i++)
        if (L.cola[i] != cmd) L.cola[j++] = L.cola[i];
    L.cola_n = j;
}

static void enviar_pendiente(uint64_t now)
{
    L.pend_envios++;
    L.pend_t = now;
    L.t_ult_envio = now;
    if (bus_escribir(L.pend_trama, CMD_LEN) != 0) {
        L.errores++;
        L.t_ult_err = now;
    }
}

static void iniciar_envio(uint8_t cmd, uint64_t now)
{
    L.pend_cmd = cmd;
    L.pend_seq = seq_siguiente();
    cmd_armar(L.pend_trama, L.pend_seq, cmd);
    L.pend_envios = 0;
    L.pend_activo = 1;
    enviar_pendiente(now);
}

static void comando_confirmado(uint8_t res)
{
    uint8_t c = L.pend_cmd;
    L.pend_activo = 0;

    if (c == CMD_START) {
        if (res == RES_OK) mensaje(0, "START confirmado: el carro esta en marcha.");
        else {
            L.start_bloqueado = 0;
            if (res == RES_ERR_SIN_CONFIG)
                mensaje(1, "ERROR: START rechazado, falta configurar colores o bifurcacion.");
            else if (res == RES_ERR_ESTADO)
                mensaje(1, "ERROR: START rechazado: el carro esta en %s, presione STOP primero.",
                        estado_nombre(L.st.estado));
            else
                mensaje(1, "ERROR: START rechazado (%s).", resultado_nombre(res));
        }
    } else if (c == CMD_STOP) {
        mensaje(0, "STOP confirmado: carro detenido.");
    } else if (cmd_es_color(c) || cmd_es_bif(c)) {
        if (res == RES_OK) {
            if (cmd_es_color(c)) L.cfg_color_deseada = c;
            else                 L.cfg_bif_deseada = c;
            mensaje(0, "Configuracion aplicada: %s (0x%02X).", cmd_nombre(c), c);
        } else if (res == RES_ERR_ESTADO) {
            mensaje(1, MSG_ERR_MOVIMIENTO);
        } else {
            mensaje(1, "ERROR: configuracion rechazada (%s).", resultado_nombre(res));
        }
    }
}

static void comando_sin_confirmar(void)
{
    uint8_t c = L.pend_cmd;
    L.pend_activo = 0;
    mensaje(1, "ERROR: el STM32 no confirmo %s tras %d intentos.", cmd_nombre(c), MAX_ENVIOS);
    if (c == CMD_START) L.start_bloqueado = 0;
    if (c == CMD_STOP)  L.stop_pedido = 1;     /* STOP se sigue intentando siempre */
}

static void procesar_estado(const estado_t *e, uint64_t now)
{
    int primera = !L.sincronizado;

    if (!primera) {
        L.pub_iguales = (e->pub_cnt == L.st.pub_cnt) ? L.pub_iguales + 1 : 0;

        /* Reinicio del STM32: pierde la config o LAST_SEQ vuelve a 0 */
        if ((L.st.cfg_color && !e->cfg_color) || (L.st.ult_seq && !e->ult_seq)) {
            mensaje(1, "AVISO: se detecto un reinicio del STM32; se reenvia la configuracion.");
            L.config_reenviar = 1;
            L.start_bloqueado = 0;
        }
    }

    L.st = *e;
    L.t_ult_ok = now;
    L.lecturas_ok++;
    L.version_mala = 0;

    if (primera) {
        L.sincronizado = 1;
        L.sig_seq = (uint8_t)(e->ult_seq >= 255u ? 1u : e->ult_seq + 1u);
        if (e->cfg_color) L.cfg_color_deseada = e->cfg_color;   /* adoptar la config vigente */
        if (e->cfg_bif)   L.cfg_bif_deseada = e->cfg_bif;
        L.config_reenviar = 1;
        if (estado_en_movimiento(e->estado)) {
            L.stop_pedido = 1;
            mensaje(1, "AVISO: el carro ya se movia al conectar; se envia STOP por seguridad.");
        } else {
            mensaje(0, "Comunicacion establecida con el STM32.");
        }
    }

    /* Reenviar la configuracion si el STM32 no la tiene (solo en IDLE) */
    if (L.config_reenviar && e->estado == EST_IDLE && !L.pend_activo && L.cola_n == 0) {
        if (e->cfg_color != L.cfg_color_deseada) encolar(L.cfg_color_deseada);
        if (e->cfg_bif != L.cfg_bif_deseada)     encolar(L.cfg_bif_deseada);
        L.config_reenviar = 0;
    }
}

static void enlace_tick(uint64_t now)
{
    uint8_t b[ST_LEN];
    int leido = 0;

    /* 1) Leer la trama de estado */
    if (bus_leer(b, ST_LEN) == 0) {
        estado_t e;
        int r = estado_decodificar(b, &e);
        if (r == DEC_OK) {
            procesar_estado(&e, now);
            leido = 1;
        } else {
            L.errores++;
            L.t_ult_err = now;
            if (r == DEC_ERR_VERSION) L.version_mala = 1;
        }
    } else {
        L.errores++;
        L.t_ult_err = now;
    }

    /* 2) Envios: STOP primero, luego confirmacion del pendiente, luego la cola */
    if (L.stop_pedido) {
        L.stop_pedido = 0;
        L.pend_activo = 0;                 /* cancela cualquier otro envio en curso */
        quitar_de_cola(CMD_START);
        iniciar_envio(CMD_STOP, now);
    } else if (L.pend_activo) {
        if (leido && L.st.ult_seq == L.pend_seq) {
            comando_confirmado(L.st.ult_resultado);
        } else if (now - L.pend_t >= T_CONFIRMACION_MS) {
            if (L.pend_envios < MAX_ENVIOS) enviar_pendiente(now);   /* misma SEQ */
            else                            comando_sin_confirmar();
        }
    } else if (L.cola_n > 0 && L.sincronizado) {
        uint8_t c = L.cola[0];
        memmove(L.cola, L.cola + 1, (size_t)(L.cola_n - 1));
        L.cola_n--;
        iniciar_envio(c, now);
    }

    /* 3) HEARTBEAT si no se envio nada en 100 ms (no espera confirmacion) */
    if (now - L.t_ult_envio >= T_HEARTBEAT_MS) {
        uint8_t t[CMD_LEN];
        cmd_armar(t, 0, CMD_HEARTBEAT);
        if (bus_escribir(t, CMD_LEN) != 0) {
            L.errores++;
            L.t_ult_err = now;
        }
        L.t_ult_envio = now;
    }
}

/* 2 = OK, 1 = DEGRADADO, 0 = SIN CONEXION */
static int enlace_nivel(uint64_t now)
{
    if (!L.sincronizado || L.version_mala || now - L.t_ult_ok > T_ENLACE_CAIDO_MS) return 0;
    if (now - L.t_ult_ok > 3u * PERIODO_TICK_MS || now - L.t_ult_err < 1000u ||
        L.pub_iguales >= PUB_IGUALES_MAX)
        return 1;
    return 2;
}

/* ---- Acciones del usuario ---------------------------------------------- */
static void pedir_start(void)
{
    uint64_t now = ahora_ms();
    if (L.start_bloqueado) {
        mensaje(1, "ERROR: START ya fue elegido. Presione STOP antes de volver a iniciar.");
    } else if (L.version_mala) {
        mensaje(1, "ERROR: version de firmware incompatible; START bloqueado.");
    } else if (enlace_nivel(now) == 0) {
        mensaje(1, "ERROR: sin comunicacion con el STM32; no se puede enviar START.");
    } else {
        L.start_bloqueado = 1;
        encolar(CMD_START);
        mensaje(0, "Enviando START...");
    }
}

static void pedir_stop(void)
{
    L.stop_pedido = 1;
    L.start_bloqueado = 0;
    mensaje(0, "Enviando STOP...");
}

static void pedir_config(uint8_t cmd)
{
    uint64_t now = ahora_ms();
    if (enlace_nivel(now) == 0) {
        mensaje(1, "ERROR: sin comunicacion con el STM32; no se envio la configuracion.");
    } else if (estado_en_movimiento(L.st.estado) || (L.start_bloqueado && L.st.estado == EST_IDLE)) {
        mensaje(1, MSG_ERR_MOVIMIENTO);              /* no se envia nada al STM32 */
    } else if (L.st.estado != EST_IDLE) {
        mensaje(1, "ERROR: el carro esta en %s. Presione STOP para volver a IDLE.",
                estado_nombre(L.st.estado));
    } else {
        encolar(cmd);
        mensaje(0, "Enviando configuracion %s (0x%02X)...", cmd_nombre(cmd), cmd);
    }
}

/* ============================================================================
 * 4. TERMINAL Y TECLADO
 * ==========================================================================*/
static struct termios term_original;
static int term_modificada = 0;
static volatile sig_atomic_t senal = 0;

static void manejador_senal(int s) { senal = s; }   /* solo levanta la bandera */

static void terminal_restaurar(void)
{
    if (term_modificada) {
        tcsetattr(STDIN_FILENO, TCSANOW, &term_original);
        term_modificada = 0;
        const char *s = "\033[?25h\n";               /* volver a mostrar el cursor */
        if (write(STDOUT_FILENO, s, strlen(s)) < 0) { }
    }
}

static void terminal_cruda(void)
{
    if (!isatty(STDIN_FILENO)) return;
    if (tcgetattr(STDIN_FILENO, &term_original) != 0) return;
    struct termios t = term_original;
    t.c_lflag &= (tcflag_t)~(ICANON | ECHO);     /* sin buffer de linea ni eco */
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    term_modificada = 1;
    atexit(terminal_restaurar);
}

enum { K_NADA, K_ARRIBA, K_ABAJO, K_ENTER, K_ESC, K_SALIR };

static uint8_t  kb[64];
static int      kb_n = 0;
static uint64_t kb_t = 0;           /* cuando llego el ultimo byte */
static int      stdin_cerrado = 0;

static void leer_teclado(void)
{
    uint8_t tmp[32];
    ssize_t r = read(STDIN_FILENO, tmp, sizeof tmp);
    if (r == 0) { stdin_cerrado = 1; return; }
    if (r < 0) return;
    for (ssize_t i = 0; i < r && kb_n < (int)sizeof kb; i++) kb[kb_n++] = tmp[i];
    kb_t = ahora_ms();
}

static void kb_consumir(int n)
{
    memmove(kb, kb + n, (size_t)(kb_n - n));
    kb_n -= n;
}

/* Saca una tecla del buffer; varias teclas en un mismo read() no se pierden */
static int tecla_siguiente(uint64_t now)
{
    if (kb_n == 0) return K_NADA;
    uint8_t c = kb[0];
    if (c == 27) {
        if (kb_n >= 3 && (kb[1] == '[' || kb[1] == 'O')) {
            uint8_t f = kb[2];
            kb_consumir(3);
            if (f == 'A') return K_ARRIBA;
            if (f == 'B') return K_ABAJO;
            return K_NADA;                          /* otra flecha o tecla especial */
        }
        if (kb_n == 1 || (kb_n == 2 && (kb[1] == '[' || kb[1] == 'O'))) {
            if (now - kb_t < T_ESC_MS) return K_NADA;   /* puede ser una flecha partida */
            kb_consumir(kb_n);
            return K_ESC;
        }
        kb_consumir(1);
        return K_ESC;
    }
    kb_consumir(1);
    if (c == '\r' || c == '\n') return K_ENTER;
    if (c == 'q' || c == 'Q')   return K_SALIR;
    if (c == 'w' || c == 'W')   return K_ARRIBA;    /* alternativa si la terminal falla */
    if (c == 's' || c == 'S')   return K_ABAJO;
    return K_NADA;
}

/* ============================================================================
 * 5. MENUS
 * ==========================================================================*/
enum { P_PRINCIPAL, P_SETTINGS, P_BIFURCACION, P_COL_ROJO, P_COL_VERDE, P_COL_CONFIRMAR };

static int     pantalla = P_PRINCIPAL;
static int     sel = 0;
static uint8_t acc_rojo = ACC_FRENAR, acc_verde = ACC_RETROCEDER;
static int     salir = 0;

static uint8_t acc_restante_verde(int i)        /* las 2 acciones que no tiene el rojo */
{
    int k = 0;
    for (uint8_t a = ACC_FRENAR; a <= ACC_VELOCIDAD; a++)
        if (a != acc_rojo) { if (k == i) return a; k++; }
    return ACC_NINGUNA;
}

static uint8_t acc_azul(void) { return (uint8_t)(6u - acc_rojo - acc_verde); }  /* 1+2+3 = 6 */

static int n_opciones(void)
{
    switch (pantalla) {
    case P_PRINCIPAL:     return 3;
    case P_SETTINGS:      return 2;
    case P_BIFURCACION:   return 3;
    case P_COL_ROJO:      return 3;
    case P_COL_VERDE:     return 2;
    case P_COL_CONFIRMAR: return 2;
    default:              return 1;
    }
}

static void ir_a(int p, int s) { pantalla = p; sel = s; }

static void manejar_tecla(int k)
{
    int n = n_opciones();
    switch (k) {
    case K_ARRIBA: sel = (sel + n - 1) % n; return;
    case K_ABAJO:  sel = (sel + 1) % n;     return;
    case K_SALIR:  salir = 1;               return;
    case K_ESC:
        switch (pantalla) {
        case P_SETTINGS:      ir_a(P_PRINCIPAL, 1); break;
        case P_BIFURCACION:   ir_a(P_SETTINGS, 0); break;
        case P_COL_ROJO:      ir_a(P_SETTINGS, 1); break;
        case P_COL_VERDE:     ir_a(P_COL_ROJO, acc_rojo - 1); break;
        case P_COL_CONFIRMAR: ir_a(P_COL_VERDE, 0); break;
        default: break;
        }
        return;
    case K_ENTER:
        switch (pantalla) {
        case P_PRINCIPAL:
            if (sel == 0) pedir_start();
            else if (sel == 1) ir_a(P_SETTINGS, 0);
            else pedir_stop();
            break;
        case P_SETTINGS:
            if (sel == 0) ir_a(P_BIFURCACION, 0);
            else ir_a(P_COL_ROJO, 0);
            break;
        case P_BIFURCACION:
            pedir_config((uint8_t)(CMD_BIF_PRIMERO + sel));
            break;
        case P_COL_ROJO:
            acc_rojo = (uint8_t)(ACC_FRENAR + sel);
            ir_a(P_COL_VERDE, 0);
            break;
        case P_COL_VERDE:
            acc_verde = acc_restante_verde(sel);
            ir_a(P_COL_CONFIRMAR, 0);
            break;
        case P_COL_CONFIRMAR:
            if (sel == 0) {
                uint8_t c = color_cmd_desde_acciones(acc_rojo, acc_verde, acc_azul());
                if (c) pedir_config(c);
                ir_a(P_SETTINGS, 1);
            } else {
                ir_a(P_SETTINGS, 1);
                mensaje(0, "Configuracion de colores cancelada.");
            }
            break;
        default: break;
        }
        return;
    default: return;
    }
}

/* ============================================================================
 * 6. PANTALLA
 * ==========================================================================*/
static char   pbuf[8192];
static size_t pn;

static void p(const char *fmt, ...)
{
    if (pn >= sizeof pbuf) return;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(pbuf + pn, sizeof pbuf - pn, fmt, ap);
    va_end(ap);
    if (r > 0) pn += (size_t)r;
    if (pn > sizeof pbuf) pn = sizeof pbuf;
}

#define FIN "\033[K\r\n"          /* borra el resto de la linea y salta */
#define VERDE_T "\033[32m"
#define AMAR_T  "\033[33m"
#define ROJO_T  "\033[31m"
#define NEGR_T  "\033[1m"
#define NORM_T  "\033[0m"

static void opcion(int i, const char *texto, const char *extra)
{
    p("  %s %s%s" NORM_T "%s" FIN, sel == i ? ">" : " ", sel == i ? NEGR_T : "", texto,
      extra ? extra : "");
}

static void dibujar(uint64_t now)
{
    pn = 0;
    p("\033[H");
    p(NEGR_T "=== CARRO SEGUIDOR DE LINEA | Raspberry Pi -> STM32 (I2C 0x%02X)%s ===" NORM_T FIN,
      I2C_DIR_STM32, modo_sim ? "  [STM32 SIMULADO]" : "");

    int nv = enlace_nivel(now);
    const char *txt = nv == 2 ? VERDE_T "OK" : nv == 1 ? AMAR_T "DEGRADADO" : ROJO_T "SIN CONEXION";
    p("Comunicacion : %s" NORM_T "   (lecturas OK %u, errores %u)%s" FIN, txt,
      (unsigned)L.lecturas_ok, (unsigned)L.errores,
      L.version_mala ? ROJO_T "  VERSION INCOMPATIBLE" NORM_T : "");

    if (!L.sincronizado) {
        p("Esperando la primera trama de estado del STM32..." FIN);
    } else {
        const estado_t *e = &L.st;
        p("Vista STM32  : %s" FIN, (e->banderas & BAN_COM_PERDIDA)
                                       ? ROJO_T "detuvo el carro por perdida de enlace" NORM_T
                                       : "recibe heartbeats");
        p("Estado carro : " NEGR_T "%-18s" NORM_T " Color bajo sensor: %s%s%s" FIN,
          estado_nombre(e->estado), color_nombre(e->color),
          (e->banderas & BAN_LINEA_PERDIDA) ? "   [linea perdida]" : "",
          (e->banderas & BAN_SENSOR_COLOR_FALLA) ? ROJO_T "   [sensor de color en falla]" NORM_T : "");
        p("Conteo       : Rojo %5u    Verde %5u    Azul %5u" FIN,
          (unsigned)e->cuenta_r, (unsigned)e->cuenta_g, (unsigned)e->cuenta_b);
        if (e->cfg_color)
            p("Colores      : [0x%02X] Rojo=%s, Verde=%s, Azul=%s" FIN, e->cfg_color,
              accion_nombre(color_accion(e->cfg_color, COLOR_ROJO)),
              accion_nombre(color_accion(e->cfg_color, COLOR_VERDE)),
              accion_nombre(color_accion(e->cfg_color, COLOR_AZUL)));
        else
            p("Colores      : (sin configurar)" FIN);
        if (e->cfg_bif) p("Bifurcacion  : [0x%02X] %s" FIN, e->cfg_bif, bif_nombre(e->cfg_bif));
        else            p("Bifurcacion  : (sin configurar)" FIN);
    }
    p("----------------------------------------------------------------------" FIN);

    int mov = estado_en_movimiento(L.st.estado);
    const char *aviso_cfg = mov ? ROJO_T "  (carro en movimiento: no se puede aplicar)" NORM_T : "";

    switch (pantalla) {
    case P_PRINCIPAL:
        p(NEGR_T "MENU PRINCIPAL" NORM_T FIN);
        opcion(0, "START", L.start_bloqueado ? AMAR_T "   (bloqueado hasta elegir STOP)" NORM_T : NULL);
        opcion(1, "SETTINGS", NULL);
        opcion(2, "STOP", NULL);
        break;
    case P_SETTINGS:
        p(NEGR_T "SETTINGS" NORM_T "%s" FIN, aviso_cfg);
        opcion(0, "Configurar bifurcaciones", NULL);
        opcion(1, "Configurar colores", NULL);
        break;
    case P_BIFURCACION:
        p(NEGR_T "SETTINGS > Configurar bifurcaciones" NORM_T "%s" FIN, aviso_cfg);
        for (int i = 0; i < 3; i++) {
            uint8_t c = (uint8_t)(CMD_BIF_PRIMERO + i);
            char t[96];
            snprintf(t, sizeof t, "%-46s [0x%02X]", bif_nombre(c), c);
            opcion(i, t, L.st.cfg_bif == c ? VERDE_T "  <- activa" NORM_T : NULL);
        }
        break;
    case P_COL_ROJO:
        p(NEGR_T "SETTINGS > Configurar colores (paso 1/3): accion para el ROJO" NORM_T "%s" FIN, aviso_cfg);
        for (int i = 0; i < 3; i++) opcion(i, accion_nombre((uint8_t)(ACC_FRENAR + i)), NULL);
        break;
    case P_COL_VERDE:
        p(NEGR_T "SETTINGS > Configurar colores (paso 2/3): accion para el VERDE" NORM_T "%s" FIN, aviso_cfg);
        p("    Rojo = %s" FIN, accion_nombre(acc_rojo));
        for (int i = 0; i < 2; i++) opcion(i, accion_nombre(acc_restante_verde(i)), NULL);
        break;
    case P_COL_CONFIRMAR: {
        uint8_t c = color_cmd_desde_acciones(acc_rojo, acc_verde, acc_azul());
        p(NEGR_T "SETTINGS > Configurar colores (paso 3/3): confirmar" NORM_T "%s" FIN, aviso_cfg);
        p("    Rojo = %s, Verde = %s, Azul = %s" FIN, accion_nombre(acc_rojo),
          accion_nombre(acc_verde), accion_nombre(acc_azul()));
        p("    Trama: %s (0x%02X)" FIN, cmd_nombre(c), c);
        opcion(0, "Enviar configuracion", NULL);
        opcion(1, "Cancelar", NULL);
        break;
    }
    default: break;
    }

    p("----------------------------------------------------------------------" FIN);
    p("%s%s" NORM_T FIN, msg_error ? ROJO_T : VERDE_T, msg);
    p("Flechas: mover   Enter: elegir   Esc: volver   q: salir (envia STOP)" FIN);
    p("\033[J");
    if (write(STDOUT_FILENO, pbuf, pn) < 0) { }
}

/* ============================================================================
 * 7. PROGRAMA PRINCIPAL DE LA INTERFAZ
 * ==========================================================================*/
static void enviar_stop_al_salir(void)
{
    for (int i = 0; i < 3; i++) {             /* STOP es idempotente: 3 envios */
        uint8_t t[CMD_LEN];
        cmd_armar(t, seq_siguiente(), CMD_STOP);
        bus_escribir(t, CMD_LEN);
        dormir_ms(20);
    }
}

static int bucle_interfaz(void)
{
    terminal_cruda();
    const char *ini = "\033[2J\033[?25l";      /* limpiar pantalla, ocultar cursor */
    if (write(STDOUT_FILENO, ini, strlen(ini)) < 0) { }

    L.cfg_color_deseada = CMD_COL_FRE_RET_VEL; /* por defecto si el STM32 no tiene config */
    L.cfg_bif_deseada   = CMD_BIF_SENSADO;
    mensaje(0, "Conectando con el STM32...");

    uint64_t prox_tick = ahora_ms(), prox_dibujo = 0;
    while (!senal && !salir) {
        uint64_t now = ahora_ms();
        int espera = prox_tick > now ? (int)(prox_tick - now) : 0;
        if (kb_n > 0 && espera > (int)T_ESC_MS) espera = (int)T_ESC_MS;

        struct pollfd pf = { stdin_cerrado ? -1 : STDIN_FILENO, POLLIN, 0 };
        if (poll(&pf, 1, espera) > 0 && (pf.revents & (POLLIN | POLLHUP))) leer_teclado();

        now = ahora_ms();
        int k;
        while ((k = tecla_siguiente(now)) != K_NADA) {
            manejar_tecla(k);
            redibujar = 1;
        }
        if (now >= prox_tick) {
            enlace_tick(now);
            prox_tick += PERIODO_TICK_MS;
            if (prox_tick <= now) prox_tick = now + PERIODO_TICK_MS;
        }
        if (redibujar || now >= prox_dibujo) {
            dibujar(now);
            redibujar = 0;
            prox_dibujo = now + 100u;
        }
    }

    enviar_stop_al_salir();
    terminal_restaurar();
    printf("Programa terminado: se envio STOP al STM32.\n");
    return 0;
}

/* ############################################################################
 * ############################################################################
 * ##                                                                        ##
 * ##   8. SECCION DE PRUEBAS                                                ##
 * ##      - STM32 simulado (--sim): practicar la interfaz sin hardware.     ##
 * ##      - Modo pruebas (--prueba): verifica con prints en consola que la  ##
 * ##        comunicacion funciona. NUNCA envia START.                       ##
 * ##                                                                        ##
 * ############################################################################
 * ##########################################################################*/

/* ---- 8.1 STM32 simulado: misma logica de comandos que el firmware -------- */
static struct {
    uint8_t  est, banderas, cfg_color, cfg_bif, color, ult_seq, ult_res, pub;
    uint16_t cuenta[3];
    uint64_t t_cmd, t_accion, t_color;
    uint8_t  sig_color;
} S;

static void sim_iniciar(void)
{
    memset(&S, 0, sizeof S);
    S.t_cmd = ahora_ms();
    S.sig_color = COLOR_ROJO;
}

static void sim_avanzar(uint64_t now)
{
    if (estado_en_movimiento(S.est) && now - S.t_cmd > T_WATCHDOG_STM_MS) {
        S.est = EST_FALLA_COM;                       /* watchdog de enlace */
        S.banderas |= BAN_COM_PERDIDA;
    }
    if (S.est == EST_ACT_FRENO && now - S.t_accion >= 10000u)  S.est = EST_RUN;
    if (S.est == EST_ACT_REVERSA && now - S.t_accion >= 3000u) S.est = EST_RUN;

    if (S.est == EST_RUN && now - S.t_color >= 2500u) {       /* "detecta" un color */
        uint8_t c = S.sig_color;
        S.sig_color = (uint8_t)(c == COLOR_AZUL ? COLOR_ROJO : c + 1u);
        if (S.cuenta[c - 1] < 0xFFFFu) S.cuenta[c - 1]++;
        S.color = c;
        S.t_color = now;
        uint8_t a = color_accion(S.cfg_color, c);
        if (a == ACC_FRENAR)     { S.est = EST_ACT_FRENO;   S.t_accion = now; }
        if (a == ACC_RETROCEDER) { S.est = EST_ACT_REVERSA; S.t_accion = now; }
    }
    if (S.color != COLOR_NINGUNO && now - S.t_color > 600u) S.color = COLOR_NINGUNO;
}

static int sim_escribir(const uint8_t *b, size_t n)
{
    uint64_t now = ahora_ms();
    sim_avanzar(now);
    uint8_t seq, cmd;
    if (!cmd_leer(b, (uint32_t)n, &seq, &cmd)) return 0;   /* ACK, pero se descarta */
    S.t_cmd = now;
    if (cmd == CMD_HEARTBEAT) return 0;

    uint8_t r = RES_OK;
    if (cmd == CMD_STOP) {
        S.est = EST_IDLE;
    } else if (cmd == CMD_START) {
        if (estado_en_movimiento(S.est)) r = RES_OK;
        else if (S.est != EST_IDLE)      r = RES_ERR_ESTADO;
        else if (!S.cfg_color || !S.cfg_bif) r = RES_ERR_SIN_CONFIG;
        else {
            memset(S.cuenta, 0, sizeof S.cuenta);
            S.banderas &= (uint8_t)~BAN_COM_PERDIDA;
            S.est = EST_RUN;
            S.t_color = now;
        }
    } else if (cmd_es_color(cmd)) {
        if (S.est != EST_IDLE) r = RES_ERR_ESTADO; else S.cfg_color = cmd;
    } else if (cmd_es_bif(cmd)) {
        if (S.est != EST_IDLE) r = RES_ERR_ESTADO; else S.cfg_bif = cmd;
    } else {
        r = RES_ERR_DESCONOCIDO;
    }
    S.ult_seq = seq;
    S.ult_res = r;
    return 0;
}

static int sim_leer(uint8_t *b, size_t n)
{
    if (n != ST_LEN) { errno = EIO; return -1; }
    sim_avanzar(ahora_ms());
    estado_t e = { PROTO_VERSION, ++S.pub, S.ult_seq, S.ult_res, S.est, S.banderas,
                   S.cfg_color, S.cfg_bif, S.color, S.cuenta[0], S.cuenta[1], S.cuenta[2] };
    estado_codificar(&e, b);
    return 0;
}

/* ---- 8.2 Modo pruebas con prints ---------------------------------------- */
static int pr_total = 0, pr_ok = 0;

static void resultado_prueba(const char *id, int ok, const char *detalle)
{
    pr_total++;
    if (ok) pr_ok++;
    printf("  [%s] %-6s %s\n", id, ok ? "OK" : "FALLA", detalle);
}

static void imprimir_hex(const char *titulo, const uint8_t *b, size_t n)
{
    printf("        %s", titulo);
    for (size_t i = 0; i < n; i++) printf(" %02X", b[i]);
    printf("\n");
}

static void imprimir_estado(const estado_t *e)
{
    printf("        version=0x%02X pub=%u ult_seq=%u resultado=%s estado=%s banderas=0x%02X\n",
           e->version, e->pub_cnt, e->ult_seq, resultado_nombre(e->ult_resultado),
           estado_nombre(e->estado), e->banderas);
    printf("        colores=0x%02X bif=0x%02X color=%s cuenta R/G/B = %u/%u/%u\n", e->cfg_color,
           e->cfg_bif, color_nombre(e->color), e->cuenta_r, e->cuenta_g, e->cuenta_b);
}

/* Lee el estado; devuelve 1 si es valido */
static int prueba_leer(estado_t *e, int mostrar)
{
    uint8_t b[ST_LEN];
    if (bus_leer(b, ST_LEN) != 0) {
        if (mostrar) printf("        lectura fallida: %s\n", explicar_errno(errno));
        return 0;
    }
    if (mostrar) imprimir_hex("bytes:", b, ST_LEN);
    int r = estado_decodificar(b, e);
    if (r != DEC_OK) {
        if (mostrar) printf("        trama descartada: %s\n", r == DEC_ERR_CRC ? "CRC incorrecto" : "version distinta");
        return 0;
    }
    if (mostrar) imprimir_estado(e);
    return 1;
}

/* Envia un comando y espera LAST_SEQ == SEQ (hasta 200 ms). Devuelve resultado o -1 */
static int prueba_comando(uint8_t cmd, uint8_t seq)
{
    uint8_t t[CMD_LEN];
    cmd_armar(t, seq, cmd);
    imprimir_hex("trama enviada:", t, CMD_LEN);
    uint64_t t0 = ahora_ms();
    if (bus_escribir(t, CMD_LEN) != 0) {
        printf("        escritura fallida: %s\n", explicar_errno(errno));
        return -1;
    }
    for (int i = 0; i < 20; i++) {
        dormir_ms(10);
        estado_t e;
        if (prueba_leer(&e, 0) && e.ult_seq == seq) {
            printf("        confirmado en %u ms: LAST_SEQ=%u resultado=%s\n",
                   (unsigned)(ahora_ms() - t0), e.ult_seq, resultado_nombre(e.ult_resultado));
            return e.ult_resultado;
        }
    }
    printf("        sin confirmacion en 200 ms\n");
    return -1;
}

static int modo_pruebas(const char *dev, int n_lecturas)
{
    char d[160];
    printf("\n######################## SECCION DE PRUEBAS ########################\n");
    printf("  Bus %s, STM32 en 0x%02X%s. Esta prueba NUNCA envia START.\n\n", dev, I2C_DIR_STM32,
           modo_sim ? " (SIMULADO)" : "");

    /* P1: CRC */
    uint16_t c = crc16((const uint8_t *)"123456789", 9);
    snprintf(d, sizeof d, "CRC-16 de \"123456789\" = 0x%04X (esperado 0x29B1)", c);
    resultado_prueba("P1", c == 0x29B1u, d);

    /* P2: tabla de tramas */
    printf("  [P2] Tabla de tramas de comando (SEQ=0x01; HEARTBEAT usa SEQ=0x00):\n");
    static const uint8_t lista[] = { CMD_HEARTBEAT, CMD_START, CMD_STOP,
        CMD_COL_FRE_RET_VEL, CMD_COL_FRE_VEL_RET, CMD_COL_RET_FRE_VEL, CMD_COL_RET_VEL_FRE,
        CMD_COL_VEL_FRE_RET, CMD_COL_VEL_RET_FRE, CMD_BIF_SENSADO, CMD_BIF_IZQUIERDA, CMD_BIF_DERECHA };
    int tabla_ok = 1;
    for (size_t i = 0; i < sizeof lista; i++) {
        uint8_t t[CMD_LEN], s, k;
        cmd_armar(t, lista[i] == CMD_HEARTBEAT ? 0 : 1, lista[i]);
        if (!cmd_leer(t, CMD_LEN, &s, &k) || k != lista[i]) tabla_ok = 0;
        printf("        %-16s 0x%02X  ->  %02X %02X %02X %02X\n", cmd_nombre(lista[i]), lista[i],
               t[0], t[1], t[2], t[3]);
    }
    resultado_prueba("P2", tabla_ok, "las 12 tramas se arman y se validan");

    /* P3: trama de estado y deteccion de errores */
    estado_t e0 = { PROTO_VERSION, 7, 9, RES_OK, EST_RUN, BAN_LINEA_PERDIDA, CMD_COL_RET_VEL_FRE,
                    CMD_BIF_DERECHA, COLOR_VERDE, 300, 2, 65535 }, e1;
    uint8_t b[ST_LEN];
    estado_codificar(&e0, b);
    int ida_vuelta = estado_decodificar(b, &e1) == DEC_OK && e1.cuenta_r == 300 &&
                     e1.cuenta_b == 65535 && e1.cfg_color == CMD_COL_RET_VEL_FRE;
    int detectados = 0;
    for (unsigned bit = 0; bit < ST_LEN * 8u; bit++) {
        b[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        if (estado_decodificar(b, &e1) != DEC_OK) detectados++;
        b[bit / 8] ^= (uint8_t)(1u << (bit % 8));
    }
    snprintf(d, sizeof d, "codificar/decodificar estado; errores de 1 bit detectados %d/%u",
             detectados, ST_LEN * 8u);
    resultado_prueba("P3", ida_vuelta && detectados == (int)(ST_LEN * 8u), d);

    /* P4: abrir el bus */
    if (bus_abrir(dev) != 0) {
        snprintf(d, sizeof d, "no se pudo abrir %s: %s", dev, explicar_errno(errno));
        resultado_prueba("P4", 0, d);
        printf("\n  Resultado: %d de %d pruebas OK\n", pr_ok, pr_total);
        return 1;
    }
    resultado_prueba("P4", 1, "bus abierto y direccion 0x42 seleccionada");

    /* P5: lecturas de estado */
    printf("  [P5] %d lecturas de estado, una cada 100 ms:\n", n_lecturas);
    int buenas = 0;
    estado_t e;
    for (int i = 0; i < n_lecturas; i++) {
        printf("      lectura %d:\n", i + 1);
        if (prueba_leer(&e, 1)) buenas++;
        dormir_ms(100);
    }
    snprintf(d, sizeof d, "%d de %d lecturas validas", buenas, n_lecturas);
    resultado_prueba("P5", buenas * 10 >= n_lecturas * 8, d);
    if (!buenas) {
        printf("\n  Sin lecturas validas: se omiten P6-P8.\n  Resultado: %d de %d pruebas OK\n",
               pr_ok, pr_total);
        bus_cerrar();
        return 1;
    }

    /* P6: heartbeat */
    printf("  [P6] HEARTBEAT (sin confirmacion):\n");
    uint8_t t[CMD_LEN];
    cmd_armar(t, 0, CMD_HEARTBEAT);
    imprimir_hex("trama enviada:", t, CMD_LEN);
    int hb = bus_escribir(t, CMD_LEN) == 0;
    resultado_prueba("P6", hb, hb ? "el STM32 dio ACK" : explicar_errno(errno));

    /* P7: STOP con confirmacion por LAST_SEQ */
    printf("  [P7] STOP con confirmacion:\n");
    uint8_t seq = (uint8_t)(e.ult_seq >= 255u ? 1u : e.ult_seq + 1u);
    int r = prueba_comando(CMD_STOP, seq);
    resultado_prueba("P7", r == RES_OK, "STOP confirmado por LAST_SEQ");
    seq = (uint8_t)(seq >= 255u ? 1u : seq + 1u);

    /* P8: reenviar la config actual (o la por defecto) y comprobar el eco */
    printf("  [P8] Configuracion (no cambia la config actual si ya existe):\n");
    if (prueba_leer(&e, 0) && e.estado == EST_IDLE) {
        uint8_t col = e.cfg_color ? e.cfg_color : CMD_COL_FRE_RET_VEL;
        uint8_t bif = e.cfg_bif ? e.cfg_bif : CMD_BIF_SENSADO;
        int r1 = prueba_comando(col, seq);
        seq = (uint8_t)(seq >= 255u ? 1u : seq + 1u);
        int r2 = prueba_comando(bif, seq);
        int eco = prueba_leer(&e, 1) && e.cfg_color == col && e.cfg_bif == bif;
        snprintf(d, sizeof d, "%s y %s aplicadas y devueltas en la trama de estado",
                 cmd_nombre(col), cmd_nombre(bif));
        resultado_prueba("P8", r1 == RES_OK && r2 == RES_OK && eco, d);
    } else {
        resultado_prueba("P8", 0, "el carro no esta en IDLE; no se prueba la configuracion");
    }

    bus_cerrar();
    printf("\n  Resultado: %d de %d pruebas OK\n", pr_ok, pr_total);
    printf("####################################################################\n\n");
    return pr_ok == pr_total ? 0 : 1;
}

/* ############################ FIN SECCION DE PRUEBAS ###################### */

/* ============================================================================
 * 9. main
 * ==========================================================================*/
int main(int argc, char **argv)
{
    const char *dev = "/dev/i2c-1";
    int prueba = 0, n_lecturas = 5;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sim")) modo_sim = 1;
        else if (!strcmp(argv[i], "--prueba")) prueba = 1;
        else if (!strcmp(argv[i], "--bus") && i + 1 < argc) dev = argv[++i];
        else if (!strcmp(argv[i], "-n") && i + 1 < argc) n_lecturas = atoi(argv[++i]);
        else {
            fprintf(stderr, "Uso: %s [--sim] [--prueba] [--bus /dev/i2c-N] [-n lecturas]\n", argv[0]);
            return 2;
        }
    }
    if (n_lecturas < 1) n_lecturas = 1;

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = manejador_senal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    if (prueba) return modo_pruebas(dev, n_lecturas);

    if (bus_abrir(dev) != 0) {
        fprintf(stderr, "No se pudo abrir %s: %s\n", dev, explicar_errno(errno));
        return 1;
    }
    int r = bucle_interfaz();
    bus_cerrar();
    return r;
}
