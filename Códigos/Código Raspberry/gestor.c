#include "gestor.h"

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <string.h>

#include "tiempo.h"

#define PERIODO_MS            50u    /* REQ-82 */
#define HEARTBEAT_MS          100u   /* REQ-70 */
#define UI_VIVA_MS            300u   /* REQ-73 */
#define LECTURAS_PARA_CAIDA   3u     /* fallos seguidos para declarar caída */
#define PUB_IGUAL_PARA_CAIDA  3u     /* REQ-76 */
#define ARRANQUE_INTENTOS     10u    /* REQ-83 paso 1: 10 × 200 ms = 2 s */
#define ARRANQUE_ESPERA_MS    200u
#define COLA_TAM              8u

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t datos[PROTO_PAYLOAD_MAX];
} orden_t;

static enlace_t        *g_en;
static pthread_t        g_hilo;
static bool             g_hilo_activo;
static atomic_bool      g_salir;
static atomic_bool      g_pedir_stop;
static _Atomic uint64_t g_ui_viva_ms;

/* Todo lo de abajo está protegido por g_mx */
static pthread_mutex_t  g_mx = PTHREAD_MUTEX_INITIALIZER;
static orden_t          g_cola[COLA_TAM];
static unsigned         g_cola_ini;
static unsigned         g_cola_n;
static gestor_info_t    g_info;
static uint32_t         g_base_r, g_base_g, g_base_b;
static uint8_t          g_config[CONFIG_LARGO];
static bool             g_config_valida;

/* ---- Cola (llamar con g_mx tomado) ---------------------------------- */

static bool cola_meter_bloqueado(uint8_t cmd, const uint8_t *datos, uint8_t len)
{
    if (g_cola_n >= COLA_TAM || len > PROTO_PAYLOAD_MAX) {
        return false;
    }
    orden_t *o = &g_cola[(g_cola_ini + g_cola_n) % COLA_TAM];
    o->cmd = cmd;
    o->len = len;
    if (len > 0u) {
        memcpy(o->datos, datos, len);
    }
    g_cola_n++;
    return true;
}

static bool cola_sacar(orden_t *o)
{
    bool hay = false;
    pthread_mutex_lock(&g_mx);
    if (g_cola_n > 0u) {
        *o = g_cola[g_cola_ini];
        g_cola_ini = (g_cola_ini + 1u) % COLA_TAM;
        g_cola_n--;
        hay = true;
    }
    pthread_mutex_unlock(&g_mx);
    return hay;
}

/* ---- Envío de una orden con registro del resultado ------------------ */

static void enviar_orden(uint8_t *seq, uint8_t cmd, const uint8_t *datos, uint8_t len)
{
    uint8_t res = 0;
    bool ok;

    if (cmd == CMD_HEARTBEAT) {
        /* Excepción de REQ-63: el heartbeat no espera confirmación ni se
         * reintenta. Solo alimenta el watchdog del STM32 y el siguiente
         * sale en 100 ms; esperar su confirmación ocuparía el bus hasta
         * ~80 ms y retrasaría un STOP (hallazgo V-26). */
        uint8_t t[PROTO_TRAMA_MIN];
        size_t n = trama_armar(t, sizeof t, *seq, CMD_HEARTBEAT, NULL, 0);
        ok = (n > 0u) && enlace_escribir(g_en, t, n);
    } else if (cmd == CMD_STOP) {
        ok = enlace_enviar(g_en, *seq, cmd, datos, len, &res, NULL);   /* nunca se aborta */
    } else {
        ok = enlace_enviar(g_en, *seq, cmd, datos, len, &res, &g_pedir_stop);
    }
    bool cancelado = !ok && cmd != CMD_STOP && cmd != CMD_HEARTBEAT &&
                     atomic_load(&g_pedir_stop);
    *seq = seq_siguiente(*seq);

    pthread_mutex_lock(&g_mx);
    if (cmd == CMD_HEARTBEAT) {
        if (!ok) {
            g_info.heartbeats_fallidos++;
            g_info.enlace_ok = false;
        }
    } else {
        g_info.ult_id++;
        g_info.ult_cmd        = cmd;
        g_info.ult_confirmado = ok;
        g_info.ult_cancelado  = cancelado;
        g_info.ult_resultado  = res;
        if (!ok && !cancelado) {
            g_info.enlace_ok = false;
        }
        /* Se recuerda la última config aceptada para reenviarla tras un
         * reset del STM32 (REQ-75 b). */
        if (cmd == CMD_SET_CONFIG && ok &&
            (res == RES_OK || res == RES_DUPLICATE) && len == CONFIG_LARGO) {
            memcpy(g_config, datos, CONFIG_LARGO);
            g_config_valida = true;
        }
    }
    pthread_mutex_unlock(&g_mx);
}

/* ---- Lectura periódica del estado ----------------------------------- */

static void procesar_lectura(void)
{
    static estado_t previo;
    static bool     hay_previo;
    static unsigned pub_igual;
    static unsigned fallos;

    estado_t e;
    enlace_res_t r = enlace_leer_estado(g_en, &e, NULL);

    pthread_mutex_lock(&g_mx);
    if (r == ENLACE_OK) {
        if (hay_previo) {
            /* REQ-75: UPTIME retrocede o LAST_SEQ vuelve a 0 => el STM32
             * se reinició */
            if (e.uptime_ms < previo.uptime_ms ||
                (previo.last_seq != 0u && e.last_seq == 0u)) {
                g_base_r += previo.count_r;
                g_base_g += previo.count_g;
                g_base_b += previo.count_b;
                g_info.reset_detectado = true;
                if (g_config_valida) {
                    (void)cola_meter_bloqueado(CMD_SET_CONFIG, g_config, CONFIG_LARGO);
                }
            }
            /* REQ-76: PUB_CNT congelado */
            pub_igual = (e.pub_cnt == previo.pub_cnt) ? pub_igual + 1u : 0u;
        }
        previo     = e;
        hay_previo = true;
        fallos     = 0;

        g_info.hay_estado = true;
        g_info.version_incompatible = false;
        g_info.estado     = e;
        g_info.total_r    = g_base_r + e.count_r;
        g_info.total_g    = g_base_g + e.count_g;
        g_info.total_b    = g_base_b + e.count_b;
        g_info.enlace_ok  = (pub_igual < PUB_IGUAL_PARA_CAIDA);
    } else {
        if (r == ENLACE_ERR_TRAMA && g_en->ultima_dec == DEC_ERR_VER) {
            g_info.version_incompatible = true;              /* REQ-83 paso 2 */
        }
        fallos++;
        if (fallos >= LECTURAS_PARA_CAIDA) {
            g_info.enlace_ok = false;
        }
    }
    pthread_mutex_unlock(&g_mx);
}

/* ---- Hilo ----------------------------------------------------------- */

static void *hilo_gestor(void *arg)
{
    (void)arg;
    uint8_t  seq = 1;
    estado_t e;
    bool     sincronizado = false;

    /* REQ-83 pasos 1 a 4: esperar al STM32, sincronizar SEQ, STOP si se
     * movía. Paso 2: si el STM32 habla otra versión, se avisa a la
     * interfaz, que bloquea Inicio. */
    for (unsigned i = 0; i < ARRANQUE_INTENTOS && !atomic_load(&g_salir); i++) {
        enlace_res_t r = enlace_leer_estado(g_en, &e, NULL);
        if (r == ENLACE_OK) {
            seq = seq_siguiente(e.last_seq);          /* REQ-61 */
            sincronizado = true;
            break;
        }
        if (r == ENLACE_ERR_TRAMA && g_en->ultima_dec == DEC_ERR_VER) {
            pthread_mutex_lock(&g_mx);
            g_info.version_incompatible = true;       /* REQ-83 paso 2 */
            pthread_mutex_unlock(&g_mx);
        }
        espera_ms(ARRANQUE_ESPERA_MS);
    }
    if (sincronizado && e.estado != ESTADO_IDLE) {
        enviar_orden(&seq, CMD_STOP, NULL, 0);
    }
    /* PENDIENTE REQ-83 paso 5: cargar la config guardada en archivo,
     * enviarla con SET_CONFIG y comparar CONFIG_HASH (REQ-51). */

    periodo_t p;
    periodo_iniciar(&p, PERIODO_MS);
    uint64_t ultimo_cmd = ahora_ms();

    while (!atomic_load(&g_salir)) {
        periodo_esperar(&p);

        /* 1) STOP primero que todo (REQ-81) */
        if (atomic_exchange(&g_pedir_stop, false)) {
            enviar_orden(&seq, CMD_STOP, NULL, 0);
            ultimo_cmd = ahora_ms();
        }

        /* 2) Estado (REQ-82) */
        procesar_lectura();

        /* 3) Un comando de la cola por vuelta */
        orden_t o;
        if (cola_sacar(&o)) {
            enviar_orden(&seq, o.cmd, o.datos, o.len);
            ultimo_cmd = ahora_ms();
        }

        /* 4) Heartbeat solo si la interfaz está viva (REQ-70, REQ-73) */
        uint64_t ya = ahora_ms();
        if (ya - ultimo_cmd >= HEARTBEAT_MS &&
            ya - atomic_load(&g_ui_viva_ms) < UI_VIVA_MS) {
            enviar_orden(&seq, CMD_HEARTBEAT, NULL, 0);
            ultimo_cmd = ya;
        }
    }

    /* REQ-77: STOP al salir (enlace_enviar ya reintenta hasta 3 veces) */
    enviar_orden(&seq, CMD_STOP, NULL, 0);
    return NULL;
}

/* ---- API ------------------------------------------------------------ */

bool gestor_iniciar(enlace_t *en)
{
    g_en = en;
    atomic_store(&g_salir, false);
    atomic_store(&g_pedir_stop, false);
    atomic_store(&g_ui_viva_ms, ahora_ms());

    /* Las señales las atiende solo el hilo principal: se bloquean
     * mientras se crea el hilo para que él las herede bloqueadas. */
    sigset_t bloquear, anterior;
    sigemptyset(&bloquear);
    sigaddset(&bloquear, SIGINT);
    sigaddset(&bloquear, SIGTERM);
    sigaddset(&bloquear, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &bloquear, &anterior);
    int r = pthread_create(&g_hilo, NULL, hilo_gestor, NULL);
    pthread_sigmask(SIG_SETMASK, &anterior, NULL);

    g_hilo_activo = (r == 0);
    return g_hilo_activo;
}

void gestor_detener(void)
{
    if (!g_hilo_activo) {
        return;
    }
    atomic_store(&g_salir, true);
    pthread_join(g_hilo, NULL);
    g_hilo_activo = false;
}

bool gestor_encolar(uint8_t cmd, const uint8_t *datos, uint8_t len)
{
    if (cmd == CMD_STOP) {
        gestor_pedir_stop();
        return true;
    }
    pthread_mutex_lock(&g_mx);
    bool ok = cola_meter_bloqueado(cmd, datos, len);
    pthread_mutex_unlock(&g_mx);
    return ok;
}

void gestor_pedir_stop(void)
{
    atomic_store(&g_pedir_stop, true);
}

void gestor_senal_de_vida(void)
{
    atomic_store(&g_ui_viva_ms, ahora_ms());
}

void gestor_info(gestor_info_t *copia)
{
    pthread_mutex_lock(&g_mx);
    *copia = g_info;
    pthread_mutex_unlock(&g_mx);
}

void gestor_limpiar_reset(void)
{
    pthread_mutex_lock(&g_mx);
    g_info.reset_detectado = false;
    pthread_mutex_unlock(&g_mx);
}
