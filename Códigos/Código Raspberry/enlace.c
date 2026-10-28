#include "enlace.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

#include "tiempo.h"

int enlace_abrir(enlace_t *en, const char *dispositivo, uint8_t dir)
{
    memset(en, 0, sizeof *en);
    en->fd = -1;

    int fd = open(dispositivo, O_RDWR);
    if (fd < 0) {
        en->ultimo_errno = errno;
        return -1;
    }
    en->fd = fd;
    if (enlace_cambiar_dir(en, dir) != 0) {
        close(fd);
        en->fd = -1;
        return -1;
    }
    return 0;
}

int enlace_cambiar_dir(enlace_t *en, uint8_t dir)
{
    if (ioctl(en->fd, I2C_SLAVE, (unsigned long)dir) < 0) {
        en->ultimo_errno = errno;
        return -1;
    }
    en->dir = dir;
    return 0;
}

void enlace_cerrar(enlace_t *en)
{
    if (en->fd >= 0) {
        close(en->fd);
        en->fd = -1;
    }
}

bool enlace_escribir(enlace_t *en, const uint8_t *datos, size_t n)
{
    ssize_t r = write(en->fd, datos, n);
    if (r == (ssize_t)n) {
        en->tx_ok++;
        return true;
    }
    en->tx_err++;
    en->ultimo_errno = (r < 0) ? errno : EIO;   /* incompleta = EIO */
    return false;
}

bool enlace_leer(enlace_t *en, uint8_t *buf, size_t n)
{
    ssize_t r = read(en->fd, buf, n);
    if (r == (ssize_t)n) {
        en->rx_ok++;
        return true;
    }
    en->rx_err++;
    en->ultimo_errno = (r < 0) ? errno : EIO;
    return false;
}

enlace_res_t enlace_leer_estado(enlace_t *en, estado_t *e,
                                uint8_t crudo[PROTO_LARGO_ESTADO])
{
    uint8_t b[PROTO_LARGO_ESTADO];

    if (!enlace_leer(en, b, sizeof b)) {
        return ENLACE_ERR_IO;
    }
    if (crudo != NULL) {
        memcpy(crudo, b, sizeof b);   /* copia de bytes, no de structs */
    }
    dec_res_t d = estado_decodificar(b, e);
    if (d != DEC_OK) {
        en->ultima_dec = d;
        en->trama_err++;
        return ENLACE_ERR_TRAMA;
    }
    return ENLACE_OK;
}

static bool hay_que_abortar(atomic_bool *abortar)
{
    return abortar != NULL && atomic_load(abortar);
}

bool enlace_enviar(enlace_t *en, uint8_t seq, uint8_t cmd,
                   const uint8_t *datos, uint8_t len, uint8_t *resultado,
                   atomic_bool *abortar)
{
    uint8_t t[PROTO_TRAMA_MAX];
    size_t n = trama_armar(t, sizeof t, seq, cmd, datos, len);
    if (n == 0) {
        return false;
    }

    estado_t e;
    for (unsigned envio = 0; envio <= ENLACE_REENVIOS; envio++) {
        if (hay_que_abortar(abortar)) {
            return false;
        }
        if (!enlace_escribir(en, t, n)) {
            continue;                         /* REQ-64: intento fallido */
        }
        for (unsigned lect = 0; lect < ENLACE_LECTURAS; lect++) {
            if (hay_que_abortar(abortar)) {
                return false;
            }
            espera_ms(ENLACE_ESPERA_MS);
            if (enlace_leer_estado(en, &e, NULL) == ENLACE_OK &&
                e.last_seq == seq) {
                if (resultado != NULL) {
                    *resultado = e.last_result;
                }
                return true;
            }
        }
    }
    return false;                             /* enlace caído */
}

const char *enlace_explicar(int err)
{
    switch (err) {
    case 0:
        return "sin error";
    case ENOENT:
        return "no existe el dispositivo: active I2C (dtparam=i2c_arm=on en "
               "/boot/firmware/config.txt) y reinicie";
    case EACCES:
        return "sin permiso: agregue su usuario al grupo i2c "
               "(sudo usermod -aG i2c $USER) y vuelva a iniciar sesión";
    case EBUSY:
        return "dirección ocupada por un driver del kernel";
    case ENXIO:
#ifdef EREMOTEIO
    case EREMOTEIO:
#endif
        return "nadie respondió (NACK): revise SDA/SCL, GND común, "
               "alimentación del esclavo y la dirección";
    case ETIMEDOUT:
        return "tiempo agotado: bus bloqueado o esclavo estirando el reloj "
               "demasiado";
    case EIO:
        return "error de E/S en el bus o transferencia incompleta";
    case EAGAIN:
        return "bus ocupado o arbitraje perdido; reintente";
    default:
        return strerror(err);
    }
}
