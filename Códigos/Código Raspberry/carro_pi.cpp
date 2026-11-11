/* =============================================================================
 * carro_pi.cpp  -  Interfaz de consola + maestro I2C   (Raspberry Pi -> STM32)
 * -----------------------------------------------------------------------------
 * Archivos necesarios en la Pi:  carro_pi.cpp  y  protocolo.h  (misma carpeta)
 *
 * Preparar la Pi (una sola vez):
 *   sudo raspi-config  -> Interface Options -> I2C -> Enable
 *   En /boot/firmware/config.txt:  dtparam=i2c_arm_baudrate=100000
 *   sudo usermod -aG i2c $USER   (cerrar sesion y volver a entrar)
 *   Comprobar cableado:  i2cdetect -y 1   (debe aparecer 42)
 *
 * Compilar:
 *   g++ -std=c++17 -Wall -Wextra -O2 -o carro_pi carro_pi.cpp
 *
 * Ejecutar:
 *   ./carro_pi                    interfaz con el STM32 (/dev/i2c-1, 0x42)
 *   ./carro_pi --bus /dev/i2c-N   usar otro bus I2C
 *
 * Teclas: flechas = mover, Enter = elegir, Esc = volver, q = salir (envia STOP)
 *
 * Diseno: un solo hilo. Cada 20 ms se lee la trama de estado, se envian los
 * comandos pendientes y, si no se envio nada en 100 ms, un HEARTBEAT. Si la
 * interfaz se cuelga, los heartbeats se detienen y el STM32 para el carro en
 * 500 ms (no hace falta un watchdog aparte entre hilos).
 *
 * Clases:
 *   BusI2C      acceso a /dev/i2c-N (se cierra solo en el destructor)
 *   Mensajes    linea de avisos / errores que ve el usuario
 *   Enlace      protocolo: estado, envio con confirmacion, heartbeat
 *   Terminal    modo crudo de la consola (se restaura en el destructor)
 *   Teclado     decodifica flechas, Enter, Esc y q
 *   Interfaz    menus y dibujo de la pantalla
 *   Aplicacion  bucle principal
 * ===========================================================================*/
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

extern "C" {
#include "protocolo.h"
}

namespace {

/* ---- Parametros de la Pi ------------------------------------------------ */
constexpr uint64_t    PERIODO_TICK_MS   = 20;   /* lectura de estado + envios               */
constexpr uint64_t    T_CONFIRMACION_MS = 60;   /* espera de LAST_SEQ antes de reenviar      */
constexpr int         MAX_ENVIOS        = 4;    /* 1 envio + 3 reenvios                      */
constexpr uint64_t    T_ENLACE_CAIDO_MS = 500;  /* sin lecturas validas -> SIN CONEXION      */
constexpr uint64_t    T_ESC_MS          = 30;   /* espera para distinguir Esc de una flecha  */
constexpr int         PUB_IGUALES_MAX   = 5;    /* PUB_CNT congelado 5 lecturas -> degradado */
constexpr std::size_t COLA_MAX          = 8;    /* comandos en espera                        */
constexpr uint64_t    PERIODO_DIBUJO_MS = 100;  /* refresco de pantalla                      */

const char* const MSG_ERR_MOVIMIENTO =
    "ERROR: no se puede configurar con el carro en movimiento. Presione STOP primero.";

/* ---- Colores de la terminal (macros para poder concatenar literales) ---- */
#define FIN     "\033[K\r\n"      /* borra el resto de la linea y salta */
#define VERDE_T "\033[32m"
#define AMAR_T  "\033[33m"
#define ROJO_T  "\033[31m"
#define NEGR_T  "\033[1m"
#define NORM_T  "\033[0m"

void escribirStdout(const std::string& s)
{
    if (::write(STDOUT_FILENO, s.data(), s.size()) < 0) { }
}

/* ============================================================================
 * 1. TIEMPO (steady_clock = CLOCK_MONOTONIC: no salta si la Pi ajusta la hora)
 * ==========================================================================*/
namespace reloj {

uint64_t ahoraMs()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

void dormirMs(unsigned ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace reloj

/* ============================================================================
 * 2. BUS I2C (i2c-dev)
 *    escribir() = una transaccion START|0x42+W|datos|STOP
 *    leer()     = una transaccion START|0x42+R|datos|STOP   (sin repeated-start)
 * ==========================================================================*/
class BusI2C {
public:
    BusI2C() = default;
    ~BusI2C() { cerrar(); }
    BusI2C(const BusI2C&) = delete;
    BusI2C& operator=(const BusI2C&) = delete;

    bool abrir(const std::string& dev)
    {
        fd_ = ::open(dev.c_str(), O_RDWR);
        if (fd_ < 0) return false;
        if (::ioctl(fd_, I2C_SLAVE, I2C_DIR_STM32) < 0) {
            int e = errno;
            cerrar();
            errno = e;
            return false;
        }
        return true;
    }

    void cerrar()
    {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    /* true si se transfirieron exactamente n bytes; si no, errno indica la causa */
    bool escribir(const uint8_t* b, std::size_t n)
    {
        ssize_t r = ::write(fd_, b, n);
        if (r == static_cast<ssize_t>(n)) return true;
        if (r >= 0) errno = EIO;
        return false;
    }

    bool leer(uint8_t* b, std::size_t n)
    {
        ssize_t r = ::read(fd_, b, n);
        if (r == static_cast<ssize_t>(n)) return true;
        if (r >= 0) errno = EIO;
        return false;
    }

    static const char* explicarErrno(int e)
    {
        switch (e) {
        case ENOENT:    return "no existe el bus: habilita I2C con raspi-config";
        case EACCES:    return "sin permiso: agrega tu usuario al grupo i2c o usa sudo";
        case ENXIO:
        case EREMOTEIO: return "el STM32 no respondio (NACK): revisa 0x42, SDA/SCL/GND y que el firmware corra";
        case ETIMEDOUT: return "tiempo agotado: bus bloqueado o clock stretching muy largo";
        case EIO:       return "error de E/S en el bus";
        case EAGAIN:    return "bus ocupado, reintentar";
        default:        return std::strerror(e);
        }
    }

private:
    int fd_ = -1;
};

/* ============================================================================
 * 3. MENSAJES PARA LA INTERFAZ
 * ==========================================================================*/
class Mensajes {
public:
    void poner(bool esError, const char* fmt, ...) __attribute__((format(printf, 3, 4)))
    {
        char tmp[200];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(tmp, sizeof tmp, fmt, ap);
        va_end(ap);
        texto_ = tmp;
        error_ = esError;
        redibujar_ = true;
    }

    const std::string& texto() const { return texto_; }
    bool esError() const { return error_; }

    void pedirRedibujo() { redibujar_ = true; }
    bool tomarRedibujo()
    {
        bool r = redibujar_;
        redibujar_ = false;
        return r;
    }

private:
    std::string texto_;
    bool error_ = false;
    bool redibujar_ = true;
};

/* ============================================================================
 * 4. ENLACE: lectura de estado, envio con confirmacion, heartbeat
 * ==========================================================================*/
class Enlace {
public:
    Enlace(BusI2C& bus, Mensajes& msg) : bus_(bus), msg_(msg) {}

    void configPorDefecto(uint8_t color, uint8_t bif)
    {
        cfgColorDeseada_ = color;
        cfgBifDeseada_ = bif;
    }

    void tick(uint64_t now);
    int  nivel(uint64_t now) const;     /* 2 = OK, 1 = DEGRADADO, 0 = SIN CONEXION */

    /* Acciones del usuario */
    void pedirStart();
    void pedirStop();
    void pedirConfig(uint8_t cmd);
    void enviarStopAlSalir();

    /* Consultas para la pantalla */
    const estado_t& estado() const { return st_; }
    bool     sincronizado() const { return sincronizado_; }
    bool     versionMala() const { return versionMala_; }
    uint32_t lecturasOk() const { return lecturasOk_; }
    uint32_t errores() const { return errores_; }
    bool     startBloqueado() const { return startBloqueado_; }

private:
    uint8_t seqSiguiente();
    void encolar(uint8_t cmd);
    void quitarDeCola(uint8_t cmd);
    void registrarError(uint64_t now)
    {
        errores_++;
        tUltErr_ = now;
    }
    void enviarPendiente(uint64_t now);
    void iniciarEnvio(uint8_t cmd, uint64_t now);
    void comandoConfirmado(uint8_t res);
    void comandoSinConfirmar();
    void procesarEstado(const estado_t& e, uint64_t now);

    BusI2C&   bus_;
    Mensajes& msg_;

    estado_t st_{};                     /* ultima trama de estado valida              */
    bool     sincronizado_ = false;     /* ya se leyo al menos una trama valida       */
    bool     versionMala_ = false;
    uint64_t tUltOk_ = 0, tUltErr_ = 0, tUltEnvio_ = 0;
    uint32_t lecturasOk_ = 0, errores_ = 0;
    int      pubIguales_ = 0;
    uint8_t  sigSeq_ = 0;

    bool     pendActivo_ = false;       /* comando esperando confirmacion             */
    uint8_t  pendCmd_ = 0, pendSeq_ = 0;
    std::array<uint8_t, CMD_LEN> pendTrama_{};
    uint64_t pendT_ = 0;
    int      pendEnvios_ = 0;

    std::vector<uint8_t> cola_;
    bool     stopPedido_ = false;       /* STOP no pasa por la cola: tiene prioridad   */

    uint8_t  cfgColorDeseada_ = 0;      /* lo que eligio el usuario                    */
    uint8_t  cfgBifDeseada_ = 0;
    bool     configReenviar_ = false;
    bool     startBloqueado_ = false;   /* START elegido: no se puede repetir hasta STOP */
};

uint8_t Enlace::seqSiguiente()
{
    uint8_t s = sigSeq_;
    sigSeq_ = static_cast<uint8_t>(s >= 255u ? 1u : s + 1u);   /* 1..255, el 0 se salta */
    return s;
}

void Enlace::encolar(uint8_t cmd)
{
    if (std::find(cola_.begin(), cola_.end(), cmd) != cola_.end()) return;   /* ya estaba */
    if (cola_.size() < COLA_MAX) cola_.push_back(cmd);
}

void Enlace::quitarDeCola(uint8_t cmd)
{
    cola_.erase(std::remove(cola_.begin(), cola_.end(), cmd), cola_.end());
}

void Enlace::enviarPendiente(uint64_t now)
{
    pendEnvios_++;
    pendT_ = now;
    tUltEnvio_ = now;
    if (!bus_.escribir(pendTrama_.data(), pendTrama_.size())) registrarError(now);
}

void Enlace::iniciarEnvio(uint8_t cmd, uint64_t now)
{
    pendCmd_ = cmd;
    pendSeq_ = seqSiguiente();
    cmd_armar(pendTrama_.data(), pendSeq_, cmd);
    pendEnvios_ = 0;
    pendActivo_ = true;
    enviarPendiente(now);
}

void Enlace::comandoConfirmado(uint8_t res)
{
    const uint8_t c = pendCmd_;
    pendActivo_ = false;

    if (c == CMD_START) {
        if (res == RES_OK) {
            msg_.poner(false, "START confirmado: el carro esta en marcha.");
        } else {
            startBloqueado_ = false;
            if (res == RES_ERR_SIN_CONFIG)
                msg_.poner(true, "ERROR: START rechazado, falta configurar colores o bifurcacion.");
            else if (res == RES_ERR_ESTADO)
                msg_.poner(true, "ERROR: START rechazado: el carro esta en %s, presione STOP primero.",
                           estado_nombre(st_.estado));
            else
                msg_.poner(true, "ERROR: START rechazado (%s).", resultado_nombre(res));
        }
    } else if (c == CMD_STOP) {
        msg_.poner(false, "STOP confirmado: carro detenido.");
    } else if (cmd_es_color(c) || cmd_es_bif(c)) {
        if (res == RES_OK) {
            if (cmd_es_color(c)) cfgColorDeseada_ = c;
            else                 cfgBifDeseada_ = c;
            msg_.poner(false, "Configuracion aplicada: %s (0x%02X).", cmd_nombre(c), c);
        } else if (res == RES_ERR_ESTADO) {
            msg_.poner(true, "%s", MSG_ERR_MOVIMIENTO);
        } else {
            msg_.poner(true, "ERROR: configuracion rechazada (%s).", resultado_nombre(res));
        }
    }
}

void Enlace::comandoSinConfirmar()
{
    const uint8_t c = pendCmd_;
    pendActivo_ = false;
    msg_.poner(true, "ERROR: el STM32 no confirmo %s tras %d intentos.", cmd_nombre(c), MAX_ENVIOS);
    if (c == CMD_START) startBloqueado_ = false;
    if (c == CMD_STOP)  stopPedido_ = true;      /* STOP se sigue intentando siempre */
}

void Enlace::procesarEstado(const estado_t& e, uint64_t now)
{
    const bool primera = !sincronizado_;

    if (!primera) {
        pubIguales_ = (e.pub_cnt == st_.pub_cnt) ? pubIguales_ + 1 : 0;

        /* Reinicio del STM32: pierde la config o LAST_SEQ vuelve a 0 */
        if ((st_.cfg_color && !e.cfg_color) || (st_.ult_seq && !e.ult_seq)) {
            msg_.poner(true, "AVISO: se detecto un reinicio del STM32; se reenvia la configuracion.");
            configReenviar_ = true;
            startBloqueado_ = false;
        }
    }

    st_ = e;
    tUltOk_ = now;
    lecturasOk_++;
    versionMala_ = false;

    if (primera) {
        sincronizado_ = true;
        sigSeq_ = static_cast<uint8_t>(e.ult_seq >= 255u ? 1u : e.ult_seq + 1u);
        if (e.cfg_color) cfgColorDeseada_ = e.cfg_color;    /* adoptar la config vigente */
        if (e.cfg_bif)   cfgBifDeseada_ = e.cfg_bif;
        configReenviar_ = true;
        if (estado_en_movimiento(e.estado)) {
            stopPedido_ = true;
            msg_.poner(true, "AVISO: el carro ya se movia al conectar; se envia STOP por seguridad.");
        } else {
            msg_.poner(false, "Comunicacion establecida con el STM32.");
        }
    }

    /* Reenviar la configuracion si el STM32 no la tiene (solo en IDLE) */
    if (configReenviar_ && e.estado == EST_IDLE && !pendActivo_ && cola_.empty()) {
        if (e.cfg_color != cfgColorDeseada_) encolar(cfgColorDeseada_);
        if (e.cfg_bif != cfgBifDeseada_)     encolar(cfgBifDeseada_);
        configReenviar_ = false;
    }
}

void Enlace::tick(uint64_t now)
{
    std::array<uint8_t, ST_LEN> b{};
    bool leido = false;

    /* 1) Leer la trama de estado */
    if (bus_.leer(b.data(), b.size())) {
        estado_t e{};
        int r = estado_decodificar(b.data(), &e);
        if (r == DEC_OK) {
            procesarEstado(e, now);
            leido = true;
        } else {
            registrarError(now);
            if (r == DEC_ERR_VERSION) versionMala_ = true;
        }
    } else {
        registrarError(now);
    }

    /* 2) Envios: STOP primero, luego confirmacion del pendiente, luego la cola */
    if (stopPedido_) {
        stopPedido_ = false;
        pendActivo_ = false;                 /* cancela cualquier otro envio en curso */
        quitarDeCola(CMD_START);
        iniciarEnvio(CMD_STOP, now);
    } else if (pendActivo_) {
        if (leido && st_.ult_seq == pendSeq_) {
            comandoConfirmado(st_.ult_resultado);
        } else if (now - pendT_ >= T_CONFIRMACION_MS) {
            if (pendEnvios_ < MAX_ENVIOS) enviarPendiente(now);   /* misma SEQ */
            else                          comandoSinConfirmar();
        }
    } else if (!cola_.empty() && sincronizado_) {
        uint8_t c = cola_.front();
        cola_.erase(cola_.begin());
        iniciarEnvio(c, now);
    }

    /* 3) HEARTBEAT si no se envio nada en 100 ms (no espera confirmacion) */
    if (now - tUltEnvio_ >= T_HEARTBEAT_MS) {
        std::array<uint8_t, CMD_LEN> t{};
        cmd_armar(t.data(), 0, CMD_HEARTBEAT);
        if (!bus_.escribir(t.data(), t.size())) registrarError(now);
        tUltEnvio_ = now;
    }
}

int Enlace::nivel(uint64_t now) const
{
    if (!sincronizado_ || versionMala_ || now - tUltOk_ > T_ENLACE_CAIDO_MS) return 0;
    if (now - tUltOk_ > 3u * PERIODO_TICK_MS || now - tUltErr_ < 1000u ||
        pubIguales_ >= PUB_IGUALES_MAX)
        return 1;
    return 2;
}

void Enlace::pedirStart()
{
    const uint64_t now = reloj::ahoraMs();
    if (startBloqueado_) {
        msg_.poner(true, "ERROR: START ya fue elegido. Presione STOP antes de volver a iniciar.");
    } else if (versionMala_) {
        msg_.poner(true, "ERROR: version de firmware incompatible; START bloqueado.");
    } else if (nivel(now) == 0) {
        msg_.poner(true, "ERROR: sin comunicacion con el STM32; no se puede enviar START.");
    } else {
        startBloqueado_ = true;
        encolar(CMD_START);
        msg_.poner(false, "Enviando START...");
    }
}

void Enlace::pedirStop()
{
    stopPedido_ = true;
    startBloqueado_ = false;
    msg_.poner(false, "Enviando STOP...");
}

void Enlace::pedirConfig(uint8_t cmd)
{
    const uint64_t now = reloj::ahoraMs();
    if (nivel(now) == 0) {
        msg_.poner(true, "ERROR: sin comunicacion con el STM32; no se envio la configuracion.");
    } else if (estado_en_movimiento(st_.estado) || (startBloqueado_ && st_.estado == EST_IDLE)) {
        msg_.poner(true, "%s", MSG_ERR_MOVIMIENTO);      /* no se envia nada al STM32 */
    } else if (st_.estado != EST_IDLE) {
        msg_.poner(true, "ERROR: el carro esta en %s. Presione STOP para volver a IDLE.",
                   estado_nombre(st_.estado));
    } else {
        encolar(cmd);
        msg_.poner(false, "Enviando configuracion %s (0x%02X)...", cmd_nombre(cmd), cmd);
    }
}

void Enlace::enviarStopAlSalir()
{
    for (int i = 0; i < 3; i++) {                /* STOP es idempotente: 3 envios */
        std::array<uint8_t, CMD_LEN> t{};
        cmd_armar(t.data(), seqSiguiente(), CMD_STOP);
        bus_.escribir(t.data(), t.size());
        reloj::dormirMs(20);
    }
}

/* ============================================================================
 * 5. TERMINAL Y TECLADO
 * ==========================================================================*/
volatile std::sig_atomic_t g_senal = 0;

void manejadorSenal(int s) { g_senal = s; }     /* solo levanta la bandera */

class Terminal {
public:
    Terminal() = default;
    ~Terminal() { restaurar(); }
    Terminal(const Terminal&) = delete;
    Terminal& operator=(const Terminal&) = delete;

    void modoCrudo()
    {
        if (!::isatty(STDIN_FILENO)) return;
        if (::tcgetattr(STDIN_FILENO, &original_) != 0) return;
        termios t = original_;
        t.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));   /* sin buffer de linea ni eco */
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        ::tcsetattr(STDIN_FILENO, TCSANOW, &t);
        modificada_ = true;
    }

    void restaurar()
    {
        if (modificada_) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            modificada_ = false;
            escribirStdout("\033[?25h\n");               /* volver a mostrar el cursor */
        }
    }

private:
    termios original_{};
    bool    modificada_ = false;
};

enum class Tecla { Nada, Arriba, Abajo, Enter, Esc, Salir };

class Teclado {
public:
    void leer()
    {
        uint8_t tmp[32];
        ssize_t r = ::read(STDIN_FILENO, tmp, sizeof tmp);
        if (r == 0) { cerrado_ = true; return; }
        if (r < 0) return;
        for (ssize_t i = 0; i < r && buf_.size() < MAX_BUF; i++) buf_.push_back(tmp[i]);
        tUlt_ = reloj::ahoraMs();
    }

    /* Saca una tecla del buffer; varias teclas en un mismo read() no se pierden */
    Tecla siguiente(uint64_t now)
    {
        if (buf_.empty()) return Tecla::Nada;
        const uint8_t c = buf_[0];
        const std::size_t n = buf_.size();

        if (c == 27) {
            if (n >= 3 && (buf_[1] == '[' || buf_[1] == 'O')) {
                uint8_t f = buf_[2];
                consumir(3);
                if (f == 'A') return Tecla::Arriba;
                if (f == 'B') return Tecla::Abajo;
                return Tecla::Nada;                       /* otra flecha o tecla especial */
            }
            if (n == 1 || (n == 2 && (buf_[1] == '[' || buf_[1] == 'O'))) {
                if (now - tUlt_ < T_ESC_MS) return Tecla::Nada;   /* puede ser una flecha partida */
                consumir(n);
                return Tecla::Esc;
            }
            consumir(1);
            return Tecla::Esc;
        }
        consumir(1);
        if (c == '\r' || c == '\n') return Tecla::Enter;
        if (c == 'q' || c == 'Q')   return Tecla::Salir;
        if (c == 'w' || c == 'W')   return Tecla::Arriba;     /* alternativa si la terminal falla */
        if (c == 's' || c == 'S')   return Tecla::Abajo;
        return Tecla::Nada;
    }

    bool hayPendientes() const { return !buf_.empty(); }
    bool cerrado() const { return cerrado_; }

private:
    void consumir(std::size_t n) { buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(n)); }

    static constexpr std::size_t MAX_BUF = 64;
    std::vector<uint8_t> buf_;
    uint64_t tUlt_ = 0;              /* cuando llego el ultimo byte */
    bool     cerrado_ = false;
};

/* ============================================================================
 * 6. MENUS Y PANTALLA
 * ==========================================================================*/
class Interfaz {
public:
    Interfaz(Enlace& enlace, Mensajes& msg) : enlace_(enlace), msg_(msg) {}

    void manejarTecla(Tecla k);
    void dibujar(uint64_t now);
    bool quiereSalir() const { return salir_; }

private:
    enum class Pantalla { Principal, Settings, Bifurcacion, ColRojo, ColVerde, ColConfirmar };

    uint8_t accRestanteVerde(int i) const;                 /* las 2 acciones que no tiene el rojo */
    uint8_t accAzul() const { return static_cast<uint8_t>(6u - accRojo_ - accVerde_); }  /* 1+2+3 = 6 */
    int     nOpciones() const;
    void    irA(Pantalla p, int s)
    {
        pantalla_ = p;
        sel_ = s;
    }

    void p(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    void opcion(int i, const char* texto, const char* extra = nullptr);

    Enlace&   enlace_;
    Mensajes& msg_;

    Pantalla pantalla_ = Pantalla::Principal;
    int      sel_ = 0;
    uint8_t  accRojo_ = ACC_FRENAR;
    uint8_t  accVerde_ = ACC_RETROCEDER;
    bool     salir_ = false;
    std::string buf_;
};

uint8_t Interfaz::accRestanteVerde(int i) const
{
    int k = 0;
    for (unsigned a = ACC_FRENAR; a <= ACC_VELOCIDAD; a++) {
        if (a != accRojo_) {
            if (k == i) return static_cast<uint8_t>(a);
            k++;
        }
    }
    return ACC_NINGUNA;
}

int Interfaz::nOpciones() const
{
    switch (pantalla_) {
    case Pantalla::Principal:    return 3;
    case Pantalla::Settings:     return 2;
    case Pantalla::Bifurcacion:  return 3;
    case Pantalla::ColRojo:      return 3;
    case Pantalla::ColVerde:     return 2;
    case Pantalla::ColConfirmar: return 2;
    }
    return 1;
}

void Interfaz::manejarTecla(Tecla k)
{
    const int n = nOpciones();
    switch (k) {
    case Tecla::Arriba: sel_ = (sel_ + n - 1) % n; return;
    case Tecla::Abajo:  sel_ = (sel_ + 1) % n;     return;
    case Tecla::Salir:  salir_ = true;             return;

    case Tecla::Esc:
        switch (pantalla_) {
        case Pantalla::Settings:     irA(Pantalla::Principal, 1); break;
        case Pantalla::Bifurcacion:  irA(Pantalla::Settings, 0); break;
        case Pantalla::ColRojo:      irA(Pantalla::Settings, 1); break;
        case Pantalla::ColVerde:     irA(Pantalla::ColRojo, accRojo_ - 1); break;
        case Pantalla::ColConfirmar: irA(Pantalla::ColVerde, 0); break;
        default: break;
        }
        return;

    case Tecla::Enter:
        switch (pantalla_) {
        case Pantalla::Principal:
            if (sel_ == 0)      enlace_.pedirStart();
            else if (sel_ == 1) irA(Pantalla::Settings, 0);
            else                enlace_.pedirStop();
            break;
        case Pantalla::Settings:
            if (sel_ == 0) irA(Pantalla::Bifurcacion, 0);
            else           irA(Pantalla::ColRojo, 0);
            break;
        case Pantalla::Bifurcacion:
            enlace_.pedirConfig(static_cast<uint8_t>(CMD_BIF_PRIMERO + sel_));
            break;
        case Pantalla::ColRojo:
            accRojo_ = static_cast<uint8_t>(ACC_FRENAR + sel_);
            irA(Pantalla::ColVerde, 0);
            break;
        case Pantalla::ColVerde:
            accVerde_ = accRestanteVerde(sel_);
            irA(Pantalla::ColConfirmar, 0);
            break;
        case Pantalla::ColConfirmar:
            if (sel_ == 0) {
                uint8_t c = color_cmd_desde_acciones(accRojo_, accVerde_, accAzul());
                if (c) enlace_.pedirConfig(c);
                irA(Pantalla::Settings, 1);
            } else {
                irA(Pantalla::Settings, 1);
                msg_.poner(false, "Configuracion de colores cancelada.");
            }
            break;
        }
        return;

    default:
        return;
    }
}

void Interfaz::p(const char* fmt, ...)
{
    char tmp[512];
    va_list ap;
    va_start(ap, fmt);
    int r = std::vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (r > 0) buf_.append(tmp, std::min(static_cast<std::size_t>(r), sizeof tmp - 1));
}

void Interfaz::opcion(int i, const char* texto, const char* extra)
{
    p("  %s %s%s" NORM_T "%s" FIN, sel_ == i ? ">" : " ", sel_ == i ? NEGR_T : "", texto,
      extra ? extra : "");
}

void Interfaz::dibujar(uint64_t now)
{
    buf_.clear();
    p("\033[H");
    p(NEGR_T "=== CARRO SEGUIDOR DE LINEA | Raspberry Pi -> STM32 (I2C 0x%02X) ===" NORM_T FIN,
      static_cast<unsigned>(I2C_DIR_STM32));

    const int nv = enlace_.nivel(now);
    const char* txt = nv == 2 ? VERDE_T "OK" : nv == 1 ? AMAR_T "DEGRADADO" : ROJO_T "SIN CONEXION";
    p("Comunicacion : %s" NORM_T "   (lecturas OK %u, errores %u)%s" FIN, txt,
      static_cast<unsigned>(enlace_.lecturasOk()), static_cast<unsigned>(enlace_.errores()),
      enlace_.versionMala() ? ROJO_T "  VERSION INCOMPATIBLE" NORM_T : "");

    const estado_t& e = enlace_.estado();
    if (!enlace_.sincronizado()) {
        p("Esperando la primera trama de estado del STM32..." FIN);
    } else {
        p("Vista STM32  : %s" FIN, (e.banderas & BAN_COM_PERDIDA)
                                       ? ROJO_T "detuvo el carro por perdida de enlace" NORM_T
                                       : "recibe heartbeats");
        p("Estado carro : " NEGR_T "%-18s" NORM_T " Color bajo sensor: %s%s%s" FIN,
          estado_nombre(e.estado), color_nombre(e.color),
          (e.banderas & BAN_LINEA_PERDIDA) ? "   [linea perdida]" : "",
          (e.banderas & BAN_SENSOR_COLOR_FALLA) ? ROJO_T "   [sensor de color en falla]" NORM_T : "");
        p("Conteo       : Rojo %5u    Verde %5u    Azul %5u" FIN,
          static_cast<unsigned>(e.cuenta_r), static_cast<unsigned>(e.cuenta_g),
          static_cast<unsigned>(e.cuenta_b));
        if (e.cfg_color)
            p("Colores      : [0x%02X] Rojo=%s, Verde=%s, Azul=%s" FIN, e.cfg_color,
              accion_nombre(color_accion(e.cfg_color, COLOR_ROJO)),
              accion_nombre(color_accion(e.cfg_color, COLOR_VERDE)),
              accion_nombre(color_accion(e.cfg_color, COLOR_AZUL)));
        else
            p("Colores      : (sin configurar)" FIN);
        if (e.cfg_bif) p("Bifurcacion  : [0x%02X] %s" FIN, e.cfg_bif, bif_nombre(e.cfg_bif));
        else           p("Bifurcacion  : (sin configurar)" FIN);
    }
    p("----------------------------------------------------------------------" FIN);

    const bool mov = estado_en_movimiento(e.estado);
    const char* avisoCfg = mov ? ROJO_T "  (carro en movimiento: no se puede aplicar)" NORM_T : "";

    switch (pantalla_) {
    case Pantalla::Principal:
        p(NEGR_T "MENU PRINCIPAL" NORM_T FIN);
        opcion(0, "START", enlace_.startBloqueado() ? AMAR_T "   (bloqueado hasta elegir STOP)" NORM_T : nullptr);
        opcion(1, "SETTINGS");
        opcion(2, "STOP");
        break;
    case Pantalla::Settings:
        p(NEGR_T "SETTINGS" NORM_T "%s" FIN, avisoCfg);
        opcion(0, "Configurar bifurcaciones");
        opcion(1, "Configurar colores");
        break;
    case Pantalla::Bifurcacion:
        p(NEGR_T "SETTINGS > Configurar bifurcaciones" NORM_T "%s" FIN, avisoCfg);
        for (int i = 0; i < 3; i++) {
            uint8_t c = static_cast<uint8_t>(CMD_BIF_PRIMERO + i);
            char t[96];
            std::snprintf(t, sizeof t, "%-46s [0x%02X]", bif_nombre(c), c);
            opcion(i, t, e.cfg_bif == c ? VERDE_T "  <- activa" NORM_T : nullptr);
        }
        break;
    case Pantalla::ColRojo:
        p(NEGR_T "SETTINGS > Configurar colores (paso 1/3): accion para el ROJO" NORM_T "%s" FIN, avisoCfg);
        for (int i = 0; i < 3; i++) opcion(i, accion_nombre(static_cast<uint8_t>(ACC_FRENAR + i)));
        break;
    case Pantalla::ColVerde:
        p(NEGR_T "SETTINGS > Configurar colores (paso 2/3): accion para el VERDE" NORM_T "%s" FIN, avisoCfg);
        p("    Rojo = %s" FIN, accion_nombre(accRojo_));
        for (int i = 0; i < 2; i++) opcion(i, accion_nombre(accRestanteVerde(i)));
        break;
    case Pantalla::ColConfirmar: {
        uint8_t c = color_cmd_desde_acciones(accRojo_, accVerde_, accAzul());
        p(NEGR_T "SETTINGS > Configurar colores (paso 3/3): confirmar" NORM_T "%s" FIN, avisoCfg);
        p("    Rojo = %s, Verde = %s, Azul = %s" FIN, accion_nombre(accRojo_),
          accion_nombre(accVerde_), accion_nombre(accAzul()));
        p("    Trama: %s (0x%02X)" FIN, cmd_nombre(c), c);
        opcion(0, "Enviar configuracion");
        opcion(1, "Cancelar");
        break;
    }
    }

    p("----------------------------------------------------------------------" FIN);
    p("%s%s" NORM_T FIN, msg_.esError() ? ROJO_T : VERDE_T, msg_.texto().c_str());
    p("Flechas: mover   Enter: elegir   Esc: volver   q: salir (envia STOP)" FIN);
    p("\033[J");
    escribirStdout(buf_);
}

/* ============================================================================
 * 7. APLICACION (bucle principal)
 * ==========================================================================*/
class Aplicacion {
public:
    bool abrirBus(const std::string& dev) { return bus_.abrir(dev); }

    int ejecutar()
    {
        terminal_.modoCrudo();
        escribirStdout("\033[2J\033[?25l");             /* limpiar pantalla, ocultar cursor */

        enlace_.configPorDefecto(CMD_COL_FRE_RET_VEL,   /* por defecto si el STM32 no tiene config */
                                 CMD_BIF_SENSADO);
        msgs_.poner(false, "Conectando con el STM32...");

        uint64_t proxTick = reloj::ahoraMs(), proxDibujo = 0;
        while (!g_senal && !interfaz_.quiereSalir()) {
            uint64_t now = reloj::ahoraMs();
            int espera = proxTick > now ? static_cast<int>(proxTick - now) : 0;
            if (teclado_.hayPendientes() && espera > static_cast<int>(T_ESC_MS))
                espera = static_cast<int>(T_ESC_MS);

            pollfd pf{ teclado_.cerrado() ? -1 : STDIN_FILENO, POLLIN, 0 };
            if (::poll(&pf, 1, espera) > 0 && (pf.revents & (POLLIN | POLLHUP))) teclado_.leer();

            now = reloj::ahoraMs();
            Tecla k;
            while ((k = teclado_.siguiente(now)) != Tecla::Nada) {
                interfaz_.manejarTecla(k);
                msgs_.pedirRedibujo();
            }
            if (now >= proxTick) {
                enlace_.tick(now);
                proxTick += PERIODO_TICK_MS;
                if (proxTick <= now) proxTick = now + PERIODO_TICK_MS;
            }
            if (msgs_.tomarRedibujo() || now >= proxDibujo) {
                interfaz_.dibujar(now);
                proxDibujo = now + PERIODO_DIBUJO_MS;
            }
        }

        enlace_.enviarStopAlSalir();
        terminal_.restaurar();
        std::cout << "Programa terminado: se envio STOP al STM32." << std::endl;
        return 0;
    }

private:
    /* El orden importa: el bus se destruye (cierra) al final */
    BusI2C   bus_;
    Mensajes msgs_;
    Enlace   enlace_{ bus_, msgs_ };
    Terminal terminal_;
    Teclado  teclado_;
    Interfaz interfaz_{ enlace_, msgs_ };
};

}  // namespace

/* ============================================================================
 * 8. main
 * ==========================================================================*/
int main(int argc, char** argv)
{
    std::string dev = "/dev/i2c-1";

    for (int i = 1; i < argc; i++) {
        const std::string arg = argv[i];
        if (arg == "--bus" && i + 1 < argc) {
            dev = argv[++i];
        } else {
            std::cerr << "Uso: " << argv[0] << " [--bus /dev/i2c-N]\n";
            return 2;
        }
    }

    struct sigaction sa{};
    sa.sa_handler = manejadorSenal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);

    Aplicacion app;
    if (!app.abrirBus(dev)) {
        const int e = errno;
        std::cerr << "No se pudo abrir " << dev << ": " << BusI2C::explicarErrno(e) << "\n";
        return 1;
    }
    return app.ejecutar();
}
