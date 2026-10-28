#include "tiempo.h"

#define NS_POR_S  1000000000LL
#define NS_POR_MS 1000000LL

uint64_t ahora_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

uint64_t ahora_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

void espera_ms(unsigned ms)
{
    struct timespec t = {
        .tv_sec  = (time_t)(ms / 1000u),
        .tv_nsec = (long)((long long)(ms % 1000u) * NS_POR_MS)
    };
    nanosleep(&t, NULL);
}

static int64_t a_ns(const struct timespec *t)
{
    return (int64_t)t->tv_sec * NS_POR_S + (int64_t)t->tv_nsec;
}

static void sumar_ms(struct timespec *t, unsigned ms)
{
    int64_t ns = a_ns(t) + (int64_t)ms * NS_POR_MS;
    t->tv_sec  = (time_t)(ns / NS_POR_S);
    t->tv_nsec = (long)(ns % NS_POR_S);
}

void periodo_iniciar(periodo_t *p, unsigned ms)
{
    clock_gettime(CLOCK_MONOTONIC, &p->siguiente);
    p->ms = ms;
}

void periodo_esperar(periodo_t *p)
{
    sumar_ms(&p->siguiente, p->ms);

    /* Si el programa se atrasó más de un periodo (p. ej. por un reintento
     * largo), se reancla al presente en lugar de ejecutar varias vueltas
     * seguidas para "ponerse al día". */
    struct timespec ya;
    clock_gettime(CLOCK_MONOTONIC, &ya);
    if (a_ns(&p->siguiente) < a_ns(&ya) - (int64_t)p->ms * NS_POR_MS) {
        p->siguiente = ya;
        sumar_ms(&p->siguiente, p->ms);
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &p->siguiente, NULL);
}
