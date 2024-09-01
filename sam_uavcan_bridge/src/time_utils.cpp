#include "time_utils.h"
#include <time.h>

static uint64_t first_us = 0;

uint64_t micros64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    if (first_us == 0) {
        first_us = tus;
    }
    return tus - first_us;
}

uint32_t millis32()
{
    return micros64() / 1000ULL;
}