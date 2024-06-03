#pragma once

#include "common/time.h"
#include "pg/pg.h"

void kaboomInit(void);
void checkKaboom(timeUs_t currentTimeUs);

#if defined(USE_ACC)
typedef struct kaboomConfig_s {
    uint8_t sensitivity;
    uint8_t more_sensitivity;
    uint16_t activation_time_secs;
    uint16_t self_destruction_time_secs;
} kaboomConfig_t;

PG_DECLARE(kaboomConfig_t, kaboomConfig);
#endif
