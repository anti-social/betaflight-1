#pragma once

#include "pg/pg.h"
#include "fc/rc_modes.h"

typedef enum kaboomControl_e {
    KABOOM_CONTROL_DISABLED,
    KABOOM_CONTROL_MORE_SENSITIVITY,
    KABOOM_CONTROL_KABOOM,
    KABOOM_CONTROL_COUNT,
} kaboomControl_t;

typedef struct kaboomControlCondition_s {
    uint8_t auxChannelIndex;
    uint8_t control;
    channelRange_t range;
} kaboomControlCondition_t;

typedef struct kaboomControlConfig_s {
    kaboomControlCondition_t controls[KABOOM_CONTROL_COUNT];
} kaboomControlConfig_t;

PG_DECLARE(kaboomControlConfig_t, kaboomControlConfig);

bool kaboomControlDisabled(void);
bool kaboomControlMoreSensitivity(void);
bool kaboomControlKaboom(void);
void kaboomControlUpdate(void);
