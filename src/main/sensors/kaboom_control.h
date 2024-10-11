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
    kaboomControl_t control;
    uint8_t auxChannelIndex;
    channelRange_t range;
} kaboomControlCondition_t;

PG_DECLARE_ARRAY(kaboomControlCondition_t, KABOOM_CONTROL_COUNT, kaboomControlConditions);

bool kaboomControlDisabled(void);
bool kaboomControlMoreSensitivity(void);
bool kaboomControlKaboom(void);
void kaboomControlUpdate(void);
