#include "kaboom_control.h"
#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/kaboom_control.h"

// PG_REGISTER_WITH_RESET_TEMPLATE(kaboomControlConfig_t, kaboomControlConfig, PG_KABOOM_CONTROL_CONFIG, 0);
PG_REGISTER_ARRAY_WITH_RESET_FN(kaboomControlCondition_t, KABOOM_CONTROL_COUNT, kaboomControlConditions, PG_KABOOM_CONTROL_CONFIG, 0);
void pgResetFn_kaboomControlConditions(kaboomControlCondition_t *cond)
{
    for (int i = 0; i < KABOOM_CONTROL_COUNT; i++) {
        cond[i].auxChannelIndex = 0;
        cond[i].range.startStep = 0;
        cond[i].range.endStep = 0;
    }
}

static bool kaboomControl[KABOOM_CONTROL_COUNT];

bool kaboomControlDisabled(void) {
    return kaboomControl[KABOOM_CONTROL_DISABLED];
}

bool kaboomControlMoreSensitivity(void) {
    return kaboomControl[KABOOM_CONTROL_MORE_SENSITIVITY];
}

bool kaboomControlKaboom(void) {
    return kaboomControl[KABOOM_CONTROL_KABOOM];
}


void kaboomControlUpdate(void) {
    for (uint8_t i = 0; i < KABOOM_CONTROL_COUNT; i++) {
        const kaboomControlCondition_t *ctl = kaboomControlConditions(i);
        kaboomControl[i] = isRangeActive(ctl->auxChannelIndex, &ctl->range);
    }
}
