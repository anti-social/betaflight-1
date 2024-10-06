#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/kaboom_control.h"

PG_REGISTER_WITH_RESET_TEMPLATE(kaboomControlConfig_t, kaboomControlConfig, PG_KABOOM_CONTROL_CONFIG, 0);

PG_RESET_TEMPLATE(kaboomControlConfig_t, kaboomControlConfig,
    .controls = {
        { .auxChannelIndex = 0, .range = { .startStep = 0, .endStep = 0 } }
    },
);

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
        const kaboomControlCondition_t *ctl = &kaboomControlConfig()->controls[i];
        kaboomControl[i] = isRangeActive(ctl->auxChannelIndex, &ctl->range);
    }
}
