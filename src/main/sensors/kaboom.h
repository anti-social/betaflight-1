#pragma once

#include "common/time.h"
#include "pg/pg.h"

#define KABOOM_TASK_HZ 100

typedef enum kaboomState_e {
    KABOOM_STATE_IDLE,
    KABOOM_STATE_ACTIVATING,
    KABOOM_STATE_WAITING,
    KABOOM_STATE_KABOOM,
} kaboomState_t;

#if defined(USE_ACC)
typedef struct kaboomConfig_s {
    uint8_t sensitivity;
    uint8_t more_sensitivity;
    uint16_t activation_time_secs;
    uint32_t self_destruction_time_secs;
    uint16_t pulse_time_ms;
    ioTag_t kaboomTag;
    ioTag_t kaboomStatusTag;
} kaboomConfig_t;

PG_DECLARE(kaboomConfig_t, kaboomConfig);
#endif

kaboomState_t kaboomGetState(void);
bool kaboomIsDisabled(void);
float kaboomCurrentGForce(void);
float kaboomGetMaxGForceSquared(void);
timeUs_t kaboomTimeToSelfDestructionUs(timeUs_t currentTimeUs);
void kaboomInit(void);
void kaboomCheck(timeUs_t currentTimeUs);
