#include "fc/runtime_config.h"

#include "io/piniobox.h"
#include "msp/msp_box.h"
#include "pg/pg_ids.h"
#include "sensors/acceleration.h"
#include "sensors/sensors.h"

#include "kaboom.h"

#ifdef USE_ACC
#define US_IN_SEC 1000000

#define KABOOM_DEFAULT_SENSITIVITY 4
#define KABOOM_DEFAULT_MORE_SENSITIVITY 2
#define KABOOM_DEFAULT_ACTIVATION_TIME_SECS 60
#define KABOOM_DEFAULT_SELF_DESTRUCTION_TIME_SECS 1200
#define KABOOM_PULSE_TIME_US 1000000

timeUs_t kaboomStartTimeUs = 0;
bool kaboomBoxState = false;

float kaboomSensitivity = (float) KABOOM_DEFAULT_SENSITIVITY;
float kaboomMoreSensitivity = (float) KABOOM_DEFAULT_MORE_SENSITIVITY;
timeUs_t activationTimeUs = (KABOOM_DEFAULT_ACTIVATION_TIME_SECS * US_IN_SEC);
timeUs_t selfDestructionTimeUs = (KABOOM_DEFAULT_SELF_DESTRUCTION_TIME_SECS * US_IN_SEC);

bool kaboomIsReady = false;
timeUs_t firstArmTimeUs = 0;

void kaboomInit(void)
{
    kaboomSensitivity = (float) (kaboomConfig()->sensitivity);
    kaboomMoreSensitivity = (float) (kaboomConfig()->more_sensitivity);
    activationTimeUs = kaboomConfig()->activation_time_secs * US_IN_SEC;
    selfDestructionTimeUs = kaboomConfig()->self_destruction_time_secs * US_IN_SEC;
}

PG_REGISTER_WITH_RESET_TEMPLATE(kaboomConfig_t, kaboomConfig, PG_KABOOM_CONFIG, 1);

PG_RESET_TEMPLATE(kaboomConfig_t, kaboomConfig,
    .sensitivity = KABOOM_DEFAULT_SENSITIVITY,
    .more_sensitivity = KABOOM_DEFAULT_MORE_SENSITIVITY,
    .activation_time_secs = KABOOM_DEFAULT_ACTIVATION_TIME_SECS,
    .self_destruction_time_secs = KABOOM_DEFAULT_SELF_DESTRUCTION_TIME_SECS,
);

void checkKaboom(timeUs_t currentTimeUs)
{
#ifdef KABOOM_TESTING
    uint8_t isArmed = true;
#else
    uint8_t isArmed = ARMING_FLAG(ARMED);
#endif

    if (isArmed && firstArmTimeUs == 0) {
        firstArmTimeUs = currentTimeUs;
    }

    if (firstArmTimeUs == 0) {
        return;
    }

    timeUs_t armDurationUs = currentTimeUs - firstArmTimeUs;
    if (armDurationUs < activationTimeUs) {
        return;
    }

    if (getBoxIdState(KABOOM_DISABLED)) {
        return;
    }

    bool pinState = false;
    if (armDurationUs >= selfDestructionTimeUs) {
        pinState = true;
    }

    float maxSensitivity = kaboomSensitivity;
    if (getBoxIdState(KABOOM_MORE_SENSITIVITY)) {
        maxSensitivity = kaboomMoreSensitivity;
    }

    for (int i = 0; i < PINIO_COUNT; i++) {
        uint8_t boxId = pinioBoxGetBoxId(i);

        if (boxId == KABOOM) {
            if (kaboomStartTimeUs != 0 && currentTimeUs - kaboomStartTimeUs < KABOOM_PULSE_TIME_US) {
                pinState = true;
            } else {
                bool currentBoxState = getBoxIdState(boxId);
                if (currentBoxState && !kaboomBoxState) {
                    pinState = true;
                } else {
                    float gForce = calcGForce();
                    if (gForce >= maxSensitivity) {
                        pinState = true;
                    }
                }

                kaboomBoxState = currentBoxState;
            }

            bool currentPinState =  pinioGet(i);
            if (pinState && !currentPinState) {
                pinioSet(i, pinState);
                kaboomStartTimeUs = currentTimeUs;
            } else if (!pinState && currentPinState) {
                pinioSet(i, pinState);
                kaboomStartTimeUs = 0;
            }

            // Do we need more kabooms?
            break;
        }
    }
}
#endif
