#include <math.h>

#include "fc/runtime_config.h"

#include "io/piniobox.h"
#include "msp/msp_box.h"
#include "pg/pg_ids.h"
#include "sensors/acceleration.h"
#include "sensors/sensors.h"

#include "kaboom.h"

#ifdef USE_ACC
#define US_IN_SEC 1000000

// Sensitivity is a square of a g-force
#define KABOOM_DEFAULT_SENSITIVITY (9 * 9)
#define KABOOM_DEFAULT_MORE_SENSITIVITY (4 * 4)
#define KABOOM_DEFAULT_ACTIVATION_TIME_SECS 60
#define KABOOM_DEFAULT_SELF_DESTRUCTION_TIME_SECS 1200
#define KABOOM_PULSE_TIME_US 1000000
#define KABOOM_SELF_DESTRUCTION_REPEAT_INTERVAL_US (KABOOM_PULSE_TIME_US * 5)

// If a user switches kaboom box 3 times in a 5 second interval
// we perceive this action as a start of an activation
#define KABOOM_MANUAL_ACTIVATION_REPEAT 3
#define KABOOM_MANUAL_ACTIVATION_TIME_US 5000000

static float sensitivity = (float) KABOOM_DEFAULT_SENSITIVITY;
static float moreSensitivity = (float) KABOOM_DEFAULT_MORE_SENSITIVITY;
static timeUs_t activationTimeUs = (KABOOM_DEFAULT_ACTIVATION_TIME_SECS * US_IN_SEC);
static timeUs_t selfDestructionTimeUs = (KABOOM_DEFAULT_SELF_DESTRUCTION_TIME_SECS * US_IN_SEC);
static int kaboomPinioIx = -1;

// G-Force measures to display maximum value for the last second period
// Do not waste memory storing all the measurements as a framerate is much lower than the task rate
#define KABOOM_G_FORCE_MEASURES_FRACTION 4
#define KABOOM_G_FORCE_MEASURES_COUNT (KABOOM_TASK_HZ / KABOOM_G_FORCE_MEASURES_FRACTION)
static float gForceSquaredMeasures[KABOOM_G_FORCE_MEASURES_COUNT] = {0.0};
static uint8_t gForceMeasuresCounter = 0;

// Runtime variables
static kaboomState_t kaboomState = KABOOM_STATE_IDLE;
static bool isDisabled = false;
static bool prevKaboomBoxState = false;
static timeUs_t kaboomStartTimeUs = 0;

// Variables to track a manual activation
static uint8_t numKaboomBoxActivated = 0;
static timeUs_t kaboomBoxActivatedTimeUs = 0;

static timeUs_t armTimeUs = 0;
// End of runtime variables

PG_REGISTER_WITH_RESET_TEMPLATE(kaboomConfig_t, kaboomConfig, PG_KABOOM_CONFIG, 1);

PG_RESET_TEMPLATE(kaboomConfig_t, kaboomConfig,
    .sensitivity = KABOOM_DEFAULT_SENSITIVITY,
    .more_sensitivity = KABOOM_DEFAULT_MORE_SENSITIVITY,
    .activation_time_secs = KABOOM_DEFAULT_ACTIVATION_TIME_SECS,
    .self_destruction_time_secs = KABOOM_DEFAULT_SELF_DESTRUCTION_TIME_SECS,
);

kaboomState_t kaboomGetState(void)
{
    return kaboomState;
}

bool kaboomIsDisabled(void)
{
    return isDisabled;
}

float currentSensitivity(void)
{
    if (getBoxIdState(KABOOM_MORE_SENSITIVITY)) {
        return moreSensitivity;
    }
    return sensitivity;
}

float kaboomCurrentGForce(void)
{
    return sqrtf(currentSensitivity()) * acc.dev.acc_1G_rec;
}

float kaboomGetMaxGForceSquared(void) {
    float maxGForceSquared = 0.0;
    for (int i = 0; i < KABOOM_G_FORCE_MEASURES_COUNT; i++) {
        if (gForceSquaredMeasures[i] > maxGForceSquared) {
            maxGForceSquared = gForceSquaredMeasures[i];
        }
    }
    return maxGForceSquared;
}

static int findKaboomPinioIndex(void)
{
    for (int i = 0; i < PINIO_COUNT; i++) {
        uint8_t boxId = pinioBoxGetBoxId(i);

        if (boxId == KABOOM) {
            return i;
        }
    }
    return -1;
}

void kaboomInit(void)
{
    float acc1GSquared = acc.dev.acc_1G * acc.dev.acc_1G;
    sensitivity = ((float) (kaboomConfig()->sensitivity * kaboomConfig()->sensitivity)) * acc1GSquared;
    moreSensitivity = ((float) (kaboomConfig()->more_sensitivity * kaboomConfig()->more_sensitivity)) * acc1GSquared;
    activationTimeUs = kaboomConfig()->activation_time_secs * US_IN_SEC;
    selfDestructionTimeUs = kaboomConfig()->self_destruction_time_secs * US_IN_SEC;

    kaboomPinioIx = findKaboomPinioIndex();

    kaboomState = KABOOM_STATE_IDLE;
    prevKaboomBoxState = false;
    kaboomStartTimeUs = 0;
    numKaboomBoxActivated = 0;
    kaboomBoxActivatedTimeUs = 0;
    armTimeUs = 0;
}

void checkKaboom(timeUs_t currentTimeUs)
{
    if (kaboomPinioIx < 0) {
        return;
    }

    bool kaboomBoxState = getBoxIdState(KABOOM);

    isDisabled = getBoxIdState(KABOOM_DISABLED);

    // We do not want to calculate real g-force because it requires a square root operation
    float gForceSquared = calcAccModulusSquared();
    uint32_t gForceMeasuresIx = gForceMeasuresCounter / KABOOM_G_FORCE_MEASURES_FRACTION;
    if (gForceMeasuresCounter % KABOOM_G_FORCE_MEASURES_FRACTION == 0 || gForceSquared > gForceSquaredMeasures[gForceMeasuresIx]) {
        gForceSquaredMeasures[gForceMeasuresIx] = gForceSquared;
    }
    if (gForceMeasuresCounter + 1 >= KABOOM_TASK_HZ) {
        gForceMeasuresCounter = 0;
    } else {
        gForceMeasuresCounter++;
    }

    if (kaboomBoxState && !prevKaboomBoxState) {
        if (numKaboomBoxActivated == 0) {
            kaboomBoxActivatedTimeUs = currentTimeUs;
        } else if (currentTimeUs - kaboomBoxActivatedTimeUs > KABOOM_MANUAL_ACTIVATION_TIME_US) {
            numKaboomBoxActivated = 0;
            kaboomBoxActivatedTimeUs = currentTimeUs;
        }
        numKaboomBoxActivated++;
    }

    bool isArmed = ARMING_FLAG(ARMED) || numKaboomBoxActivated >= KABOOM_MANUAL_ACTIVATION_REPEAT;

    if (isArmed && armTimeUs == 0) {
        kaboomState = KABOOM_STATE_ACTIVATING;
        armTimeUs = currentTimeUs;
    }

    if (armTimeUs == 0) {
        goto exit;
    }

    timeUs_t armDurationUs = currentTimeUs - armTimeUs;
    if (armDurationUs < activationTimeUs) {
        goto exit;
    }

    kaboomState = KABOOM_STATE_WAITING;

    if (!isDisabled) {
        if (armDurationUs >= selfDestructionTimeUs) {
            if ((armDurationUs - selfDestructionTimeUs) % KABOOM_SELF_DESTRUCTION_REPEAT_INTERVAL_US < KABOOM_PULSE_TIME_US) {
                kaboomState = KABOOM_STATE_KABOOM;
            }
        }

        if (kaboomStartTimeUs != 0 && currentTimeUs - kaboomStartTimeUs < KABOOM_PULSE_TIME_US) {
            kaboomState = KABOOM_STATE_KABOOM;
        } else {
            if (kaboomBoxState && !prevKaboomBoxState) {
                kaboomState = KABOOM_STATE_KABOOM;
            } else {
                if (gForceSquared >= currentSensitivity()) {
                    kaboomState = KABOOM_STATE_KABOOM;
                }
            }
        }
    }

    bool kaboomPinState =  pinioGet(kaboomPinioIx);
    if (kaboomState == KABOOM_STATE_KABOOM && !kaboomPinState) {
        pinioSet(kaboomPinioIx, true);
        kaboomStartTimeUs = currentTimeUs;
    } else if (kaboomState != KABOOM_STATE_KABOOM && kaboomPinState) {
        pinioSet(kaboomPinioIx, false);
        kaboomStartTimeUs = 0;
    }

    exit:
    prevKaboomBoxState = kaboomBoxState;
}
#endif
