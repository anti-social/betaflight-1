#include "gtest/gtest.h"

extern "C" {
    #include "common/filter.h"
    #include "drivers/pinio.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "io/beeper.h"
    #include "pg/pg_ids.h"
    #include "sensors/acceleration.h"
    #include "sensors/acceleration_init.h"
    #include "sensors/kaboom.h"

    uint8_t simulationKaboomPinioBoxId = KABOOM;
    bool simulationKaboomBoxState = false;
    bool simulationKaboomDisabledBoxState = false;
    bool simulationKaboomMoreSensitivityBoxState = false;

    // msp/msp_box.h
    bool getBoxIdState(boxId_e boxid);

    // io/piniobox.h
    uint8_t pinioBoxGetBoxId(int index);

    // drivers/pinio.h
    bool pinioState[PINIO_COUNT] = {false};
    bool pinioGet(int index);
    void pinioSet(int index, bool on);
}

class KaboomTest : public ::testing::Test
{
protected:
    static void SetUpTestCase() {}

    virtual void SetUp() {
        DISABLE_ARMING_FLAG(ARMED);

        simulationKaboomPinioBoxId = KABOOM;
        simulationKaboomBoxState = false;
        simulationKaboomDisabledBoxState = false;
        simulationKaboomMoreSensitivityBoxState = false;

        memset(kaboomConfigMutable(), 0, sizeof(kaboomConfig_t));
        kaboomConfigMutable()->sensitivity = 5;
        kaboomConfigMutable()->more_sensitivity = 2;
        kaboomConfigMutable()->activation_time_secs = 60;
        kaboomConfigMutable()->self_destruction_time_secs = 1200;

        memset(&acc, 0, sizeof(acc));
        acc.dev.acc_1G = 1;
        acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;
    }

    virtual void TearDown() {}

    timeUs_t toWaitingState() {
        timeUs_t simulationTimeUs = 1000;
        ENABLE_ARMING_FLAG(ARMED);
        kaboomCheck(simulationTimeUs);

        simulationTimeUs += 60000000;
        kaboomCheck(simulationTimeUs);
        EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());

        return simulationTimeUs;
    }
};

TEST_F(KaboomTest, TestIdleIfNoKaboomPin)
{
    // given
    simulationKaboomPinioBoxId = BOXID_NONE;

    kaboomInit();
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    kaboomCheck(0);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    kaboomCheck(1000);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    ENABLE_ARMING_FLAG(ARMED);
    kaboomCheck(2000);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    ENABLE_ARMING_FLAG(ARMED);
    kaboomCheck(60002000);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());
};

TEST_F(KaboomTest, TestIdleUntilArmed)
{
    // given
    simulationKaboomPinioBoxId = KABOOM;

    kaboomInit();
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    kaboomCheck(0);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    kaboomCheck(1000);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    ENABLE_ARMING_FLAG(ARMED);
    kaboomCheck(2000);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());

    // when
    DISABLE_ARMING_FLAG(ARMED);
    kaboomCheck(3000);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());

    // when
    ENABLE_ARMING_FLAG(ARMED);
    simulationKaboomDisabledBoxState = true;
    kaboomCheck(4000);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());

    // when
    simulationKaboomDisabledBoxState = false;
    kaboomCheck(60001999);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());

    // when
    kaboomCheck(60002000);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());

};

TEST_F(KaboomTest, TestManualActivation)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    timeUs_t simulationTimeUs = 1000;

    // when
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    for (int i = 0; i < 20; i++) {
        // when
        simulationKaboomBoxState = i % 2;
        simulationTimeUs += 1500000;
        kaboomCheck(simulationTimeUs);
        // then
        EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());
    }

    simulationTimeUs += 5000000;

    for (int i = 0; i < 5; i++) {
        // when
        simulationKaboomBoxState = i % 2;
        simulationTimeUs += 1000000;
        kaboomCheck(simulationTimeUs);
        // then
        EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());
    }
    // when
    simulationKaboomBoxState = true;
    simulationTimeUs += 1000000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());
}

TEST_F(KaboomTest, TestManualKaboom)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    timeUs_t simulationTimeUs = toWaitingState();

    simulationKaboomBoxState = true;
    kaboomCheck(simulationTimeUs);
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
}

TEST_F(KaboomTest, TestKaboomSensitivity)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    timeUs_t simulationTimeUs = toWaitingState();

    // when
    acc.accADC[0] = 2.0;
    acc.accADC[1] = 2.0;
    acc.accADC[2] = 2.0;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());

    // when
    acc.accADC[0] = 5.0;
    acc.accADC[1] = 0.0;
    acc.accADC[2] = 0.0;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
}

TEST_F(KaboomTest, TestKaboomMoreSensitivity)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    timeUs_t simulationTimeUs = toWaitingState();

    // when
    acc.accADC[0] = 2.0;
    acc.accADC[1] = 2.0;
    acc.accADC[2] = 2.0;
    simulationKaboomMoreSensitivityBoxState = true;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
}

TEST_F(KaboomTest, TestKaboomSelfDestruction)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    // when
    timeUs_t simulationTimeUs = toWaitingState();
    // then
    EXPECT_EQ(1200000000 - 60000000, kaboomTimeToSelfDestructionUs(simulationTimeUs));

    // when
    simulationTimeUs += 1200000000 - 60000000 - 5000000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(5000000, kaboomTimeToSelfDestructionUs(simulationTimeUs));

    // when
    simulationTimeUs += 4999999;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(1, kaboomTimeToSelfDestructionUs(simulationTimeUs));

    // when
    simulationTimeUs += 1;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
    EXPECT_EQ(0, kaboomTimeToSelfDestructionUs(simulationTimeUs));

    // when
    simulationTimeUs += 500000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
    EXPECT_EQ(0, kaboomTimeToSelfDestructionUs(simulationTimeUs));

    // when
    simulationTimeUs += 499000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
    EXPECT_EQ(0, kaboomTimeToSelfDestructionUs(simulationTimeUs));

    // when
    simulationTimeUs += 1000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(0, kaboomTimeToSelfDestructionUs(simulationTimeUs));
}

TEST_F(KaboomTest, TestKaboomDisabled)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());
    EXPECT_EQ(false, kaboomIsDisabled());

    timeUs_t simulationTimeUs = 0;

    // when
    ENABLE_ARMING_FLAG(ARMED);
    simulationTimeUs += 1000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());
    EXPECT_EQ(false, kaboomIsDisabled());

    // when
    simulationKaboomDisabledBoxState = true;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_ACTIVATING, kaboomGetState());
    EXPECT_EQ(true, kaboomIsDisabled());

    // when
    acc.accADC[0] = 5.0;
    acc.accADC[1] = 0.0;
    acc.accADC[2] = 0.0;
    simulationKaboomDisabledBoxState = true;
    simulationTimeUs += 60000000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(true, kaboomIsDisabled());

    // when
    acc.accADC[0] = 2.0;
    acc.accADC[1] = 2.0;
    acc.accADC[2] = 2.0;
    simulationKaboomMoreSensitivityBoxState = true;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(true, kaboomIsDisabled());

    // when
    simulationKaboomBoxState = true;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(true, kaboomIsDisabled());

    // when
    simulationKaboomDisabledBoxState = false;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
    EXPECT_EQ(false, kaboomIsDisabled());

    // when
    simulationTimeUs += 500000;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_KABOOM, kaboomGetState());
    EXPECT_EQ(false, kaboomIsDisabled());

    // when
    simulationKaboomDisabledBoxState = true;
    kaboomCheck(simulationTimeUs);
    // then
    EXPECT_EQ(KABOOM_STATE_WAITING, kaboomGetState());
    EXPECT_EQ(true, kaboomIsDisabled());
}

TEST_F(KaboomTest, TestKaboomGetMaxGForceSquared)
{
    // given
    kaboomInit();
    // then
    EXPECT_EQ(KABOOM_STATE_IDLE, kaboomGetState());

    for (int i = 0; i < 1000; i++) {
        // when
        acc.accADC[0] = (float) i;
        kaboomCheck(0);
        // then
        EXPECT_EQ((float) i * (float) i, kaboomGetMaxGForceSquared());
    }
}

// STUBS
extern "C" {
    // config/feature.h
    bool featureIsEnabled(uint8_t) { return true; }

    // sensors/acceleration.h
    bool accIsCalibrationComplete(void) { return true; }
    void performAcclerationCalibration(rollAndPitchTrims_t *) {}
    void performInflightAccelerationCalibration(rollAndPitchTrims_t *) {}

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);

    // sensors/acceleration_init.h
    // TODO: test acceleration filtering
    accelerationRuntime_t accelerationRuntime = {
        .accLpfCutHz = 25,
        // .accFilter = {}
        // pt2Filter_t accFilter[XYZ_AXIS_COUNT];
        // flightDynamicsTrims_t *accelerationTrims;
        // uint16_t calibratingA;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
    };

    // sensors/sensors.h
    uint8_t detectedSensors[] = { GYRO_NONE, ACC_NONE };

    // io/beeper.c
    void beeperConfirmationBeeps(uint8_t) {}

    // msp/msp_box.c
    bool getBoxIdState(boxId_e boxId) {
        switch (boxId) {
            case KABOOM:
                return simulationKaboomBoxState;
            case KABOOM_DISABLED:
                return simulationKaboomDisabledBoxState;
            case KABOOM_MORE_SENSITIVITY:
                return simulationKaboomMoreSensitivityBoxState;
            default:
                return false;
        }
    }

    // io/piniobox.c
    uint8_t pinioBoxGetBoxId(int ix) {
        if (ix == 0) {
            return simulationKaboomPinioBoxId;
        }
        return 255;
    }

    // drivers/pinio.c
    bool pinioGet(int ix) {
        return pinioState[ix];
    }
    void pinioSet(int ix, bool on) {
        pinioState[ix] = on;
    }
}
