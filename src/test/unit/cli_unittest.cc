/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#include <math.h>

extern "C" {
    #include "platform.h"
    #include "target.h"
    #include "build/version.h"
    #include "io/gps.h"
    #include "cli/cli.h"
    #include "cli/settings.h"
    #include "common/printf.h"
    #include "common/maths.h"
    #include "common/gps_conversion.h"
    #include "config/feature.h"
    #include "drivers/buf_writer.h"
    #include "drivers/vtx_common.h"
    #include "config/config.h"
    #include "fc/rc_adjustments.h"
    #include "fc/runtime_config.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "io/beeper.h"
    #include "io/ledstrip.h"
    #include "io/serial.h"
    #include "io/vtx.h"
    #include "msp/msp.h"
    #include "msp/msp_box.h"
    #include "osd/osd.h"
    #include "pg/pg.h"
    #include "pg/gps_rescue.h"
    #include "pg/pg_ids.h"
    #include "pg/beeper.h"
    #include "pg/gps.h"
    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"
    #include "sensors/battery.h"
    #include "sensors/kaboom_control.h"
    #include "sensors/gyro.h"

    void cliSet(const char *cmdName, char *cmdline);
    int cliGetSettingIndex(char *name, uint8_t length);
    void *cliGetValuePointer(const clivalue_t *value);

    extern void pgResetFn_kaboomControlConditions(kaboomControlCondition_t *cond);
    void printKaboomControl(
        dumpFlags_t dumpMask,
        const kaboomControlCondition_t *cond,
        const kaboomControlCondition_t *condDefault,
        const char *headingStr
    );
    void cliKaboomControl(const char *, char *);

    const clivalue_t valueTable[] = {
        { .name = "array_unit_test",   .type = VAR_INT8  | MODE_ARRAY  | MASTER_VALUE, .config = { .array = { .length = 3}},                     .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
        { .name = "str_unit_test",     .type = VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config = { .string = { 0, 16, 0 }},                      .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
        { .name = "wos_unit_test",     .type = VAR_UINT8 | MODE_STRING | MASTER_VALUE, .config = { .string = { 0, 16, STRING_FLAGS_WRITEONCE }}, .pgn = PG_RESERVED_FOR_TESTING_1, .offset = 0 },
    };
    const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
    const lookupTableEntry_t lookupTables[] = {};
    const char * const lookupTableOsdDisplayPortDevice[] = {};
    const char * const buildKey = NULL;
    const char * const releaseName = NULL;

    bufWriter_t cliWriterDesc;
    extern bufWriter_t *cliWriter;
    extern bufWriter_t *cliErrorWriter;
    void dummyBufWriter(void *, void *, int);

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(ledStripConfig_t, ledStripConfig, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER(ledStripStatusModeConfig_t, ledStripStatusModeConfig, PG_LED_STRIP_STATUS_MODE_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);
    PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);
    PG_REGISTER_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);
    PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);
    PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);
    PG_REGISTER_ARRAY(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
    PG_REGISTER_ARRAY(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);

    PG_REGISTER_WITH_RESET_FN(int8_t, unitTestData, PG_RESERVED_FOR_TESTING_1, 0);
}

#include <vector>
#include "unittest_macros.h"
#include "gtest/gtest.h"

using namespace std;

const bool PRINT_TEST_DATA = false;


TEST(CLIUnittest, TestCliSetArray)
{
    char *str = (char *)"array_unit_test    =   123,  -3  , 1";
    cliSet("", str);

    const uint16_t index = cliGetSettingIndex(str, 15);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];
    int8_t *data = (int8_t *)cliGetValuePointer(&val);

    if (PRINT_TEST_DATA) {
        printf("\n===============================\n");
        for(int i = 0; i < val.config.array.length; i++){
            printf("data[%d] = %d\n", i, data[i]);
        }
        printf("\n===============================\n");
    }

    EXPECT_EQ(123, data[0]);
    EXPECT_EQ( -3, data[1]);
    EXPECT_EQ(  1, data[2]);
}

TEST(CLIUnittest, TestCliSetStringNoFlags)
{
    char *str = (char *)"str_unit_test    =   SAMPLE";
    cliSet("", str);

    const uint16_t index = cliGetSettingIndex(str, 13);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];
    uint8_t *data = (uint8_t *)cliGetValuePointer(&val);

    if (PRINT_TEST_DATA) {
        printf("\n===============================\n");
        for(int i = 0; i < val.config.string.maxlength && data[i] != 0; i++){
            printf("data[%d] = %d (%c)\n", i, data[i], data[i]);
        }
        printf("\n===============================\n");
    }

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);
}

TEST(CLIUnittest, TestCliSetStringWriteOnce)
{
    char *str1 = (char *)"wos_unit_test    =   SAMPLE";
    char *str2 = (char *)"wos_unit_test    =   ELPMAS";
    cliSet("", str1);

    const uint16_t index = cliGetSettingIndex(str1, 13);
    EXPECT_LT(index, valueTableEntryCount);

    const clivalue_t val = valueTable[index];

    uint8_t *data = (uint8_t *)cliGetValuePointer(&val);
    if (PRINT_TEST_DATA) {
        printf("\n===============================\n");
        for(int i = 0; i < val.config.string.maxlength && data[i] != 0; i++){
            printf("data[%d] = %d (%c)\n", i, data[i], data[i]);
        }
        printf("\n===============================\n");
    }
    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

    cliSet("", str2);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);

    cliSet("", str1);

    EXPECT_EQ('S', data[0]);
    EXPECT_EQ('A', data[1]);
    EXPECT_EQ('M', data[2]);
    EXPECT_EQ('P', data[3]);
    EXPECT_EQ('L', data[4]);
    EXPECT_EQ('E', data[5]);
    EXPECT_EQ(0,   data[6]);
}

// Kaboom tests
static uint8_t data[1000];
static vector<string> outLines;

class CliWriteTest : public ::testing::Test
{

protected:
    static void SetUpTestCase() {}

    virtual void SetUp() {
        pgResetFn_kaboomControlConditions(kaboomControlConditionsMutable(0));

        bufWriterInit(&cliWriterDesc, data, ARRAYLEN(data), &dummyBufWriter, NULL);
        cliWriter = cliErrorWriter = &cliWriterDesc;
    }

    virtual void TearDown() {
        cliWriter = NULL;
        outLines.clear();
    }

    static void dummyBufWriter(void *, void *_data, int size) {
        uint8_t *data = (uint8_t *)_data;
        ostringstream stream;
        for (int i = 0; i < size; i++) {
            stream << (char)data[i];
        }
        outLines.push_back(stream.str());
    }
};

TEST_F(CliWriteTest, TestPrintKaboomControl_NoHeading)
{
    printKaboomControl(DUMP_MASTER, kaboomControlConditions(0), NULL, NULL);
    vector<string> expected = {
        "kaboom_control 0 0 900 900", "\r\n",
        "kaboom_control 1 0 900 900", "\r\n",
        "kaboom_control 2 0 900 900", "\r\n"
    };
    EXPECT_EQ(expected, outLines);
}

TEST_F(CliWriteTest, TestPrintKaboomControl_WithHeading)
{
    const char heading[] = "kaboom_control";
    printKaboomControl(DUMP_MASTER, kaboomControlConditions(0), NULL, heading);
    vector<string> expected = {
        "\r\n# ", "kaboom_control", "\r\n",
        "kaboom_control 0 0 900 900", "\r\n",
        "kaboom_control 1 0 900 900", "\r\n",
        "kaboom_control 2 0 900 900", "\r\n"
    };
    EXPECT_EQ(expected, outLines);
}

TEST_F(CliWriteTest, TestPrintKaboomControl_DiffAll)
{
    kaboomControlCondition_t modifiedConditions[] = {
        { .auxChannelIndex = 7, .range = { .startStep = 0, .endStep = 12 } },
        { .auxChannelIndex = 3, .range = { .startStep = 0, .endStep = 12 } },
        { .auxChannelIndex = 3, .range = { .startStep = 36, .endStep = 48 } }
    };
    const char heading[] = "kaboom_control";
    printKaboomControl(DUMP_ALL, modifiedConditions, kaboomControlConditions(0), heading);
    vector<string> expected = {
        "\r\n# ", "kaboom_control", "\r\n",
        "kaboom_control 0 7 900 1200", "\r\n",
        "kaboom_control 1 3 900 1200", "\r\n",
        "kaboom_control 2 3 1800 2100", "\r\n"
    };
    EXPECT_EQ(expected, outLines);
}

TEST_F(CliWriteTest, TestCliKaboomControl_Show)
{
    const char cmd[] = "kaboom_control";
    char args[] = "";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "kaboom_control 0 0 900 900", "\r\n",
        "kaboom_control 1 0 900 900", "\r\n",
        "kaboom_control 2 0 900 900", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
    EXPECT_EQ(0, cond[2].auxChannelIndex);
}

TEST_F(CliWriteTest, TestCliKaboomControl_NotEnoughArgs)
{
    const char cmd[] = "kaboom_control";
    char args[] = "0 ";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "kaboom_control", ": ", "INVALID ARGUMENT COUNT###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
    EXPECT_EQ(0, cond[2].auxChannelIndex);
}

TEST_F(CliWriteTest, TestCliKaboomControl_InvalidChannelIndex)
{
    const char cmd[] = "kaboom_control";
    char args[] = "0 14 1800 2100";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "kaboom_control", ": ", "CHANNEL_INDEX NOT BETWEEN 0 AND 13###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
    EXPECT_EQ(0, cond[2].auxChannelIndex);
}

TEST_F(CliWriteTest, TestCliKaboomControl_InvalidKaboomControl)
{
    const char cmd[] = "kaboom_control";
    char args[] = "3 13 1800 2100";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "kaboom_control", ": ", "KABOOM_CONTROL NOT BETWEEN 0 AND 2###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
    EXPECT_EQ(0, cond[2].auxChannelIndex);
}

TEST_F(CliWriteTest, TestCliKaboomControl_OutOfRangeNumber)
{
    const char cmd[] = "kaboom_control";
    char args[] = "9223372036854775808 13 1800 2100";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "kaboom_control", ": ", "KABOOM_CONTROL NOT BETWEEN 0 AND 2###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
    EXPECT_EQ(0, cond[2].auxChannelIndex);
}

TEST_F(CliWriteTest, TestCliKaboomControl_NotANumber)
{
    const char cmd[] = "kaboom_control";
    char args[] = "a 13 1800 2100";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "###ERROR IN ", "kaboom_control", ": ", "KABOOM_CONTROL IS NOT A NUMBER###", "\r\n"
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
    EXPECT_EQ(0, cond[2].auxChannelIndex);
}

TEST_F(CliWriteTest, TestCliKaboomControl_SetCondition)
{
    const char cmd[] = "kaboom_control";
    char args[] = "2 1  900 1200";
    cliKaboomControl(cmd, args);
    vector<string> expected = {
        "kaboom_control 2 1 900 1200", "\r\n",
    };
    EXPECT_EQ(expected, outLines);

    auto cond = kaboomControlConditions(0);
    auto ctl2 = cond[2];
    EXPECT_EQ(1, ctl2.auxChannelIndex);
    EXPECT_EQ(0, ctl2.range.startStep);
    EXPECT_EQ(12, ctl2.range.endStep);
    EXPECT_EQ(0, cond[0].auxChannelIndex);
    EXPECT_EQ(0, cond[1].auxChannelIndex);
}
// End of kaboom tests

// STUBS
extern "C" {

int16_t debug[8];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

uint16_t batteryWarningVoltage;
uint8_t useHottAlarmSoundPeriod (void) { return 0; }
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000}; // see baudRate_e

uint8_t debugMode;
int32_t schedLoopStartCycles;
int32_t taskGuardCycles;

uint32_t micros(void) {return 0;}

int32_t getAmperage(void)
{
    return 100;
}

uint16_t getBatteryVoltage(void)
{
    return 42;
}

batteryState_e getBatteryState(void)
{
    return BATTERY_OK;
}

uint8_t calculateBatteryPercentageRemaining(void)
{
    return 67;
}

uint8_t getMotorCount()
{
    return 4;
}

size_t getEEPROMStorageSize()
{
    return 0;
}


void setPrintfSerialPort(struct serialPort_s) {}

static const box_t boxes[] = { { "DUMMYBOX", 0, 0 } };
const box_t *findBoxByPermanentId(uint8_t) { return &boxes[0]; }
const box_t *findBoxByBoxId(boxId_e) { return &boxes[0]; }

int8_t unitTestDataArray[3];

void pgResetFn_unitTestData(int8_t *)
{}

uint32_t getBeeperOffMask(void) { return 0; }
uint32_t getPreferredBeeperOffMask(void) { return 0; }

void beeper(beeperMode_e) {}
void beeperSilence(void) {}
void beeperConfirmationBeeps(uint8_t) {}
void beeperWarningBeeps(uint8_t) {}
void beeperUpdate(timeUs_t) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
beeperMode_e beeperModeForTableIndex(int) {return BEEPER_SILENCE;}
uint32_t beeperModeMaskForTableIndex(int idx) {UNUSED(idx); return 0;}
const char *beeperNameForTableIndex(int) {return NULL;}
int beeperTableEntryCount(void) {return 0;}
bool isBeeperOn(void) {return false;}
void beeperOffSetAll(uint8_t) {}
void setBeeperOffMask(uint32_t) {}
void setPreferredBeeperOffMask(uint32_t) {}

void beeperOffSet(uint32_t) {}
void beeperOffClear(uint32_t) {}
void beeperOffClearAll(void) {}
bool parseColor(int, const char *) {return false; }
bool resetEEPROM(void) { return true; }
// void bufWriterFlush(bufWriter_t *) {}
void mixerResetDisarmedMotors(void) {}

typedef enum {
    DUMMY
} pageId_e;

void dashboardShowFixedPage(pageId_e){}
void dashboardUpdate(timeUs_t) {}

bool parseLedStripConfig(int, const char *){return false; }
const char rcChannelLetters[] = "AERT12345678abcdefgh";

void parseRcChannels(const char *, rxConfig_t *){}
void mixerLoadMix(int, motorMixer_t *) {}
bool setModeColor(ledModeIndex_e, int, int) { return false; }
float motorConvertFromExternal(uint16_t) { return 1.0; }
void motorShutdown(void) { }
uint8_t getCurrentPidProfileIndex(void){ return 1; }
uint8_t getCurrentControlRateProfileIndex(void){ return 1; }
void changeControlRateProfile(uint8_t) {}
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *) {}
void writeEEPROM() {}
serialPortConfig_t *serialFindPortConfigurationMutable(serialPortIdentifier_e) {return NULL; }
baudRate_e lookupBaudRateIndex(uint32_t){return BAUD_9600; }
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e){ return NULL; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return NULL; }
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
uint32_t millis(void) { return 0; }
uint8_t getBatteryCellCount(void) { return 1; }
void servoMixerLoadMix(int) {}
const char * getBatteryStateString(void){ return "_getBatteryStateString_"; }

uint32_t stackTotalSize(void) { return 0x4000; }
uint32_t stackHighMem(void) { return 0x80000000; }
uint16_t getEEPROMConfigSize(void) { return 1024; }

uint8_t __config_start = 0x00;
uint8_t __config_end = 0x10;
uint16_t averageSystemLoadPercent = 0;

timeDelta_t getTaskDeltaTimeUs(taskId_e){ return 0; }
uint16_t currentRxIntervalUs = 9000;

/*const char *armingDisableFlagNames[]= {
"DUMMYDISABLEFLAGNAME"
};*/

void getTaskInfo(taskId_e, taskInfo_t *) {}
void getCheckFuncInfo(cfCheckFuncInfo_t *) {}
void schedulerResetTaskMaxExecutionTime(taskId_e) {}
void schedulerResetCheckFunctionMaxExecutionTime(void) {}

const char * const targetName = "UNITTEST";
const char* const buildDate = "Jan 01 2017";
const char * const buildTime = "00:00:00";
const char * const shortGitRevision = "MASTER";

//uint32_t serialRxBytesWaiting(const serialPort_t *) {return 0;}
//uint8_t serialRead(serialPort_t *){return 0;}

//void serialWriteBufShim(void *, const uint8_t *, int) {}
//void setArmingDisabled(armingDisableFlags_e) {}

void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void systemResetToBootloader(void) {}
void resetConfig(void) {}
void systemReset(void) {}
void writeUnmodifiedConfigToEEPROM(void) {}

void changePidProfile(uint8_t) {}
bool serialIsPortAvailable(serialPortIdentifier_e) { return false; }
void generateLedConfig(ledConfig_t *, char *, size_t) {}
//bool isSerialTransmitBufferEmpty(const serialPort_t *) {return true; }
//void serialWrite(serialPort_t *, uint8_t ch) { printf("%c", ch);}

//void serialSetCtrlLineStateCb(serialPort_t *, void (*)(void *, uint16_t ), void *) {}
void serialSetCtrlLineStateDtrPin(serialPort_t *, ioTag_t ) {}
void serialSetCtrlLineState(serialPort_t *, uint16_t ) {}

//void serialSetBaudRateCb(serialPort_t *, void (*)(serialPort_t *context, uint32_t baud), serialPort_t *) {}
void rescheduleTask(taskId_e, timeDelta_t){}
void schedulerSetNextStateTime(timeDelta_t ){}
char *getBoardName(void) { return NULL; }
char *getManufacturerId(void) { return NULL; }
bool boardInformationIsSet(void) { return true; }

bool setBoardName(char *newBoardName) { UNUSED(newBoardName); return true; };
bool setManufacturerId(char *newManufacturerId) { UNUSED(newManufacturerId); return true; };
bool persistBoardInformation(void) { return true; };

void activeAdjustmentRangeReset(void) {}
void analyzeModeActivationConditions(void) {}
bool isModeActivationConditionConfigured(const modeActivationCondition_t *, const modeActivationCondition_t *) { return false; }

void delay(uint32_t) {}
displayPort_t *osdGetDisplayPort(osdDisplayPortDevice_e *) { return NULL; }
mcuTypeId_e getMcuTypeId(void) { return MCU_TYPE_UNKNOWN; }
uint16_t getCurrentRxIntervalUs(void) { return 0; }
uint16_t getAverageSystemLoadPercent(void) { return 0; }
bool isRangeActive(uint8_t, const channelRange_t *) { return false; }
}
