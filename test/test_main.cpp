#define UNIT_TEST 1
#ifdef UNIT_TEST
#include <unity.h>

#include "StabilizationStub.h"

#include "../src/stateMachine/StateMachine.h"
#include "../src/stateMachine/states/Disarmed.h"
#include "../src/stateMachine/states/Safety.h"
#include "../src/stateMachine/states/AccroMode.h"
#include "../src/stateMachine/states/AngleMode.h"

#include "../src/stabilization/hardware/InertialMeasurementUnit.h"

StateMachine stateMachine;
StabilizationStub stabilization;

void test_stateMachine_initState(void) {
    float loopTimeSec = 1.0;
    TEST_ASSERT_EQUAL(Mode::initialization, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(500);
    TEST_ASSERT_EQUAL(Mode::disarmed, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(500);
    TEST_ASSERT_EQUAL(Mode::angleMode, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(500);
    TEST_ASSERT_EQUAL(Mode::accroMode, stateMachine.GetStateName());
}

void test_stateMachine_startingState(void) {
    float loopTimeSec = 1.0;
    stateMachine.SetState(Disarmed::GetInstance());
    TEST_ASSERT_EQUAL(Mode::disarmed, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(500);
    TEST_ASSERT_EQUAL(Mode::disarmed, stateMachine.GetStateName());
}

void test_InertialMeasurementUnit(void) {
    InertialMeasurementUnit inertialMeasurementUnit;
    inertialMeasurementUnit.Init();
    inertialMeasurementUnit.ComputeOffsets();
    delay(4000);
    TEST_ASSERT_TRUE(inertialMeasurementUnit.AreOffsetComputed());

    static const int nbAxis = 3;
    float accRaw[nbAxis] = {0, 0, 0};
    float gyroRaw[nbAxis] = {0, 0, 0};
    inertialMeasurementUnit.GetCorrectedAccelGyro(accRaw, gyroRaw);
}

void test_ComputeDelta(void) {
    static const int size = 10;
    int16_t list[size] = {-1099, -789, -54, -43, -678, -908, -85, -1076, -85, -5};
    int16_t delta = 0;
    TEST_ASSERT_TRUE(CustomMath::ComputeDelta(list, size, &delta));
    TEST_ASSERT_EQUAL(1094, delta);
}

void test_ComputeMean() {
    float mean = 0.0;
    static const int size = 4;
    int16_t deltaThreshold = 0.1;
    int16_t samples1[size] = {0, 0, 0, 0};
    TEST_ASSERT_TRUE(CustomMath::ComputeMean(samples1, size, deltaThreshold, &mean));
    TEST_ASSERT_EQUAL(0, mean);

    int16_t samples2[size] = {90, 276, 0, 4};
    TEST_ASSERT_FALSE(CustomMath::ComputeMean(samples2, size, deltaThreshold, &mean));

    deltaThreshold = 277;
    TEST_ASSERT_TRUE(CustomMath::ComputeMean(samples2, size, deltaThreshold, &mean));

    int16_t samples3[size] = {11, 12, 12, 13};
    TEST_ASSERT_TRUE(CustomMath::ComputeMean(samples3, size, deltaThreshold, &mean));
    TEST_ASSERT_EQUAL(12, mean);
}

void test_VectorNormalize() {
    static const int vectorSize = 3;
    float vector[vectorSize] = {2.886, 0.897, 9.736};
    const float expectedVector[vectorSize] = {0.283, 0.088, 0.955};

    CustomMath::VectorNormalize(vector, vectorSize);
    for (int coord = 0; coord < vectorSize; coord++)
        TEST_ASSERT_FLOAT_WITHIN(0.001, expectedVector[coord], vector[coord]);

    float vector2[vectorSize] = {0.0, 0.0, 0.0};
    const float expectedVector2[vectorSize] = {0.0, 0.0, 0.0};

    for (int coord = 0; coord < vectorSize; coord++)
        TEST_ASSERT_FLOAT_WITHIN(0.001, expectedVector2[coord], vector2[coord]);
}

void test_RadioReception() {
    RadioReception radioReception;
    TEST_ASSERT_TRUE(radioReception.Init());
    delay(1000);
    TEST_ASSERT_UINT32_WITHIN(RadioReception::MAX_ANGLE * 2, 0, radioReception.GetPitchAngle());
    TEST_ASSERT_UINT32_WITHIN(RadioReception::MAX_ANGLE * 2, 0, radioReception.GetRollAngle());
    TEST_ASSERT_UINT32_WITHIN(RadioReception::MAX_YAW_SPEED * 2, 0, radioReception.GetYawSpeed());
    TEST_ASSERT_UINT32_WITHIN(RadioReception::MAX_ROT_SPEED * 2, 0,
                              radioReception.GetPitchSpeed());
    TEST_ASSERT_UINT32_WITHIN(RadioReception::MAX_ROT_SPEED * 2, 0,
                              radioReception.GetRollSpeed());
}

void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    stateMachine.Init();
    UNITY_BEGIN(); // IMPORTANT LINE!
}

void loop() {
    CustomSerialPrint::println("****************** STARTING TESTS ******************");
    RUN_TEST(test_stateMachine_initState);
    RUN_TEST(test_stateMachine_startingState);
    RUN_TEST(test_InertialMeasurementUnit);
    RUN_TEST(test_RadioReception);
    RUN_TEST(test_ComputeDelta);
    RUN_TEST(test_ComputeMean);
    RUN_TEST(test_VectorNormalize);
    CustomSerialPrint::println("****************** END OF TESTS ******************");
    UNITY_END(); // stop unit testing
}
#endif