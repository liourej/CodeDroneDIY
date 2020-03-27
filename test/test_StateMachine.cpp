#include <Arduino.h>
#include <unity.h>

#include "../src/stabilization/StabilizationStub.h"

#include "../src/stateMachine/StateMachine.h"
#include "../src/stateMachine/states/StartingState.h"
#include "../src/stateMachine/states/DisarmedState.h"
#include "../src/stateMachine/states/SafetyState.h"
#include "../src/stateMachine/states/AccroState.h"
#include "../src/stateMachine/states/AngleState.h"

#include "../src/stabilization/hardware/InertialMeasurementUnit.h"

StateMachine stateMachine;
StabilizationStub stabilization;

void test_stateMachine_initState(void) {
    float loopTimeSec = 1.0;
    stateMachine.SetState(InitState::GetInstance());
    TEST_ASSERT_EQUAL(Mode::initialization, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(2000);
    TEST_ASSERT_EQUAL(Mode::starting, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(2000);
    TEST_ASSERT_EQUAL(Mode::angle, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(2000);
    TEST_ASSERT_EQUAL(Mode::accro, stateMachine.GetStateName());
}

void test_stateMachine_startingState(void) {
    float loopTimeSec = 1.0;
    stateMachine.SetState(StartingState::GetInstance());
    TEST_ASSERT_EQUAL(Mode::starting, stateMachine.GetStateName());
    stateMachine.Run(loopTimeSec);
    delay(2000);
    TEST_ASSERT_EQUAL(Mode::starting, stateMachine.GetStateName());
}

void test_InertialMeasurementUnit(void) {
    InertialMeasurementUnit inertialMeasurementUnit;
    inertialMeasurementUnit.Init();
    inertialMeasurementUnit.ComputeOffsets();
    delay(5000);
    TEST_ASSERT_TRUE(inertialMeasurementUnit.AreOffsetComputed());
    
   /*  static const int nbAxis = 3;
    float accRaw[nbAxis] = {0, 0, 0};
    float gyroRaw[nbAxis] = {0, 0, 0};
    inertialMeasurementUnit.GetCorrectedAccelGyro(accRaw, gyroRaw); */
}

void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    stateMachine.Init();
    UNITY_BEGIN(); // IMPORTANT LINE!
}

void loop() {
    RUN_TEST(test_stateMachine_initState);
    RUN_TEST(test_stateMachine_startingState);
    RUN_TEST(test_InertialMeasurementUnit);
    delay(6000);
    UNITY_END(); // stop unit testing
}