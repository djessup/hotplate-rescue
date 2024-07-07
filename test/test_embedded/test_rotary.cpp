//
// Created by DJ on 3/07/2024.
//
#include <Arduino.h>
#include <unity.h>
#include <rotary.h>
#include <rotary_states.h>

#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 4

volatile uint8_t* CLK_REGISTER;
volatile uint8_t* DT_REGISTER;


/* START TEST FUNCS */

// Tests that the Rotary constructor correctly initializes the pins and state
void ConstructorInitializesPinsAndState() {
    Rotary encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
    TEST_ASSERT_EQUAL(R_START, encoder.getState());
}

// Tests that the process method correctly identifies a clockwise rotation
void ProcessIdentifiesClockwiseRotation() {
    Rotary encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
    encoder.process(); // Move to R_CW_BEGIN
    *CLK_REGISTER |= 0b11; // Simulate both pins high
    uint8_t result = encoder.process(); // Should move to R_CW_NEXT and emit DIR_CW
    TEST_ASSERT_EQUAL(R_START | DIR_CW, result);
}

// Tests that the process method correctly identifies a counter-clockwise rotation
void ProcessIdentifiesCounterClockwiseRotation() {
    Rotary encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
    *CLK_REGISTER |= 0b00; // Simulate pin1 low
    *DT_REGISTER |= 0b01; // Simulate pin2 high
    encoder.process(); // Move to R_CCW_BEGIN
    *CLK_REGISTER |= 0b11; // Simulate both pins high
    uint8_t result = encoder.process(); // Should move to R_CCW_NEXT and emit DIR_CCW
    TEST_ASSERT_EQUAL(R_START | DIR_CCW, result);
}

// Tests that invalid state transitions reset the state machine
void InvalidStateTransitionsResetStateMachine() {
    Rotary encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
    *CLK_REGISTER |= 0b10; // Simulate an invalid state
    *DT_REGISTER |= 0b01; // Simulate an invalid state
    encoder.process(); // Should reset to R_START
    TEST_ASSERT_EQUAL(R_START, encoder.getState());
}

// Tests that the process method handles debounce by staying in intermediate states
void ProcessHandlesDebounceByStayingInIntermediateStates() {
    Rotary encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
    *CLK_REGISTER |= 0b01; // Simulate pin1 high
    *DT_REGISTER |= 0b00; // Simulate pin2 low
    encoder.process(); // Move to R_CW_BEGIN
    *CLK_REGISTER |= 0b01; // Keep pin1 high (simulate bounce)
    *DT_REGISTER |= 0b00; // Keep pin2 low (simulate bounce)
    encoder.process(); // Should stay in R_CW_BEGIN due to debounce
    TEST_ASSERT_EQUAL(R_CW_BEGIN, encoder.getState());
}
/* END TEST FUNCS */


void setUp(void) {
    // *CLK_REGISTER = 0;
    // *DT_REGISTER = 0;
    // Get the port for each pin
    uint8_t DT_PORT = digitalPinToPort(ENCODER_CLK_PIN);
    uint8_t CLK_PORT = digitalPinToPort(ENCODER_DT_PIN);

    CLK_REGISTER = portOutputRegister(DT_PORT);
    DT_REGISTER = portOutputRegister(CLK_PORT);
    // Get pointers to the input registers of the ports
}

void tearDown(void) {
    // clean stuff up here
}

int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(ConstructorInitializesPinsAndState);
    RUN_TEST(ProcessIdentifiesClockwiseRotation);
    RUN_TEST(ProcessIdentifiesCounterClockwiseRotation);
    RUN_TEST(InvalidStateTransitionsResetStateMachine);
    RUN_TEST(ProcessHandlesDebounceByStayingInIntermediateStates);
    return UNITY_END();
}

/**
  * For Arduino framework tests
  */
#include <Arduino.h>
void setup() {
    // Wait ~2 seconds before the Unity test runner
    // establishes connection with a board Serial interface

    delay(2000);

    runUnityTests();
}
void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}


