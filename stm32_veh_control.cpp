#include "stm32_veh_control.h"
//  Globals 

//  Configurable
const float DEAD_BAND = 6.0f;
float rate_limit = 6000.0f;

volatile uint32_t rx_steer_rise_us = 0, steer_pulse_us = PWM_NEU;
volatile uint32_t rx_throttle_rise_us = 0, throttle_pulse_us = PWM_NEU;
volatile unsigned long last_rx_time_ms = 0;

Servo SteerHandle;
Servo ESCHandle;

VehicleCmd cmd;
MODE mode = MANUAL;

//  Internal state 
float steer_us = PWM_NEU;
float throttle_us = PWM_NEU;
float target_steer_us = PWM_NEU;
float target_throttle_us = PWM_NEU;
unsigned long last_update_ms = 0;

//  Serial protocol 
const uint8_t START_BYTE = 0xAA;
const uint8_t STOP_BYTE  = 0x55;

enum PARSE_STATE { WAIT_START, READ_PAYLOAD, WAIT_STOP };
static PARSE_STATE state = WAIT_START;

uint8_t rx_buffer[sizeof(VehicleCmd)];
uint8_t rx_index = 0;

//  Interrupt Service Routines 
void rxSteerChange() {
  if (digitalRead(RX_STEER_PIN)) {
    rx_steer_rise_us = micros();
  } else {
    steer_pulse_us = (uint16_t)(micros() - rx_steer_rise_us);
    last_rx_time_ms = millis();
  }
}

void rxThrottleChange() {
  if (digitalRead(RX_THROTTLE_PIN)) {
    rx_throttle_rise_us = micros();
  } else {
    throttle_pulse_us = (uint16_t)(micros() - rx_throttle_rise_us);
    last_rx_time_ms = millis();
  }
}

//  Tasks 
void readSerialTask(void* arg) {
  for (;;) {
    while (CMD_SERIAL.available()) {
      uint8_t byte_in = CMD_SERIAL.read();

      switch (state) {
        case WAIT_START:
          if (byte_in == START_BYTE) {
            rx_index = 0;
            state = READ_PAYLOAD;
          }
          break;

        case READ_PAYLOAD:
          rx_buffer[rx_index++] = byte_in;
          if (rx_index >= sizeof(VehicleCmd))
            state = WAIT_STOP;
          break;

        case WAIT_STOP:
          if (byte_in == STOP_BYTE)
            memcpy(&cmd, rx_buffer, sizeof(VehicleCmd));
          state = WAIT_START;
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void readModeTask(void* arg) {
  pinMode(MODE_PIN, INPUT_PULLUP);
  for (;;) {
    bool rc_active = (millis() - last_rx_time_ms) < RX_TIMEOUT_MS;
    bool pi_mode_hi = digitalRead(MODE_PIN);
    mode = rc_active ? MANUAL : (pi_mode_hi ? AUTO : MANUAL);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void updateOutputs() {
  float steer = constrain(steer_us, PWM_MIN, PWM_MAX);
  float throttle = constrain(throttle_us, PWM_MIN, PWM_MAX);

  if (fabs(steer - PWM_NEU) < DEAD_BAND) steer = PWM_NEU;
  if (fabs(throttle - PWM_NEU) < DEAD_BAND) throttle = PWM_NEU;

  SteerHandle.writeMicroseconds(static_cast<uint16_t>(steer));
  ESCHandle.writeMicroseconds(static_cast<uint16_t>(throttle));
}

// Limits rate of change of PWM pulse width in us
void limitRate(float& current, float target, float rate_limit, float dt) {
  float diff = target - current;
  float max_step = rate_limit * dt; // how many us can it vary by per tick

  if(fabs(diff) <= max_step) {
    current = target;
  } else {
    current += (diff > 0 ? max_step : -max_step);
  }
}


void controlTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    // Placeholder for control logic (mixing, rate limiting, etc.)
    vTaskDelayUntil(&lastWake, period);
  }
}


void init() {
  // PWM I/O setup
  pinMode(RX_STEER_PIN, INPUT);
  pinMode(RX_THROTTLE_PIN, INPUT);

  SteerHandle.attach(STEER_OUT_PIN, PWM_MIN, PWM_MAX);
  ESCHandle.attach(ESC_OUT_PIN, PWM_MIN, PWM_MAX);

  attachInterrupt(digitalPinToInterrupt(RX_STEER_PIN), rxSteerChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_THROTTLE_PIN), rxThrottleChange, CHANGE);

  // Serial
  CMD_SERIAL.begin(115200);
  TELE_SERIAL.begin(115200);

  // Start tasks
  xTaskCreate(readSerialTask, "SerialCmd", 256, nullptr, 1, nullptr);
  xTaskCreate(readModeTask, "ModeTask", 128, nullptr, 1, nullptr);
  xTaskCreate(controlTask, "Control", 256, nullptr, 1, nullptr);
}