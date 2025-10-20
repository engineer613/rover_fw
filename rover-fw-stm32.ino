#include <Arduino.h>
#include <Servo.h>
#include <STM32FreeRTOS.h>

/*  PIN ASSIGNMENTS */

const uint8_t RX_STEER_PIN   = PA0;   // RC steering input (TIM2_CH1)
const uint8_t RX_THROTTLE_PIN= PA1;   // RC throttle input (TIM2_CH2)

Servo SteerHandle;
Servo ESCHandle;
const uint8_t STEER_OUT_PIN  = PA8;   // Steering servo output (TIM1_CH1)
const uint8_t ESC_OUT_PIN    = PA10;   // ESC output (TIM1_CH2)

#define CMD_SERIAL Serial // MicroUSB
#define TELE_SERIAL Serial1 // PA9 and PA10 pins for serial telemetry

/* ----------------------- RC Receiver Signal Detection ------------------------------------ */

volatile uint32_t rx_steer_rise_us = 0, steer_pulse_us = PWM_NEU;
volatile uint32_t rx_throttle_rise_us = 0, throttle_pulse_us = PWM_NEU;
volatile unsigned long last_rx_time_ms = 0;
const unsigned long RX_TIMEOUT_MS = 200;   // 0.2 s

// PWM Limits
const uint16_t STEER_PWM_MIN = 1000;
const uint16_t STEER_PWM_NEU = 1500;
const uint16_t STEER_PWM_MAX = 2000;

const uint16_t ESC_PWM_MIN = 1000;
const uint16_t ESC_PWM_NEU = 1500;
const uint16_t ESC_PWM_MAX = 2000;

// Rate Limit
float steer_us = PWM_NEU;
float throttle_us = PWM_NEU;
float target_steer_us = PWM_NEU;
float target_throttle_us = PWM_NEU;
float rate_limit = 6000.0f;   // µs per second
unsigned long last_update_ms = 0;


// Interrupt Service Routines for rising edges of PWM signals from Radio receiver
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


/* --------------------SERIAL COMMANDS FROM PI--------------------------------- */

struct VehicleCmd {
  float steer_pwm;
  float steer_rate_limit;
  float throttle_pwm;
};

VehicleCmd cmd;

const uint8_t START_BYTE = 0xAA;
const uint8_t STOP_BYTE = 0x55;

enum PARSE_STATE {
  WAIT_START,
  READ_CMD,
  WAIT_STOP
};

PARSE_STATE pstate = WAIT_START;

uint8_t rx_buffer[sizeof(VehicleCmd)];
uint8_t rx_index = 0;

// Task to read serial commands from Pi
void readSerialTask(void* arg) {
  for(;;) {
    while (CMD_SERIAL.available()) {
      uint8_t byte_in = CMD_SERIAL.read();

      switch (state) {
        case WAIT_START:
          if(byte_in == START_BYTE) {
            rx_index = 0;
            state = READ_PAYLOAD;
          }
          break;
        
        case READ_PAYLOAD:
          rx_buffer[rx_index++] = byte_in;
          if(rx_index >= sizeof(VehicleCmd))
            state = WAIT_STOP;
          break;

        case WAIT_STOP:
          if(byte_in == STOP_BYTE) {
            memcpy(&cmd, rx_buffer, sizeof(VehicleCmd));
          }
          state = WAIT_START;
          break;
      }   
    }

    vTaskDelay(pdMS_TO_TICKS(2)); // small delay so other tasks can run
  }
}


/* ------------------------------MODE LOGIC------------------------------------------- */

const uint8_t MODE_PIN = PB0;

// MODE
enum MODE {
  MANUAL,
  ACTIVE,
  READY, 
  AUTO
};

MODE mode = MANUAL;

// Task to determine mode based on GPIO pin (set by Pi) and Radio RX PWM signals
void readModeTask(void *arg) {
  pinMode(MODE_PIN, INPUT_PULLUP);

  for(;;) {
    bool rc_active = (millis() - last_rx_time_ms) < RX_TIMEOUT_MS;
    bool pi_mode_hi = digitalRead(MODE_PIN);

    // If Radio receiver is high, override and set to MANUAL
    // Else let the Pi signal the mode over GPIO (HIGH = AUTO, LOW = MANUAL)
    if (rc_active) {
      mode = MANUAL;
    } else {
      mode = pi_mode_hi ? AUTO : MANUAL;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//----------------------------------------------------------------------------------------

void updateOutputs() {
  float steer = constrain(steer_us, PWM_MIN, PWM_MAX);
  float throttle = constrain(throttle_us, PWM_MIN, PWM_MAX);

  const float DEAD_BAND = 3.0f; // 3 us

  if(fabs(steer - PWM_NEU) <  DEAD_BAND) steer = PWM_NEU;
  if(fabs(throttle - PWM_NEU) < DEAD_BAND) throttle = PWM_NEU;

  SteerHandle.writeMicroseconds((uint16_t)steer);
  ESCHandle.writeMicroseconds((uint16_t)throttle);
}



void controlTask(void * arg) {
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t lastWake = xTaskGetTickCount(); // returns current time in RTOS ticks
}


void setup() {
  pinMode(RX_STEER_PIN, INPUT);
  pinMode(RX_THROTTLE_PIN, INPUT);

  SteerHandle.attach(STEER_OUT_PIN, PWM_MIN, PWM_MAX);
  ESCHandle.attach(STEER_OUT_PIN, PWM_MIN, PWM_MAX);

  attachInterrupt(digitalPinToInterrupt(RX_STEER_PIN), rxSteerChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_THROTTLE_PIN), rxThrottleChange, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:

}
