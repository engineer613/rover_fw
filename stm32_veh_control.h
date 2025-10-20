#pragma once
#include <Arduino.h>
#include <Servo.h>
#include <STM32FreeRTOS.h>

//  Pin Assignments 
constexpr uint8_t RX_STEER_PIN    = PA0;
constexpr uint8_t RX_THROTTLE_PIN = PA1;
constexpr uint8_t STEER_OUT_PIN   = PA8;
constexpr uint8_t ESC_OUT_PIN     = PA10;
constexpr uint8_t MODE_PIN        = PB0;

//  PWM Limits 
constexpr uint16_t PWM_MIN = 1000;
constexpr uint16_t PWM_NEU = 1500;
constexpr uint16_t PWM_MAX = 2000;

//  RX Timeout 
constexpr unsigned long RX_TIMEOUT_MS = 200;

//  Serial Definitions 
#define CMD_SERIAL Serial
#define TELE_SERIAL Serial1

//  Structs and Enums 
struct VehicleCmd {
  float steer_pwm;
  float steer_rate_limit;
  float steer_rate_cmd;

  float throttle_pwm;
  float throttle_rate_limit;
  float throttle_rate_cmd;

  float jerk_limit;
};

enum MODE {
  MANUAL,
  ACTIVE,
  READY,
  AUTO
};

//  Global Variables 
extern volatile uint32_t steer_pulse_us;
extern volatile uint32_t throttle_pulse_us;
extern volatile unsigned long last_rx_time_ms;
extern MODE mode;
extern VehicleCmd cmd;
extern Servo SteerHandle;
extern Servo ESCHandle;

//  Function Declarations 
void initFirmware();

void rxSteerChange();
void rxThrottleChange();

void readSerialTask(void* arg);
void readModeTask(void* arg);
void controlTask(void* arg);

void updateOutputs();
