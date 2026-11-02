#pragma once

#include <Arduino.h>

// Hardware: Arduino UNO R4 WiFi and a 5 V HC-SR04-compatible module.
// These pins match Ultrasonic Project Draft 5.
namespace config {
constexpr uint8_t TRIG_PIN = 9;
constexpr uint8_t ECHO_PIN = 2;          // Interrupt-capable on UNO R4 WiFi.
constexpr uint8_t ENVELOPE_PIN = A0;     // Optional conditioned analog RX tap.

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr uint8_t ADC_BITS = 14;

constexpr uint32_t TRIGGER_PULSE_US = 10;
constexpr uint32_t MIN_PING_INTERVAL_US = 60000UL;
constexpr uint32_t ECHO_TIMEOUT_US = 30000UL;

// Analog capture window, relative to the trigger and ECHO falling edge.
// Values are provisional and need oscilloscope validation for the actual sensor.
constexpr uint32_t ANALOG_BLANKING_US = 300UL;
constexpr uint32_t ANALOG_POST_FALL_US = 800UL;

// Safety: do NOT attach an unverified internal sensor node to A0.
// Turn on capture via 'ANALOG ON' only after checking voltage, bias and loading.
constexpr bool ANALOG_ENABLED_AT_BOOT = false;
constexpr uint16_t MAX_BURST_COUNT = 1000;
}