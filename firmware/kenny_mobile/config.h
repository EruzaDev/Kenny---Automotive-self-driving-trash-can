#pragma once
#include <cstdint>

#define KENNY_DIRECT_GPIO 1
#define KENNY_CARRIER_MCP23017 2

// IMPORTANT: select the wiring actually installed before uploading.
// 1 = the earlier independent-GPIO L298N wiring.
// 2 = hardware/kenny_carrier PCB (MCP23017 directions, E-stop GPIO18).
#ifndef KENNY_WIRING
#define KENNY_WIRING KENNY_DIRECT_GPIO
#endif
#if KENNY_WIRING != KENNY_DIRECT_GPIO && KENNY_WIRING != KENNY_CARRIER_MCP23017
#error "Select KENNY_DIRECT_GPIO or KENNY_CARRIER_MCP23017"
#endif

constexpr char WIFI_SSID[] = "KENNY-Robot";
constexpr char WIFI_PASSWORD[] = "KennyDrive!2026";  // Default local Wi-Fi password.
static_assert(sizeof(WIFI_PASSWORD) >= 9, "Wi-Fi password must have at least 8 characters");
constexpr uint32_t ENCODER_CPR = 1320;  // Expected x4 count; replace with measured CPR.
constexpr float WHEEL_RADIUS = 0.040f;
constexpr float NOMINAL_RPM = 333.0f;
constexpr int MAX_PWM = 90;
constexpr float KP = 1.5f, KI = 0.5f;  // Commissioning gains; tune on raised wheels.
constexpr float RPM_SLEW_PER_SECOND = 40.0f;
constexpr uint32_t ENCODER_TIMEOUT_MS = 500;
constexpr unsigned MOTOR_COUNT = 4;

#if KENNY_WIRING == KENNY_CARRIER_MCP23017
constexpr uint8_t ESTOP_PIN = 18;
constexpr uint8_t I2C_SDA = 21, I2C_SCL = 22, MCP_ADDRESS = 0x20;
constexpr uint8_t HX_DOUT = 16, HX_SCK = 17;
constexpr char WIRING_NAME[] = "Carrier PCB / MCP23017";
#else
constexpr uint8_t ESTOP_PIN = 15;
constexpr char WIRING_NAME[] = "Direct GPIO";
#endif
