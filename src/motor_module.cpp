#include "motor_module.h"

#include <Preferences.h>

// пример Skywalker HobbyKing 40A ESC — стандарт RC PWM (как сервопривод):
// 50 Гц, 1000 µs = мин/стоп, 2000 µs = макс.
static const uint8_t PIN_MOTOR_A = 25;
static const uint8_t PIN_MOTOR_B = 26;
static const uint8_t LEDC_CHANNEL_A = 0;
static const uint8_t LEDC_CHANNEL_B = 1;

static const uint32_t ESC_FREQ_HZ = 50;
static const uint16_t ESC_PERIOD_US = 20000;
static const uint16_t ESC_DEFAULT_MIN_US = 1000;
static const uint16_t ESC_DEFAULT_MAX_US = 2000;
static const uint8_t LEDC_RES_BITS = 14;

static bool g_armed = false;
static float g_percent = 0.0f;
static uint16_t g_pulseUs = ESC_DEFAULT_MIN_US;
static uint16_t g_minUs = ESC_DEFAULT_MIN_US;
static uint16_t g_maxUs = ESC_DEFAULT_MAX_US;

static uint32_t maxDuty() {
  return (1u << LEDC_RES_BITS) - 1u;
}

static uint16_t pulseUsToDuty(uint16_t pulseUs) {
  pulseUs = constrain(pulseUs, g_minUs, g_maxUs);
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(pulseUs) * maxDuty()) / ESC_PERIOD_US);
}

static void writePulseUs(uint16_t pulseUs) {
  pulseUs = constrain(pulseUs, g_minUs, g_maxUs);
  g_pulseUs = pulseUs;
  const uint16_t duty = pulseUsToDuty(pulseUs);
  ledcWrite(LEDC_CHANNEL_A, duty);
  ledcWrite(LEDC_CHANNEL_B, duty);
}

static uint16_t percentToPulseUs(float percent) {
  if (percent <= 0.0f) {
    return g_minUs;
  }
  if (percent >= 100.0f) {
    return g_maxUs;
  }
  return static_cast<uint16_t>(
      g_minUs + (g_maxUs - g_minUs) * (percent / 100.0f) + 0.5f);
}

void skatMotorInit() {
  Preferences prefs;
  prefs.begin("skat", true);
  g_minUs = prefs.getUShort("esc_min", ESC_DEFAULT_MIN_US);
  g_maxUs = prefs.getUShort("esc_max", ESC_DEFAULT_MAX_US);
  prefs.end();

  if (g_minUs < 800 || g_minUs > 1500) {
    g_minUs = ESC_DEFAULT_MIN_US;
  }
  if (g_maxUs < 1500 || g_maxUs > 2200 || g_maxUs <= g_minUs) {
    g_maxUs = ESC_DEFAULT_MAX_US;
  }

  ledcSetup(LEDC_CHANNEL_A, ESC_FREQ_HZ, LEDC_RES_BITS);
  ledcSetup(LEDC_CHANNEL_B, ESC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(PIN_MOTOR_A, LEDC_CHANNEL_A);
  ledcAttachPin(PIN_MOTOR_B, LEDC_CHANNEL_B);

  g_armed = false;
  g_percent = 0.0f;
  writePulseUs(g_minUs);

  Serial.printf("[MOTOR] ESC 50Hz GPIO25/26, pulse %u–%u us (disarmed)\n",
                g_minUs, g_maxUs);
}

void skatMotorArm(bool armed) {
  g_armed = armed;
  if (!g_armed) {
    g_percent = 0.0f;
    writePulseUs(g_minUs);
    Serial.printf("[MOTOR] DISARM -> %u us\n", g_minUs);
  } else {
    Serial.println("[MOTOR] ARMED (throttle allowed)");
  }
}

bool skatMotorIsArmed() {
  return g_armed;
}

void skatMotorSetPercent(float percent) {
  if (!g_armed) {
    return;
  }

  if (percent < 0.0f) {
    percent = 0.0f;
  } else if (percent > 100.0f) {
    percent = 100.0f;
  }

  g_percent = percent;
  writePulseUs(percentToPulseUs(g_percent));
}

float skatMotorGetPercent() {
  return g_percent;
}

uint16_t skatMotorGetPulseUs() {
  return g_pulseUs;
}

uint16_t skatMotorGetMinPulseUs() {
  return g_minUs;
}

uint16_t skatMotorGetMaxPulseUs() {
  return g_maxUs;
}

uint16_t skatMotorGetRawPwm() {
  return g_pulseUs;
}
