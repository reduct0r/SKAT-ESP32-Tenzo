#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <HX711.h>
#include <INA226.h>

#include "ble_module.h"
#include "motor_module.h"

// ── Пины (D18/D19/D21/D22 на DevKit = GPIO 18/19/21/22) ──
// HX711: DT → D18 (GPIO18), SCK → D19 (GPIO19)
static const uint8_t PIN_HX711_DT  = 18;
static const uint8_t PIN_HX711_SCK = 19;
static const uint8_t PIN_I2C_SDA   = 21;
static const uint8_t PIN_I2C_SCL   = 22;

static const char *BLE_DEVICE_NAME = "SKAT-Tenzo";

// ── INA226: внешний шунт 5 mΩ + опционально R100 (0.1 Ω) на модуле ──
static const float SHUNT_EXT_OHMS     = 0.005f;
static const float SHUNT_MODULE_OHMS  = 0.100f;  // SMD «R100» = 0.1 Ω
static const float MAX_MOTOR_AMPS     = 16.0f;

static const float DEFAULT_SCALE = 420.0f;

static HX711 scale;
static INA226 *ina226 = nullptr;
static Preferences prefs;

static float loadCellScale = DEFAULT_SCALE;
static float forceGrams    = 0.0f;
static float currentAmps   = 0.0f;
static float busVoltage    = 0.0f;
static float shuntExtOhms  = SHUNT_EXT_OHMS;
static float shuntBrdOhms  = SHUNT_MODULE_OHMS;
static bool  includeBoardShunt = false;
static float currentZeroAmps = 0.0f;
static float shuntVoltageMv = 0.0f;
static int8_t currentSign  = 1;
static int8_t forceSign    = 1;
static uint16_t busVScaleE4 = 10000;
static long  hx711Raw      = 0;
static bool  hx711Ok       = false;
static bool  ina226Ok      = false;
static bool  ina226CalOk   = false;
static uint8_t ina226Addr  = 0;
static char i2cScanResult[96] = "none";
static uint8_t i2cFoundAddrs[8];
static uint8_t i2cFoundCount = 0;

static void measureCurrentZero();

static float effectiveShuntOhms() {
  float total = shuntExtOhms;
  if (includeBoardShunt) {
    total += shuntBrdOhms;
  }
  if (total < 0.0001f) {
    total = SHUNT_EXT_OHMS;
  }
  return total;
}

static void applyIna226Calibration() {
  if (!ina226Ok || ina226 == nullptr) {
    return;
  }

  const float eff = effectiveShuntOhms();
  const float maxI = min(MAX_MOTOR_AMPS, 0.0819f / eff);

  ina226->setAverage(INA226_16_SAMPLES);
  ina226->setBusVoltageConversionTime(INA226_1100_us);
  ina226->setShuntVoltageConversionTime(INA226_1100_us);
  const int calErr = ina226->setMaxCurrentShunt(maxI, eff);
  if (calErr != INA226_ERR_NONE) {
    Serial.printf("INA226: cal error 0x%04X (shunt=%.4f Ohm, maxI=%.2f A)\n",
                  calErr, eff, maxI);
    ina226CalOk = false;
  } else {
    ina226CalOk = true;
    Serial.printf("INA226: cal OK, eff shunt=%.4f Ohm, maxI=%.2f A\n", eff, maxI);
  }
}

static void printChipInfo() {
  Serial.println("--- Informatsiya o plate ---");
  Serial.printf("Chip        : %s rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("CPU         : %d x %d MHz\n", ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("Flash       : %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.println("-----------------------------");
}

static uint8_t scanI2cBus() {
  Serial.println("--- Skanirovanie I2C (SDA=21, SCL=22) ---");
  Wire.setTimeOut(50);

  i2cScanResult[0] = '\0';
  i2cFoundCount = 0;
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  ustroystvo na 0x%02X\n", addr);
      if (i2cFoundCount < sizeof(i2cFoundAddrs)) {
        i2cFoundAddrs[i2cFoundCount++] = addr;
      }
      if (found == 0) {
        snprintf(i2cScanResult, sizeof(i2cScanResult), "0x%02X", addr);
      } else {
        char tmp[12];
        snprintf(tmp, sizeof(tmp), ",0x%02X", addr);
        strncat(i2cScanResult, tmp, sizeof(i2cScanResult) - strlen(i2cScanResult) - 1);
      }
      found++;
    } else if (err == 4) {
      Serial.println("  oshibka I2C (wire fault) — proverite provoda i zemlyu");
    }
  }

  if (found == 0) {
    strncpy(i2cScanResult, "none", sizeof(i2cScanResult));
    Serial.println("  ustroystv ne naydeno");
    Serial.println("  proverka:");
    Serial.println("    SDA -> GPIO21 (D21), SCL -> GPIO22 (D22)");
    Serial.println("    obshchaya GND ESP32 i modulya");
    Serial.println("    VCC modulya 3.3V (5V ne obyazatelno)");
  }

  Wire.setTimeOut(1000);
  Serial.println("-----------------------------------------");
  return found;
}

static bool probeIna226At(uint8_t addr, bool strictId) {
  INA226 probe(addr);
  if (!probe.begin()) {
    return false;
  }

  const uint16_t mfg = probe.getManufacturerID();
  const uint16_t die = probe.getDieID();
  Serial.printf("  adress 0x%02X: Manufacturer=0x%04X Die=0x%04X\n", addr, mfg, die);

  if (mfg == 0x5449 && die == 0x2260) {
    return true;
  }

  if (!strictId) {
    Serial.println("  ispolzuem ustroystvo po rezultatu skanirovaniya I2C");
    return true;
  }

  if (mfg != 0x0000 && mfg != 0xFFFF) {
    Serial.println("  vozmozhno klona INA226 — probuem ispolzovat");
    return true;
  }

  return false;
}

static uint8_t findIna226Address() {
  for (uint8_t i = 0; i < i2cFoundCount; i++) {
    const uint8_t addr = i2cFoundAddrs[i];
    if (addr >= 0x40 && addr <= 0x4F && probeIna226At(addr, false)) {
      return addr;
    }
  }

  static const uint8_t candidates[] = {0x44, 0x45, 0x40, 0x41, 0x42, 0x43, 0x46, 0x47};
  for (uint8_t addr : candidates) {
    if (probeIna226At(addr, true)) {
      return addr;
    }
  }
  return 0;
}

static String buildDataJson() {
  const float forceN = forceGrams * 0.00980665f;
  char json[640];
  snprintf(json, sizeof(json),
           "{\"force_g\":%.2f,\"force_n\":%.3f,\"current_a\":%.4f,\"bus_v\":%.2f,"
           "\"shunt_mv\":%.3f,\"shunt_ohm\":%.5f,\"shunt_ext\":%.5f,\"include_brd\":%s,"
           "\"current_zero_a\":%.4f,\"current_sign\":%d,\"force_sign\":%d,"
           "\"bus_v_scale_e4\":%u,"
           "\"hx711_raw\":%ld,"
           "\"hx711_ok\":%s,\"ina226_ok\":%s,\"ina226_cal\":%s,"
           "\"ina226_addr\":%u,\"i2c_scan\":\"%s\",\"scale\":%.2f,"
           "\"motors_armed\":%s,\"motor_pwm_pct\":%.1f,\"motor_pwm\":%u,"
           "\"esc_pulse_us\":%u,\"esc_min_us\":%u,\"esc_max_us\":%u}",
           forceGrams, forceN, currentAmps, busVoltage,
           shuntVoltageMv, effectiveShuntOhms(), shuntExtOhms,
           includeBoardShunt ? "true" : "false",
           currentZeroAmps, currentSign, forceSign,
           busVScaleE4,
           hx711Raw,
           hx711Ok ? "true" : "false",
           ina226Ok ? "true" : "false",
           ina226CalOk ? "true" : "false",
           ina226Addr,
           i2cScanResult,
           loadCellScale,
           skatMotorIsArmed() ? "true" : "false",
           skatMotorGetPercent(),
           skatMotorGetRawPwm(),
           skatMotorGetPulseUs(),
           skatMotorGetMinPulseUs(),
           skatMotorGetMaxPulseUs());
  return String(json);
}

static String jsonTare() {
  if (!hx711Ok) {
    return String("{\"ok\":false,\"error\":\"HX711 offline\"}");
  }
  if (!scale.wait_ready_timeout(2000)) {
    return String("{\"ok\":false,\"error\":\"HX711 timeout\"}");
  }
  scale.tare(5);
  const long off = scale.get_offset();
  prefs.putLong("offset", off);
  char json[96];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"tare\",\"offset\":%ld,\"scale\":%.3f}",
           off, loadCellScale);
  return String(json);
}

static String jsonCalibrate(float knownGrams) {
  if (!hx711Ok) {
    return String("{\"ok\":false,\"error\":\"HX711 offline\"}");
  }
  if (knownGrams <= 0.0f) {
    return String("{\"ok\":false,\"error\":\"invalid grams\"}");
  }
  if (!scale.wait_ready_timeout(3000)) {
    return String("{\"ok\":false,\"error\":\"HX711 timeout\"}");
  }

  // get_value = read_average − OFFSET (после tare без груза ≈ 0)
  const long avgCounts = static_cast<long>(scale.get_value(25));
  if (labs(avgCounts) < 1000) {
    return String("{\"ok\":false,\"error\":\"signal too weak — положите груз и не обнуляйте с грузом\"}");
  }

  loadCellScale = static_cast<float>(avgCounts) / knownGrams;
  scale.set_scale(loadCellScale);
  prefs.putFloat("scale", loadCellScale);

  const float verifyGrams = scale.get_units(5);
  forceGrams = verifyGrams;
  hx711Raw = scale.read();

  char json[220];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"calibrate\",\"scale\":%.3f,\"counts\":%ld,"
           "\"grams\":%.2f,\"force_g\":%.2f}",
           loadCellScale, avgCounts, knownGrams, verifyGrams);
  Serial.printf("[HX711] cal: counts=%ld scale=%.3f ref=%.1fg -> %.1fg\n",
                avgCounts, loadCellScale, knownGrams, verifyGrams);
  return String(json);
}

static String jsonSetScale(float newScale) {
  if (newScale <= 0.0f || newScale > 1000000.0f) {
    return String("{\"ok\":false,\"error\":\"invalid scale\"}");
  }

  loadCellScale = newScale;
  prefs.putFloat("scale", loadCellScale);
  if (hx711Ok) {
    scale.set_scale(loadCellScale);
  }

  char json[96];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"set_scale\",\"scale\":%.3f}",
           loadCellScale);
  return String(json);
}

static String jsonReset() {
  loadCellScale = DEFAULT_SCALE;
  prefs.putFloat("scale", loadCellScale);
  if (hx711Ok) {
    scale.set_scale(loadCellScale);
  }

  String response = String("{\"ok\":true,\"cmd\":\"reset\",\"scale\":") +
                    String(loadCellScale, 3) + String("}");
  if (hx711Ok) {
    const String tareResult = jsonTare();
    if (tareResult.indexOf("\"ok\":true") < 0) {
      response = String("{\"ok\":false,\"error\":\"scale reset ok, tare failed\"}");
    } else {
      response = String("{\"ok\":true,\"cmd\":\"reset\",\"scale\":") +
                 String(loadCellScale, 3) + String(",\"tare\":true}");
    }
  }
  return response;
}

static String jsonCalibrateBusV(float refVolts) {
  if (!ina226Ok || ina226 == nullptr) {
    return String("{\"ok\":false,\"error\":\"INA226 offline\"}");
  }
  if (refVolts < 1.0f || refVolts > 48.0f) {
    return String("{\"ok\":false,\"error\":\"ref_v must be 1..48 V\"}");
  }

  float sum = 0.0f;
  const int samples = 24;
  for (int i = 0; i < samples; i++) {
    ina226->waitConversionReady(200);
    sum += ina226->getBusVoltage();
    delay(10);
  }
  const float rawVolts = sum / static_cast<float>(samples);
  if (rawVolts < 0.5f) {
    return String("{\"ok\":false,\"error\":\"bus voltage too low — check VIN+\"}");
  }

  // Всегда от сырого значения чипа (scale=10000), не от текущего busVScaleE4 —
  const uint32_t newScale = static_cast<uint32_t>(
      lroundf(10000.0f * refVolts / rawVolts));
  if (newScale < 5000 || newScale > 20000) {
    return String("{\"ok\":false,\"error\":\"scale out of range\"}");
  }

  const uint16_t oldScale = busVScaleE4;
  busVScaleE4 = static_cast<uint16_t>(newScale);
  prefs.putUShort("bus_v_scale", busVScaleE4);

  char json[192];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"calibrate_bus_v\",\"ref_v\":%.3f,"
           "\"raw_v\":%.3f,\"bus_v_scale_e4\":%u,\"prev_scale_e4\":%u}",
           refVolts, rawVolts, busVScaleE4, oldScale);
  Serial.printf("[INA226] bus_v cal: ref=%.3f raw=%.3f scale %u -> %u\n",
                refVolts, rawVolts, oldScale, busVScaleE4);
  return String(json);
}

static String jsonRecalIna226() {
  if (!ina226Ok || ina226 == nullptr) {
    return String("{\"ok\":false,\"error\":\"INA226 offline\"}");
  }
  applyIna226Calibration();
  if (!ina226CalOk) {
    char json[96];
    snprintf(json, sizeof(json),
             "{\"ok\":false,\"error\":\"cal failed\"}");
    return String(json);
  }
  return String("{\"ok\":true,\"cmd\":\"recal_ina226\"}");
}

static String jsonSetShunt(float extOhm, float brdOhm, bool includeBrd) {
  if (extOhm < 0.0001f || extOhm > 0.5f) {
    return String("{\"ok\":false,\"error\":\"invalid ext shunt\"}");
  }
  if (brdOhm < 0.0001f || brdOhm > 0.5f) {
    brdOhm = SHUNT_MODULE_OHMS;
  }

  shuntExtOhms = extOhm;
  shuntBrdOhms = brdOhm;
  includeBoardShunt = includeBrd;
  prefs.putFloat("shunt_ext", shuntExtOhms);
  prefs.putFloat("shunt_brd", shuntBrdOhms);
  prefs.putBool("shunt_brd_inc", includeBoardShunt);

  currentZeroAmps = 0.0f;
  prefs.putFloat("i_zero_a", 0.0f);

  if (ina226Ok) {
    applyIna226Calibration();
  }

  char json[160];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"set_shunt\",\"shunt_ohm\":%.5f,"
           "\"shunt_ext\":%.5f,\"shunt_brd\":%.5f,\"include_brd\":%s}",
           effectiveShuntOhms(), shuntExtOhms, shuntBrdOhms,
           includeBoardShunt ? "true" : "false");
  return String(json);
}

static void measureCurrentZero() {
  if (!ina226Ok || ina226 == nullptr) {
    return;
  }

  const float eff = effectiveShuntOhms();
  float sum = 0.0f;
  const int samples = 32;
  for (int i = 0; i < samples; i++) {
    delay(25);
    ina226->waitConversionReady(200);
    sum += ina226->getShuntVoltage_mV() / 1000.0f / eff;
  }
  currentZeroAmps = sum / static_cast<float>(samples);
  prefs.putFloat("i_zero_a", currentZeroAmps);
  Serial.printf("[INA226] zero offset = %.4f A (eff shunt %.5f Ohm)\n",
                currentZeroAmps, eff);
}

static String jsonZeroCurrent() {
  if (!ina226Ok || ina226 == nullptr) {
    return String("{\"ok\":false,\"error\":\"INA226 offline\"}");
  }
  if (skatMotorIsArmed()) {
    return String("{\"ok\":false,\"error\":\"disarm motors first\"}");
  }
  measureCurrentZero();
  char json[96];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"zero_current\",\"current_zero_a\":%.4f}",
           currentZeroAmps);
  return String(json);
}

static String jsonSetCurrentSign(int sign) {
  if (sign != 1 && sign != -1) {
    return String("{\"ok\":false,\"error\":\"sign must be 1 or -1\"}");
  }
  currentSign = static_cast<int8_t>(sign);
  prefs.putInt("i_sign", sign);
  char json[80];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"set_current_sign\",\"current_sign\":%d}", sign);
  return String(json);
}

static String jsonSetForceSign(int sign) {
  if (sign != 1 && sign != -1) {
    return String("{\"ok\":false,\"error\":\"sign must be 1 or -1\"}");
  }
  forceSign = static_cast<int8_t>(sign);
  prefs.putInt("force_sign", sign);
  char json[80];
  snprintf(json, sizeof(json),
           "{\"ok\":true,\"cmd\":\"set_force_sign\",\"force_sign\":%d}", sign);
  return String(json);
}

static String extractJsonCmd(const String &json) {
  const int keyPos = json.indexOf("\"cmd\"");
  if (keyPos < 0) {
    return "";
  }
  const int colon = json.indexOf(':', keyPos);
  if (colon < 0) {
    return "";
  }
  const int q1 = json.indexOf('"', colon + 1);
  if (q1 < 0) {
    return "";
  }
  const int q2 = json.indexOf('"', q1 + 1);
  if (q2 < 0) {
    return "";
  }
  return json.substring(q1 + 1, q2);
}

static bool extractJsonFloat(const String &json, const char *key, float &out) {
  const String needle = String("\"") + key + "\"";
  const int keyPos = json.indexOf(needle);
  if (keyPos < 0) {
    return false;
  }
  const int colon = json.indexOf(':', keyPos);
  if (colon < 0) {
    return false;
  }
  out = json.substring(colon + 1).toFloat();
  return true;
}

static bool extractJsonBool(const String &json, const char *key, bool &out) {
  const String needle = String("\"") + key + "\"";
  const int keyPos = json.indexOf(needle);
  if (keyPos < 0) {
    return false;
  }
  const int colon = json.indexOf(':', keyPos);
  if (colon < 0) {
    return false;
  }
  String tail = json.substring(colon + 1);
  tail.trim();
  if (tail.startsWith("true")) {
    out = true;
    return true;
  }
  if (tail.startsWith("false")) {
    out = false;
    return true;
  }
  return false;
}

static String handleBleCommand(const String &cmdJson) {
  const String cmd = extractJsonCmd(cmdJson);
  if (cmd.length() == 0) {
    return String("{\"ok\":false,\"error\":\"cmd required\"}");
  }

  if (cmd == "get" || cmd == "data") {
    String data = buildDataJson();
    if (data.startsWith("{")) {
      return String("{\"ok\":true,\"type\":\"data\",") + data.substring(1);
    }
    return data;
  }
  if (cmd == "tare") {
    return jsonTare();
  }
  if (cmd == "calibrate") {
    float grams = 0.0f;
    if (!extractJsonFloat(cmdJson, "grams", grams)) {
      return String("{\"ok\":false,\"error\":\"grams required\"}");
    }
    return jsonCalibrate(grams);
  }
  if (cmd == "set_scale") {
    float scaleVal = 0.0f;
    if (!extractJsonFloat(cmdJson, "scale", scaleVal)) {
      return String("{\"ok\":false,\"error\":\"scale required\"}");
    }
    return jsonSetScale(scaleVal);
  }
  if (cmd == "reset") {
    return jsonReset();
  }
  if (cmd == "recal_ina226") {
    return jsonRecalIna226();
  }
  if (cmd == "calibrate_bus_v") {
    float refV = 0.0f;
    if (!extractJsonFloat(cmdJson, "ref_v", refV)) {
      return String("{\"ok\":false,\"error\":\"ref_v required\"}");
    }
    return jsonCalibrateBusV(refV);
  }
  if (cmd == "zero_current") {
    return jsonZeroCurrent();
  }
  if (cmd == "set_current_sign") {
    float signVal = 0.0f;
    if (!extractJsonFloat(cmdJson, "sign", signVal)) {
      return String("{\"ok\":false,\"error\":\"sign required\"}");
    }
    return jsonSetCurrentSign(static_cast<int>(signVal));
  }
  if (cmd == "set_force_sign") {
    float signVal = 0.0f;
    if (!extractJsonFloat(cmdJson, "sign", signVal)) {
      return String("{\"ok\":false,\"error\":\"sign required\"}");
    }
    return jsonSetForceSign(static_cast<int>(signVal));
  }
  if (cmd == "set_shunt") {
    float extOhm = shuntExtOhms;
    float brdOhm = shuntBrdOhms;
    bool includeBrd = includeBoardShunt;
    extractJsonFloat(cmdJson, "ext_ohm", extOhm);
    extractJsonFloat(cmdJson, "brd_ohm", brdOhm);
    extractJsonBool(cmdJson, "include_brd", includeBrd);
    return jsonSetShunt(extOhm, brdOhm, includeBrd);
  }
  if (cmd == "arm") {
    skatMotorArm(true);
    return String("{\"ok\":true,\"cmd\":\"arm\",\"motors_armed\":true}");
  }
  if (cmd == "disarm") {
    skatMotorArm(false);
    return String("{\"ok\":true,\"cmd\":\"disarm\",\"motors_armed\":false,\"motor_pwm_pct\":0}");
  }
  if (cmd == "set_pwm") {
    float pct = 0.0f;
    if (!extractJsonFloat(cmdJson, "pct", pct) && !extractJsonFloat(cmdJson, "pwm", pct)) {
      return String("{\"ok\":false,\"error\":\"pct required\"}");
    }
    if (!skatMotorIsArmed()) {
      return String("{\"ok\":false,\"error\":\"motors not armed\"}");
    }
    skatMotorSetPercent(pct);
    char json[160];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"cmd\":\"set_pwm\",\"motor_pwm_pct\":%.1f,"
             "\"motor_pwm\":%u,\"esc_pulse_us\":%u}",
             skatMotorGetPercent(), skatMotorGetRawPwm(), skatMotorGetPulseUs());
    return String(json);
  }

  return String("{\"ok\":false,\"error\":\"unknown cmd\"}");
}

static void readSensors() {
  if (hx711Ok && scale.wait_ready_timeout(200)) {
    hx711Raw = scale.read();
    forceGrams = scale.get_units(5) * static_cast<float>(forceSign);
  }

  if (ina226Ok && ina226 != nullptr) {
    shuntVoltageMv = ina226->getShuntVoltage_mV();
    busVoltage = ina226->getBusVoltage() *
                 (static_cast<float>(busVScaleE4) / 10000.0f);
    const float eff = effectiveShuntOhms();
    const float rawAmps = (shuntVoltageMv / 1000.0f) / eff;
    currentAmps = (rawAmps - currentZeroAmps) * static_cast<float>(currentSign);
  }
}

static bool initLoadCell() {
  if (loadCellScale <= 0.01f || loadCellScale > 100000.0f) {
    loadCellScale = DEFAULT_SCALE;
    prefs.putFloat("scale", loadCellScale);
  }

  scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
  scale.set_scale(loadCellScale);
  const long savedOffset = prefs.getLong("offset", 0);
  scale.set_offset(savedOffset);
  if (!scale.wait_ready_timeout(5000)) {
    Serial.println("HX711: net otveta (GPIO18 DT, GPIO19 SCK)");
    return false;
  }

  hx711Raw = scale.read();
  hx711Ok = true;
  Serial.printf("HX711: OK, scale=%.3f, offset=%ld, raw=%ld\n",
                loadCellScale, savedOffset, hx711Raw);
  if (labs(hx711Raw) < 500) {
    Serial.println("HX711: WARNING — raw mal, proverite provodku");
  }
  Serial.println("HX711: tare tolko cherez prilozhenie (bez gruza)");
  return true;
}

static bool initCurrentSensor() {
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  delay(10);

  scanI2cBus();

  ina226Addr = findIna226Address();
  if (ina226Addr == 0) {
    Serial.println("INA226: ne nayden na I2C");
    return false;
  }

  ina226 = new INA226(ina226Addr);
  if (!ina226->begin()) {
    Serial.printf("INA226: begin() fail na 0x%02X\n", ina226Addr);
    delete ina226;
    ina226 = nullptr;
    return false;
  }

  ina226Ok = true;
  prefs.putUChar("ina_addr", ina226Addr);
  Serial.printf("INA226: nayden na 0x%02X\n", ina226Addr);

  applyIna226Calibration();

  delay(100);
  busVoltage = ina226->getBusVoltage();
  shuntVoltageMv = ina226->getShuntVoltage_mV();
  if (busVoltage < 1.0f) {
    Serial.println("[INA226] WARNING: bus_v ~0 V — podkluchite VIN+ k BAT+");
  } else {
    Serial.printf("[INA226] bus_v=%.2f V, shunt=%.3f mV, eff=%.5f Ohm\n",
                  busVoltage, shuntVoltageMv, effectiveShuntOhms());
  }

  return true;
}

static void setupBle() {
  skatBleInit(
      BLE_DEVICE_NAME,
      [](const String &cmdJson) { return handleBleCommand(cmdJson); },
      []() { return buildDataJson(); });
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\nSKAT-Tenzo starting...");

  printChipInfo();

  prefs.begin("skat", false);
  loadCellScale = prefs.getFloat("scale", DEFAULT_SCALE);
  shuntExtOhms = prefs.getFloat("shunt_ext", prefs.getFloat("shunt_ohm", SHUNT_EXT_OHMS));
  shuntBrdOhms = prefs.getFloat("shunt_brd", SHUNT_MODULE_OHMS);
  includeBoardShunt = prefs.getBool("shunt_brd_inc", false);
  if (shuntExtOhms < 0.0001f || shuntExtOhms > 0.5f) {
    shuntExtOhms = SHUNT_EXT_OHMS;
  }
  if (shuntBrdOhms < 0.0001f || shuntBrdOhms > 0.5f) {
    shuntBrdOhms = SHUNT_MODULE_OHMS;
  }
  currentZeroAmps = prefs.getFloat("i_zero_a", 0.0f);
  if (fabsf(currentZeroAmps) > 20.0f) {
    currentZeroAmps = 0.0f;
  }
  currentSign = static_cast<int8_t>(prefs.getInt("i_sign", 1));
  if (currentSign != 1 && currentSign != -1) {
    currentSign = 1;
  }
  forceSign = static_cast<int8_t>(prefs.getInt("force_sign", 1));
  if (forceSign != 1 && forceSign != -1) {
    forceSign = 1;
  }
  busVScaleE4 = prefs.getUShort("bus_v_scale", 10000);
  if (busVScaleE4 < 5000 || busVScaleE4 > 20000) {
    busVScaleE4 = 10000;
  }

  initLoadCell();
  initCurrentSensor();
  skatMotorInit();
  Serial.println("[MOTOR] ESC arming: min pulse 2 s...");
  delay(2500);
  setupBle();

  Serial.printf("HX711 : %s\n", hx711Ok ? "OK" : "FAIL");
  Serial.printf("INA226: %s (addr 0x%02X, cal %s)\n",
                ina226Ok ? "OK" : "FAIL",
                ina226Addr,
                ina226CalOk ? "OK" : "fallback");
  Serial.println("BLE   : advertising as SKAT-Tenzo");
}

void loop() {
  skatBleProcessPendingCommand();
  readSensors();
  skatBleLoop();

  static uint32_t lastLog = 0;
  if (millis() - lastLog >= 3000) {
    lastLog = millis();
    Serial.printf("[%.0fs] F=%.1fg raw=%ld I=%.4fA sh=%.2fmV U=%.2fV ble=%s\n",
                  millis() / 1000.0f, forceGrams, hx711Raw, currentAmps, shuntVoltageMv,
                  busVoltage, skatBleConnected() ? "connected" : "adv");
  }

  delay(50);
}
