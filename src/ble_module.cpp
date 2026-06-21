#include "ble_module.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "motor_module.h"

static const char *kServiceUuid = "a1b2c3d4-e5f6-7890-abcd-ef1234567890";
static const char *kTelemetryUuid = "a1b2c3d4-e5f6-7890-abcd-ef1234567891";
static const char *kCommandUuid = "a1b2c3d4-e5f6-7890-abcd-ef1234567892";
static const char *kResponseUuid = "a1b2c3d4-e5f6-7890-abcd-ef1234567893";

static const uint32_t kTelemetryIntervalMs = 500;

static BLECharacteristic *g_telemetryChar = nullptr;
static BLECharacteristic *g_responseChar = nullptr;

static bool g_deviceConnected = false;
static bool g_oldDeviceConnected = false;
static uint32_t g_lastTelemetryMs = 0;

static BleCommandHandler g_onCommand;
static BleTelemetryProvider g_telemetryProvider;

static String g_pendingCmdJson;
static volatile bool g_cmdPending = false;

static bool parseJsonFloat(const String &json, const char *key, float &out) {
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

static bool isDeferredBleCommand(const String &cmdJson) {
  return cmdJson.indexOf("\"set_pwm\"") >= 0 ||
         cmdJson.indexOf("\"tare\"") >= 0 ||
         cmdJson.indexOf("\"calibrate\"") >= 0 ||
         cmdJson.indexOf("\"reset\"") >= 0 ||
         cmdJson.indexOf("\"zero_current\"") >= 0 ||
         cmdJson.indexOf("\"recal_ina226\"") >= 0 ||
         cmdJson.indexOf("\"calibrate_bus_v\"") >= 0 ||
         cmdJson.indexOf("\"set_shunt\"") >= 0;
}

static bool isImmediateBleCommand(const String &cmdJson) {
  return cmdJson.indexOf("\"arm\"") >= 0 ||
         cmdJson.indexOf("\"disarm\"") >= 0 ||
         cmdJson.indexOf("\"get\"") >= 0 ||
         cmdJson.indexOf("\"data\"") >= 0 ||
         cmdJson.indexOf("\"set_current_sign\"") >= 0 ||
         cmdJson.indexOf("\"set_force_sign\"") >= 0;
}

static void notifyResponse(const String &json) {
  if (g_responseChar == nullptr || !g_deviceConnected) {
    return;
  }
  g_responseChar->setValue(json.c_str());
  g_responseChar->notify();
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    g_deviceConnected = true;
    g_lastTelemetryMs = millis();
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    g_deviceConnected = false;
    skatMotorArm(false);
    Serial.println("[BLE] Client disconnected — motors DISARM");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    const std::string value = characteristic->getValue();
    if (value.empty() || !g_onCommand) {
      return;
    }

    const String cmdJson = String(value.c_str());

    if (cmdJson.indexOf("\"set_pwm\"") >= 0) {
      float pct = 0.0f;
      if (parseJsonFloat(cmdJson, "pct", pct) ||
          parseJsonFloat(cmdJson, "pwm", pct)) {
        if (pct < 0.0f) {
          pct = 0.0f;
        } else if (pct > 100.0f) {
          pct = 100.0f;
        }
        if (skatMotorIsArmed()) {
          skatMotorSetPercent(pct);
        }
      }
      return;
    }

    if (isImmediateBleCommand(cmdJson)) {
      const String response = g_onCommand(cmdJson);
      notifyResponse(response);
      return;
    }

    if (isDeferredBleCommand(cmdJson)) {
      Serial.print("[BLE] Command: ");
      Serial.println(value.c_str());
      g_pendingCmdJson = cmdJson;
      g_cmdPending = true;
    }
  }
};

void skatBleInit(const char *deviceName,
                 BleCommandHandler onCommand,
                 BleTelemetryProvider telemetryProvider) {
  g_onCommand = std::move(onCommand);
  g_telemetryProvider = std::move(telemetryProvider);

  BLEDevice::init(deviceName);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(kServiceUuid);

  g_telemetryChar = service->createCharacteristic(
      kTelemetryUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_telemetryChar->addDescriptor(new BLE2902());
  g_telemetryChar->setValue("{}");

  BLECharacteristic *commandChar = service->createCharacteristic(
      kCommandUuid,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);
  commandChar->setCallbacks(new CommandCallbacks());

  g_responseChar = service->createCharacteristic(
      kResponseUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_responseChar->addDescriptor(new BLE2902());
  g_responseChar->setValue("{\"ok\":true,\"msg\":\"ready\"}");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] GATT server started");
  Serial.print("[BLE] Device name: ");
  Serial.println(deviceName);
  Serial.print("[BLE] Service UUID: ");
  Serial.println(kServiceUuid);
}

void skatBleLoop() {
  if (g_deviceConnected && !g_oldDeviceConnected) {
    g_oldDeviceConnected = true;
    g_lastTelemetryMs = millis();
  }

  if (!g_deviceConnected && g_oldDeviceConnected) {
    g_oldDeviceConnected = false;
  }

  if (!g_deviceConnected || g_telemetryChar == nullptr || !g_telemetryProvider) {
    return;
  }

  const uint32_t now = millis();
  if (now - g_lastTelemetryMs < kTelemetryIntervalMs) {
    return;
  }
  g_lastTelemetryMs = now;

  const String telemetry = g_telemetryProvider();
  g_telemetryChar->setValue(telemetry.c_str());
  g_telemetryChar->notify();
}

void skatBleProcessPendingCommand() {
  if (!g_cmdPending || !g_onCommand) {
    return;
  }
  g_cmdPending = false;
  const String cmdJson = g_pendingCmdJson;
  g_pendingCmdJson = "";
  Serial.println("[BLE] Processing deferred command");
  const String response = g_onCommand(cmdJson);
  notifyResponse(response);
}

bool skatBleConnected() {
  return g_deviceConnected;
}
