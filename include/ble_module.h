#pragma once

#include <Arduino.h>
#include <functional>

using BleTelemetryProvider = std::function<String()>;
using BleCommandHandler = std::function<String(const String &cmdJson)>;

void skatBleInit(const char *deviceName,
                 BleCommandHandler onCommand,
                 BleTelemetryProvider telemetryProvider);

void skatBleLoop();

/** Выполнить отложенную BLE-команду (tare, calibrate и т.д.) — только из loop(). */
void skatBleProcessPendingCommand();

bool skatBleConnected();
