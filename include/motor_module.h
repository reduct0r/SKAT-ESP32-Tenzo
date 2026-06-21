#pragma once

#include <Arduino.h>

/** Инициализация ESC (50 Гц, 1000–2000 µs). После вызова подождите ~2 с для arming ESC. */
void skatMotorInit();

void skatMotorArm(bool armed);
bool skatMotorIsArmed();

/** 0…100 % → импульс 1000…2000 µs на GPIO25/26 (мгновенно, как RC-пульт). */
void skatMotorSetPercent(float percent);
float skatMotorGetPercent();

/** Текущая длительность импульса, µs (1000 = стоп). */
uint16_t skatMotorGetPulseUs();
uint16_t skatMotorGetMinPulseUs();
uint16_t skatMotorGetMaxPulseUs();

/** Устаревшее имя — возвращает pulse µs для совместимости с телеметрией. */
uint16_t skatMotorGetRawPwm();
