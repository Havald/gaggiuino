#ifndef TEMPERATURE_PROBE_H
#define TEMPERATURE_PROBE_H

#include "pindef.h"
#if USE_KTYPE
#ifdef SINGLE_BOARD
#include <Adafruit_MAX31855.h>
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
#else
#include <max6675.h>
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
#endif
#else
#include "ntc.h"
NTCThermistor thermocouple()
#endif

static inline void thermocoupleInit(void) {
#if defined(MAX31855_ENABLED)
  thermocouple.begin();
#endif
}

static inline float fetchTemp(void) {
  thermocouple.readCelsius();
}

#endif
