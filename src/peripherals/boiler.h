#ifndef BOILER_H
#define BOILER_H

#include "../utils.h"
#include "../peripherals/peripherals.h"
#include "../peripherals/pump.h"
#include "../eeprom_data.h"
#include "../sensors_state.h"
#include <Arduino.h>


#define STEAM_TEMPERATURE         162.f
#define STEAM_WAND_HOT_WATER_TEMP 105.f
#define DELTA_RANGE             0.25f // % to apply as delta

// Values used to estimate energy spent
#define IDLE_POWER_USE 5
#define PUMP_POWER 48.0

// characteristics of the temperature controller -- model by Tom Brazier https://github.com/tombrazier/hotmetal
#define MAX_HEATER_POWER 2280.0             // W at 240V (43 Ohm elements in semi-parallel)
#define SAFE_HEATER_POWER 2280.0            // W power beyond which I dare not drive the beast
#define ELEMENT_MAX_TEMPERATURE 190.0       // °C the thermal fuses melt at 172°C and elements are about 20K hotter than fuse mountings
#define SHELL_MAX_TEMPERATURE 170.0         // °C the temp probe circuit maxes out at about 155°C - don't exceed it or we're flying blind
#define AMBIENT_TEMPERATURE 20.0            // °C this does not need to be exact
#define RESERVOIR_TEMPERATURE 20.0          // °C the reservoir gets heated a little by the machine over time - model this?
#define LATENT_HEAT_VAPORISATION_100 2257.0 // J/g latent heat of vaporisation of water at 100°C
#define SPEC_HEAT_WATER_100 4.216           // J/g/K specific heat of water at 100°C
#define SPEC_HEAT_ALUMINIUM 0.9             // J/g/K specific heat of aluminium (i.e. boiler shell)
#define SPEC_HEAT_BRASS 0.38                // J/g/K specific heat of brass (i.e. brew head)
#define BOILER_VOLUME 100.0                 // ml volume of water in boiler when full (I measured it)
#define MASS_BOILER_SHELL 609.0             // g mass of the aluminium boiler shell (I measured it)
#define MASS_BREW_HEAD 1172.0               // g mass of the brew head (I measured it)
#define MASS_PORTAFILTER 290.0              // g mass of the brew head (I measured it)
#define MASS_PORTAFILTER_2_SPOUT 490.0
#define HEAT_CAPACITY_BODY 395.0            // J/K heat capacity of the body/housing of the machine (i.e. what is lost once-off during startup)
#define BREWHEAD_AMBIENT_XFER_COEFF 0.55    // W/K power lost from brew head to ambient air
#define BODY_AMBIENT_XFER_COEFF 0.6         // W/K power lost from body to ambient air
#define BOILER_WATER_XFER_COEFF_NOFLOW 14.7 // W/K rate at which boiler shell heats water per °C difference in boiler and water temperature
#define BOILER_WATER_XFER_COEFF_STEAM 25.0  // W/K ditto for when steam is flowing
#define WATER_GROUPHEAD_XFER_COEFF 44.6     // W/K water to brewhead
#define BOILER_BREWHEAD_XFER_COEFF 3.3      // W/K ditto for the brewhead (calculated from measuring brewhead temperature at 60s of full power=14.2K+ambient and boiler temp=67.21K+ambient and rate of change of brewhead temp=0.43K/s)
#define ELEMENT_SHELL_XFER_COEFF 14.0       // W/K rate at which heat transfers from element half of boiler to shell half
#define BOILER_BODY_XFER_COEFF 6.0          // W/K rate at which heat transfers into the body/housing of the machine


void justDoCoffee(eepromValues_t &runningCfg, SensorState &currentState, bool brewActive, bool preinfusionFinished);
void steamCtrl(eepromValues_t &runningCfg, SensorState &currentState, bool brewActive);

void setBoilerOn(void);
void setBoilerOff(void);

#endif

