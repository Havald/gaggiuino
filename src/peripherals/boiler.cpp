#include "boiler.h"

//delta stuff
inline static float TEMP_DELTA(float d) { return (d*DELTA_RANGE); }

void justDoCoffee(eepromValues_t &runningCfg, SensorState &currentState, bool brewActive, bool preinfusionFinished) {
  int HPWR_LOW = runningCfg.hpwr / runningCfg.mainDivider;
  static double heaterWave;
  static bool heaterState;
  float BREW_TEMP_DELTA;
  // Calculating the boiler heating power range based on the below input values
  int HPWR_OUT = mapRange(currentState.temperature, runningCfg.setpoint - 10, runningCfg.setpoint, runningCfg.hpwr, HPWR_LOW, 0);
  HPWR_OUT = constrain(HPWR_OUT, HPWR_LOW, runningCfg.hpwr);  // limits range of sensor values to HPWR_LOW and HPWR
  BREW_TEMP_DELTA = mapRange(currentState.temperature, runningCfg.setpoint, runningCfg.setpoint + TEMP_DELTA(runningCfg.setpoint), TEMP_DELTA(runningCfg.setpoint), 0, 0);
  BREW_TEMP_DELTA = constrain(BREW_TEMP_DELTA, 0, TEMP_DELTA(runningCfg.setpoint));

  if (brewActive) {
  // Applying the HPWR_OUT variable as part of the relay switching logic
    if (currentState.temperature > runningCfg.setpoint && currentState.temperature < runningCfg.setpoint + 0.25f && !preinfusionFinished ) {
      if (millis() - heaterWave > HPWR_OUT * runningCfg.brewDivider && !heaterState ) {
        setBoilerOff();
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR_LOW * runningCfg.mainDivider && heaterState ) {
        setBoilerOn();
        heaterState=false;
        heaterWave=millis();
      }
    } else if (currentState.temperature > runningCfg.setpoint - 1.5f && currentState.temperature < runningCfg.setpoint + (runningCfg.brewDeltaState ? BREW_TEMP_DELTA : 0.f) && preinfusionFinished ) {
      if (millis() - heaterWave > runningCfg.hpwr * runningCfg.brewDivider && !heaterState ) {
        setBoilerOn();
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > runningCfg.hpwr && heaterState ) {
        setBoilerOff();
        heaterState=false;
        heaterWave=millis();
      }
    } else if (runningCfg.brewDeltaState && currentState.temperature >= (runningCfg.setpoint + BREW_TEMP_DELTA) && currentState.temperature <= (runningCfg.setpoint + BREW_TEMP_DELTA + 2.5f)  && preinfusionFinished ) {
      if (millis() - heaterWave > runningCfg.hpwr * runningCfg.mainDivider && !heaterState ) {
        setBoilerOn();
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > runningCfg.hpwr && heaterState ) {
        setBoilerOff();
        heaterState=false;
        heaterWave=millis();
      }
    } else if(currentState.temperature <= runningCfg.setpoint - 1.5f) {
      setBoilerOn();
    } else {
      setBoilerOff();
    }
  } else { //if brewState == 0
    if (currentState.temperature < ((float)runningCfg.setpoint - 10.00f)) {
      setBoilerOn();
    } else if (currentState.temperature >= ((float)runningCfg.setpoint - 10.f) && currentState.temperature < ((float)runningCfg.setpoint - 5.f)) {
      if (millis() - heaterWave > HPWR_OUT && !heaterState) {
        setBoilerOn();
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR_OUT / runningCfg.brewDivider && heaterState ) {
        setBoilerOff();
        heaterState=false;
        heaterWave=millis();
      }
    } else if ((currentState.temperature >= ((float)runningCfg.setpoint - 5.f)) && (currentState.temperature <= ((float)runningCfg.setpoint - 0.25f))) {
      if (millis() - heaterWave > HPWR_OUT * runningCfg.brewDivider && !heaterState) {
        setBoilerOn();
        heaterState=!heaterState;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR_OUT / runningCfg.brewDivider && heaterState ) {
        setBoilerOff();
        heaterState=!heaterState;
        heaterWave=millis();
      }
    } else {
      setBoilerOff();
    }
  }
}

//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################

void steamCtrl(eepromValues_t &runningCfg, SensorState &currentState, bool brewActive) {
    // steam temp control, needs to be aggressive to keep steam pressure acceptable
  if ((currentState.temperature > runningCfg.setpoint - 10.f) && (currentState.temperature <= STEAM_WAND_HOT_WATER_TEMP)) {
    setBoilerOn();
    brewActive ? setPumpFullOn() : setPumpOff();
  }else if ((currentState.pressure <= 9.f) && (currentState.temperature > STEAM_WAND_HOT_WATER_TEMP) && (currentState.temperature <= STEAM_TEMPERATURE)) {
    setBoilerOn();
    if (currentState.pressure < 1.5f) {
      #ifndef SINGLE_BOARD
        openValve();
      #endif
      setPumpToRawValue(5);
    }
  } else {
    setBoilerOff();
    (currentState.pressure < 1.5f) ? setPumpToRawValue(5) : setPumpOff();
  }
}


  float estimateTemps() {

    static double pumpPowerRate = 0.0;
    static double heaterPower = 0.0;
    static bool first = true;
    static double boilingPoint = 100.0;
    static unsigned int lastTime = millis();
    if (tempC < 5 || millis() < 6000) {
        return 0;
    }
    if (first || millis() - lastTime > 1000) {
        shellTemp = tempC;
        ambientTemp = min(shellTemp, AMBIENT_TEMPERATURE);
        elementTemp = shellTemp;
        waterTemp = shellTemp;
        waterTempWithoutInflow = waterTemp;
        // based on heat transfer coefficients to and from brewhead, we can caluculate its steady state temp - use that
        brewHeadTemp = (ambientTemp * BREWHEAD_AMBIENT_XFER_COEFF + shellTemp * BOILER_BREWHEAD_XFER_COEFF) / (BREWHEAD_AMBIENT_XFER_COEFF + BOILER_BREWHEAD_XFER_COEFF);
        bodyTemp = shellTemp;
        first = false;
    }
    lastTime = millis();
    bool steamOn = steamButtonState();
    bool pumpOn = brewButtonState();
    float flowRate = (brewActive || (steamButtonState() && timerActive)) ? flowEstVal : (preheatState.getState() == State::PREHEAT_COOL ? 1.5 : 0);
    float waterLevel = waterDistance;

    static double flowSetpoint = 0.0;
    double tempSetpoint = boilerControl.GetSetpoint();

    static unsigned long lastBoilerPidTime = 0;
    double deltaTime = (double)(millis() - lastBoilerPidTime) / 1000.0;
    static double lastTempSetpoint = 0.0;
    if (lastTempSetpoint != tempSetpoint || deltaTime > 0.1) {
        lastTempSetpoint = tempSetpoint;
        lastBoilerPidTime += deltaTime * 1000;

        // handle reading errors by switching the heater off and moving on
        double readTemperature = tempC;

        // instead of using the temperature as read, we model the total energy and use the read value in place of one of the variables
        // this helps greatly because there is about 8s latency between applying a step change to the power and seeing a result

        // How much heat is lost to water flowing out?
        // Assumptions: 1. Water flows out at the same rate as it flows in.
        //              2. Water flowing in is at ambient temperature.
        //              3. Water enthalpies at 100°C are close enough for the other temperatures at which we operate.
        //              4. Density of water flowing in is 1;
        double waterToFlowPower = flowRate * (waterTemp - RESERVOIR_TEMPERATURE) * SPEC_HEAT_WATER_100;

        // How much power is lost to the atmosphere from the brew head?
        double brewHeadToAmbientPower = (brewHeadTemp - ambientTemp) * BREWHEAD_AMBIENT_XFER_COEFF;

        // How much power is transferred from the boiler to the water?
        double shellToWaterPower = (shellTemp - waterTemp) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;
        double elementToWaterPower = (elementTemp - waterTemp) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;

        double shellToRemainingWaterPower = (shellTemp - waterTempWithoutInflow) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;
        double elementToRemainingWaterPower = (elementTemp - waterTempWithoutInflow) * BOILER_WATER_XFER_COEFF_NOFLOW / 2.0;

        static bool steamValveOpen = false;
        if (steamWandOpen || flowRate > 1.0) {
            shellToWaterPower = (shellTemp - waterTemp) * BOILER_WATER_XFER_COEFF_STEAM / 2.0;
            elementToWaterPower = (elementTemp - waterTemp) * BOILER_WATER_XFER_COEFF_STEAM / 2.0;

            shellToRemainingWaterPower = (shellTemp - waterTempWithoutInflow) * BOILER_WATER_XFER_COEFF_STEAM / 2.0;
            elementToRemainingWaterPower = (elementTemp - waterTempWithoutInflow) * BOILER_WATER_XFER_COEFF_STEAM / 2.0;
        }

        double steamHeatLoss = 0.0;
        double steamHeatLossRWater = 0.0;

        // calculate boiling point at pressure, using Clausius–Clapeyron relation
        boilingPoint = (-4890.5 / (-13.103 - log(1.0 / (livePressure + 1.0)))) - 273.15;

        if (digitalRead(solenoidPin) || steamWandOpen) {
            // we lose exactly the right amount of heat through vaporisation that the water temp ends up being 100°C
            // which won't be exactly right - the back pressure through the nozzle will mean it's a little more than 100°C

            steamHeatLoss = (waterTemp - boilingPoint) * (SPEC_HEAT_WATER_100 * BOILER_VOLUME) / deltaTime + shellToWaterPower + elementToWaterPower - waterToFlowPower;
            steamHeatLoss = max(0.0, steamHeatLoss);

            steamHeatLossRWater = (waterTempWithoutInflow - boilingPoint) * (SPEC_HEAT_WATER_100 * BOILER_VOLUME) / deltaTime + shellToRemainingWaterPower + elementToRemainingWaterPower;
            steamHeatLossRWater = max(0.0, steamHeatLossRWater);
        }

        double transferTemp = brewActive && liquidPumped < 100 ? (1.f / (1.f + exp(12.5f - liquidPumped / 5)) * (waterTemp - waterTempWithoutInflow) + waterTempWithoutInflow) : waterTemp;
        double expectedWaterToBrewHeadPower = SPEC_HEAT_WATER_100 * preinfuseFillFlow * (transferTemp - brewHeadTemp);
        dynamicSetpoint = tempSetpoint + (tempSetpoint <= 100 ? expectedWaterToBrewHeadPower / WATER_GROUPHEAD_XFER_COEFF : 0);
        exitTemp = transferTemp - expectedWaterToBrewHeadPower / WATER_GROUPHEAD_XFER_COEFF;

        // How much power is transferred from the boiler to the brew head?
        double shellToBrewHeadPower = (shellTemp - brewHeadTemp) * BOILER_BREWHEAD_XFER_COEFF / 2.0;
        double elementToBrewHeadPower = (elementTemp - brewHeadTemp) * BOILER_BREWHEAD_XFER_COEFF / 2.0;
        double waterFlowToBrewHeadPower = (transferTemp - brewHeadTemp) * flowRate * SPEC_HEAT_WATER_100;
        if (brewHeadTemp < boilingPoint && digitalRead(solenoidPin))
            waterFlowToBrewHeadPower += steamHeatLoss;

        double elementToShellPower = (elementTemp - shellTemp) * ELEMENT_SHELL_XFER_COEFF;

        double shellToBodyPower = (shellTemp - bodyTemp) * BOILER_BODY_XFER_COEFF / 2.0;
        double elementToBodyPower = (elementTemp - bodyTemp) * BOILER_BODY_XFER_COEFF / 2.0;

        // Now work out the temperature, which comes from power that didn't go into heat loss or heating the incoming water.
        brewHeadTemp += deltaTime * (shellToBrewHeadPower + elementToBrewHeadPower + waterFlowToBrewHeadPower - brewHeadToAmbientPower) / (SPEC_HEAT_BRASS * (MASS_BREW_HEAD + MASS_PORTAFILTER));
        waterTemp += deltaTime * (shellToWaterPower + elementToWaterPower - waterToFlowPower - steamHeatLoss) / (SPEC_HEAT_WATER_100 * BOILER_VOLUME);
        waterTempWithoutInflow += deltaTime * (shellToRemainingWaterPower + elementToRemainingWaterPower - steamHeatLossRWater) / (SPEC_HEAT_WATER_100 * BOILER_VOLUME);
        shellTemp += deltaTime * (elementToShellPower - shellToBrewHeadPower - shellToWaterPower - shellToBodyPower) / (SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0);
        elementTemp += deltaTime * (heaterPower - elementToShellPower - elementToBrewHeadPower - elementToWaterPower - elementToBodyPower) / (SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0);
        bodyTemp += deltaTime * (shellToBodyPower + elementToBodyPower) / HEAT_CAPACITY_BODY;
        // every two seconds, work out the average power by which shellTemp diverged from readTemperature
        static double accumulationPeriod = 0.0;
        static double shellTempAccumulatedError = 0.0;
        accumulationPeriod += deltaTime;
        shellTempAccumulatedError += readTemperature - shellTemp;
        shellTemp = readTemperature;

        if (waterTemp <= 95.0) {
            steamValveOpen = false;
            accumulationPeriod = 0.0;
            shellTempAccumulatedError = 0.0;
        } else if (accumulationPeriod >= 2.0) {
            double divergencePower = shellTempAccumulatedError * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0 / accumulationPeriod;
            if (steamValveOpen && divergencePower > 100.0 || !steamValveOpen && divergencePower < -100.0)
                steamValveOpen = !steamValveOpen;
            accumulationPeriod = 0.0;
            shellTempAccumulatedError = 0.0;
        }

        double desiredAverageShellTemp = dynamicSetpoint;
        double desiredWaterInputPower;
        if (tempSetpoint < 100.0) {
            // we want the water to reach target temperature in the next 15s so calculate the necessary average temperature of the shell
            desiredWaterInputPower = (dynamicSetpoint - waterTemp) * SPEC_HEAT_WATER_100 * BOILER_VOLUME / 15.0;
        } else {
            desiredWaterInputPower = steamHeatLoss ? steamHeatLoss : (dynamicSetpoint - waterTemp) * SPEC_HEAT_WATER_100 * BOILER_VOLUME / (livePressure < 1.9 ? 2.0 : 5.0);
        }
        desiredWaterInputPower += waterToFlowPower;
        desiredAverageShellTemp = waterTemp + desiredWaterInputPower / BOILER_WATER_XFER_COEFF_NOFLOW;
        if (flowRate > 0.5)
            desiredAverageShellTemp = waterTemp + desiredWaterInputPower / 10.0;
        // now clip the temperature so that it won't take more than 20s to lose excess heat to ambient
        double maxStableAverageShellTemp = (brewHeadToAmbientPower * 20.0 - (waterTemp - dynamicSetpoint) * SPEC_HEAT_WATER_100 * BOILER_VOLUME) / SPEC_HEAT_ALUMINIUM / MASS_BOILER_SHELL + dynamicSetpoint;
        desiredAverageShellTemp = min(desiredAverageShellTemp, maxStableAverageShellTemp);

        // arrange heater power so that the average boiler energy will be correct in 2 seconds (if possible)
        // the error term handles boiler shell and water - other known power sinks are added explicitly
        double error = (shellTemp - desiredAverageShellTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0;
        error += (elementTemp - desiredAverageShellTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0;
        heaterPower = shellToBrewHeadPower + elementToBrewHeadPower + shellToBodyPower + elementToBodyPower + shellToWaterPower + elementToWaterPower - error / 2.0;

        // keep power level safe and sane (where it would take two seconds to get triac or elements over max temp and five seconds to get shell over max temp)
        double maxAllowableElementToShellPower = (SHELL_MAX_TEMPERATURE - shellTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 5.0 + shellToWaterPower + shellToBrewHeadPower + shellToBodyPower;
        double maxAllowableElementTemp = maxAllowableElementToShellPower / ELEMENT_SHELL_XFER_COEFF + shellTemp;
        maxAllowableElementTemp = min(maxAllowableElementTemp, ELEMENT_MAX_TEMPERATURE);
        double maxAllowablePowerForElement = (maxAllowableElementTemp - elementTemp) * SPEC_HEAT_ALUMINIUM * MASS_BOILER_SHELL / 2.0 + elementToShellPower + elementToWaterPower + elementToBrewHeadPower + elementToBodyPower;
        double wantedHeaterPower = max(0.0, min(maxAllowablePowerForElement, min(SAFE_HEATER_POWER, heaterPower)));

        if (!powerButtonState()) {
            heaterPower = 0;
        } else {
            heaterPower = wantedHeaterPower;
        }

        setHeaterPower(heaterPower);

        static int timeCount = 0;
        if (timeCount == 0) {
            static int rowCount = 0;
            if (rowCount == 0) {
                Serial.println(F("time\tsteam \tpump  \theat \tw.heat \telemnt\tshell \twater\tb.head\tboil\tpump \twater\tvolume\tflow \tflow\tpulse"));
                Serial.println(F("    \tswitch\tswitch\tpower\tpower\ttemp\ttemp\ttemp \ttemp\tpoint\tpower\tlevel\t      \tsetpt\trate\tcount"));
            }
            rowCount = (rowCount + 1) % 30;

            // print the data to any computer that happens to be listening
            Serial.print((double)micros() / 1000000.0);
            Serial.print("\t");
            Serial.print(steamOn ? "on" : "off");
            Serial.print("\t");
            Serial.print(pumpOn ? "on" : "off");
            Serial.print("\t");
            Serial.print(heaterPower);
            Serial.print("\t");
            Serial.print(wantedHeaterPower);
            Serial.print("\t");
            Serial.print(elementTemp);
            Serial.print("\t");
            Serial.print(shellTemp);
            Serial.print("\t");
            Serial.print(waterTemp);
            Serial.print("\t");
            Serial.print(brewHeadTemp);
            Serial.print("\t");
            Serial.print(boilingPoint);
            Serial.print("\t");
            Serial.print(String(dbgPressure, 4));
            Serial.print("\t");
            Serial.print(waterTempWithoutInflow);
            Serial.print("\t");
            Serial.print(liquidPumped);
            Serial.print("\t");
            Serial.print(steamHeatLoss);
            Serial.print("\t");
            Serial.print(dynamicSetpoint);
            Serial.print("\t");
            Serial.print(pumpStrokeCount);
            Serial.println();
        }
        timeCount = (timeCount + 1) % 10;
    }

    return mapRange(heaterPower, 0, 1230, 0, 1000, 1);
}
// Actuating the heater element
void setBoilerOn(void) {
  digitalWrite(relayPin, HIGH);  // boilerPin -> HIGH
}

void setBoilerOff(void) {
  digitalWrite(relayPin, LOW);  // boilerPin -> LOW
}