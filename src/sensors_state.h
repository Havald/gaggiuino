#ifndef SENSORS_STATE_H
#define SENSORS_STATE_H

struct SensorState {
  float temperature;
  float pressure;
  float pressureDelta;
  float pumpFlow;
  float weightFlow;
  float liquidPumped;
  float weight;
  float flowDelta;
  bool isOutputFlow;
};

#endif
