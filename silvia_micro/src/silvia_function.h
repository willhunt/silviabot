#ifndef SILVIA_FUNCTION_H
#define SILVIA_FUNCTION_H

#include <Arduino.h>
#include "silvia_io.h"
#include "silvia__controllers.h"
#include "silvia_sensors.h"
#include "silvia_timer.h"

void statusRequestCallback();
void changeMode(unsigned char new_mode);
void changeBrew(bool brew)

// Defined in silvia_main.ino
extern unsigned char mode;
extern RelayOutput power_output;
extern RelayOutput brew_output;
extern WaterLevelSensor water_sensor;
extern TemperatureController heater;
extern BrewTimer brew_timer;

#endif  // SILVIA_FUNCTION_H