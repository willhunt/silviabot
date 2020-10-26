/*
Author: Willaim Hunt
Date: October 2020
Project: Silvia ROS

Description: Arduino files for Rancillio Silvia espresso machine control
Notes: 
*/

// Pre-compile definitions
#define BAUDRATE 115200
#define TEMP_SENSOR_PIN A0
#define WATER_SENSOR_PIN 7
#define HEAT_RELAY_PIN 13
#define POWER_RELAY_PIN 9
#define BREW_RELAY_PIN 12
#define POWER_SWITCH_PIN 4
#define BREW_SWITCH_PIN 3
#define DIMMER_PIN 5
#define I2C_ADDR 0x8
#define SAFETY_TEMPERATURE 120

// Imports
#include <ros.h>
#include "silvia_sensors.h"
#include "silvia_timer.h"
#include "silvia_modes.h"
#include "silvia_io.h"
#include "silvia_controllers.h"

// ROS
ros::NodeHandle nh;

// Mode
unsigned char mode = MODE_OFF;

// Relays
RelayOutput power_output = RelayOutput(POWER_RELAY_PIN);
RelayOutput brew_output = RelayOutput(BREW_RELAY_PIN);

// Sensors
TemperatureSensor temperature_sensor(TEMP_SENSOR_PIN);
// PressureSensor pressure_sensor(PRESSURE_SENSOR_PIN);
WaterLevelSensor water_sensor(WATER_SENSOR_PIN);
BrewTimer brew_timer = BrewTimer();

// Controllers
TemperatureController heater(&temperature_sensor.reading, HEAT_RELAY_PIN);
// PressureController pump(&pressure_sensor.reading, DIMMER_PIN);

// Switches
// SwitchInput power_switch = SwitchInput(POWER_SWITCH_PIN, &powerOnSwitch, &powerOffSwitch);
// SwitchInput brew_switch = SwitchInput(BREW_SWITCH_PIN, &brewOnSwitch, &brewOffSwitch);

void setup(void) {
    nh.getHardware()->setBaud(BAUDRATE);
    nh.initNode();
    temperature_sensor.setup(&nh);
    water_sensor.setup(&nh);
    brew_timer.setup(&nh);
}

void loop(void)  {
    temperature_sensor.update();  // Method includes sampling time check
    temperature_sensor.publish();
    brew_timer.update(brew_output);
    brew_timer.publish();
    water_sensor.lowWater();
    water_sensor.publish();
    nh.spinOnce();
}
