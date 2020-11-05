/*
Author: Willaim Hunt
Date: October 2020
Project: Silvia ROS

Description: Arduino files for Rancilio Silvia espresso machine control
Notes: 
*/

// Pre-compile definitions
#define BAUDRATE 115200
#define TEMP_SENSOR_PIN A0
#define PRESSURE_SENSOR_PIN A1
#define WATER_SENSOR_PIN A2
#define HEAT_RELAY_PIN 13
#define POWER_RELAY_PIN 9
#define BREW_RELAY_PIN 12
#define POWER_SWITCH_PIN 4
#define BREW_SWITCH_PIN 3
#define DIMMER_PIN 5
#define I2C_ADDR 0x8
#define SAFETY_TEMPERATURE 120
#define SAFETY_PRESSURE 11e5

// Imports  
#include <ros.h>
#include <Wire.h>
#include "silvia_sensors.h"
#include "silvia_input.h"
#include "silvia_output.h"
#include "silvia_controllers.h"
#include "silvia_timer.h"
#include "silvia_state.h"
#include "silvia_clean.h"
#include "silvia_modes.h"
#include "silvia_display.h"
#include "silvia_ros.h"

// ROS
NodeHandle nh;

// State
SilviaStatus silvia_status = SilviaStatus();
CleaningProcess cleaner = CleaningProcess();

// Relays
RelayOutput power_output = RelayOutput(POWER_RELAY_PIN);
RelayOutput brew_output = RelayOutput(BREW_RELAY_PIN);

// Sensors
TemperatureSensor temperature_sensor(TEMP_SENSOR_PIN);
PressureSensor pressure_sensor(PRESSURE_SENSOR_PIN);
WaterLevelSensor water_sensor(WATER_SENSOR_PIN);
BrewTimer brew_timer = BrewTimer();

// Controllers
TemperatureController heater(&temperature_sensor.reading, HEAT_RELAY_PIN);
PressureController pump(&pressure_sensor.reading, DIMMER_PIN);

// Switches
PowerSwitch power_switch = PowerSwitch(POWER_SWITCH_PIN);
BrewSwitch brew_switch = BrewSwitch(BREW_SWITCH_PIN);

// Display
SilviaDisplay display = SilviaDisplay(&Wire);

void setup(void) {
    nh.getHardware()->setBaud(BAUDRATE);
    nh.initNode();

    temperature_sensor.setup(&nh);
    pressure_sensor.setup(&nh);
    water_sensor.setup(&nh);
    brew_timer.setup(&nh);
    heater.setup(&nh);
    heater.setSafetyLimit(0, SAFETY_TEMPERATURE);
    pump.setup(&nh);
    pump.setSafetyLimit(0, SAFETY_PRESSURE);
    cleaner.setup(&nh);
    silvia_status.setup(&nh);

    // Display startup
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        // nh.logerror("Display not initialised");
    }
    display.showBlank();
}

void loop(void)  {
    power_switch.update();
    brew_switch.update();

    temperature_sensor.update();
    temperature_sensor.publish();

    pressure_sensor.update();
    pressure_sensor.publish();

    unsigned long brew_time = brew_timer.update(silvia_status.getBrew());
    brew_timer.publish();

    water_sensor.lowWater();
    water_sensor.publish();

    heater.compute();
    heater.controlOutput();
    heater.publish();

    pump.compute(brew_time);
    pump.controlOutput();
    pump.publish();  // Causing issue...........

    int cleaner_mode = cleaner.update();
    // Cleaner returns previous mode if finished
    if (cleaner_mode != MODE_CLEAN && cleaner_mode != MODE_IGNORE) {
        silvia_status.changeMode(cleaner_mode);
    }
    cleaner.publish();

    silvia_status.publish();

    display.update();
    
    nh.spinOnce();
}
