#ifndef SILVIA_DISPLAY_H
#define SILVIA_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <ros.h>
#include <django_interface/SilviaController.h>
#include <django_interface/SilviaStatus.h>
#include <django_interface/SilviaBrewTimer.h>
#include <django_interface/SilviaCleaner.h>
#include <django_interface/SilviaMass.h>
#include "silvia_logo.h"
#include "silvia_state.h"
#include "silvia_sensors.h"
#include "silvia_controllers.h"
#include "silvia_clean.h"
#include "silvia_timer.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_PADDING 4 // Edge padding
#define UPDATE_INTERVAL 1000  // Update/refresh interval
#define WELCOME_INTERVAL 2000  // Display welcome
#define ACCEPTABLE_DATA_AGE 2000  // Interval over which to display "old" data

extern SilviaStatus silvia_status;
extern TemperatureSensor temperature_sensor;
extern PressureSensor pressure_sensor;
extern TemperatureController heater;
extern PressureController pump;
extern PressureSensor pressure_sensor;
extern BrewTimer brew_timer;

class SilviaDisplay : public Adafruit_SSD1306 {
    private:
        NodeHandle* nh_;
        ros::Subscriber<django_interface::SilviaMass, SilviaDisplay> mass_subscriber_;
        unsigned long t_power_on_;  // Time machine was last turned on
        bool power_status_last_update_;  // Power status recorded last update
        unsigned long t_last_update_;
        double mass_;
        unsigned long t_mass_update_;  // Mass update is coming externally so need to flag if data is old

        void drawCentreString(const char *buffer, int x, int y);
        void formatPressureString(char* buffer, double pressure);
        void drawPressureString(char* buffer, double pressure, int x, int y, int size);
        void massCallback(const django_interface::SilviaMass& msg);

    public:
        SilviaDisplay(TwoWire* wire);
        void setup(NodeHandle* nh);    
        void showData();
        void showLogo();
        void showBlank();
        void update();
        void showDebug(char* text);
};

#endif // SILVIA_DISPLAY_H
