#ifndef SILVIA_CLEAN_H
#define SILVIA_CLEAN_H

#define UPDATE_INTERVAL 100 // ms
#define PUB_CLEANER_INTERVAL 500  // ms

#include <Arduino.h>
#include <ros.h>
#include <django_interface/SilviaSettings.h>
#include <django_interface/SilviaCleaner.h>
#include "silvia_output.h"
#include "silvia_sensors.h"
#include "silvia_ros.h"
#include "silvia_controllers.h"

class CleaningProcess : public SilviaPublisher {
    private:
        bool active_;  // Cleaner active?
        int n_cycles_;  // Number of pump on/off cycles
        unsigned long t_on_;  // Time pump is on for each cycle
        unsigned long t_off_;  // Time pump is off for each cycle
        unsigned long t_total_;  // Total cleaning time
        unsigned char previous_mode_;
        unsigned long duration_;  // Duration of cleaning so far
        unsigned long t_start_;
        django_interface::SilviaCleaner msg_;
        void pumpOn();
        void pumpOff();
        void populateMessage(); 

    public:
        CleaningProcess();
        void setup(NodeHandle* nh);
        void start(int previous_mode);
        void stop();
        int update();
        int getTimeRemaining();
        int getTimeRemainingMins();
        int getTimeRemainingSecs();
        void setTimings(int n_cycles, int t_on, int t_off);
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
};

extern RelayOutput brew_output;
extern PressureController pump;
extern WaterLevelSensor water_sensor;

#endif // SILVIA_CLEAN_H
