#ifndef SILVIA_CLEAN_H
#define SILVIA_CLEAN_H

#define UPDATE_INTERVAL 100 //ms

#include <Arduino.h>
#include <django_interface/SilviaSettings.h>
#include "silvia_io.h"
#include "silvia_sensors.h"
#include "silvia_timer.h"

class CleaningProcess {
    private:
        int n_cycles_;
        int t_on_;
        int t_off_;
        int t_total_;
        unsigned char previous_mode_;
        unsigned long duration_;
        unsigned long last_update_;
        ros::Subscriber<django_interface::SilviaSettings, CleaningProcess> settings_subscriber_;
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
        void pumpOn();
        void pumpOff();      

    public:
        CleaningProcess();
        void start(unsigned char previous_mode);
        void stop();
        unsigned char update();
        int getTimeRemaining();
        void setTimings(int n_cycles, int t_on, int t_off);
};

extern RelayOutput brew_output;
extern WaterLevelSensor water_sensor;

#endif // SILVIA_CLEAN_H