#ifndef SILVIA_TEMPERATURE_CONTROLLER_H
#define SILVIA_TEMPERATURE_CONTROLLER_H

#define PROFILE_STEPS 5

#include <Arduino.h>
#include <RBDdimmer.h>
#include <django_interface/SilviaSettings.h>
#include <std_msgs/Float64.h>
#include "silvia_pid.h"

/* 
PID controller for boiler temperature
*/
class TemperatureController : public PidController {
    private:
        int relay_pin_;
        unsigned long tpc_window_start_;  // Time proportional control start time [millis]
        unsigned long tpc_window_size_;  // Time proportional control window size [millis]
        ros::Subscriber<django_interface::SilviaSettings, TemperatureController> settings_subscriber_;
        ros::Subscriber<std_msgs::Float64, TemperatureController> override_subscriber_;
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
        void overrideCallback(const std_msgs::Float64& msg);
        bool temporary_override_;

    public:
        TemperatureController(double* input, int relay_pin);
        void controlOutput();
        int getRelayPin();
        bool getTemporaryOverride();
        void temporaryOverride(bool override, double output);
};

/* 
PID controller for pump pressure
*/
class PressureController : public PidController {
    private:
        int dimmer_pin_;
        dimmerLamp dimmer_;
        unsigned long t_profile_[PROFILE_STEPS];
        double pressure_profile_[PROFILE_STEPS];
        ros::Subscriber<django_interface::SilviaSettings, PressureController> settings_subscriber_;
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
        void setProfile(unsigned long t_profile_[], double pressure_profile_[]);

    public:
        PressureController(double* input, int relay_pin);
        void controlOutput();
        int getDimmerPin();
        double getProfilePressure(unsigned long t);
        void deactivate();
        void compute(unsigned long t);
};

#endif // SILVIA_TEMPERATURE_CONTROLLER_H
