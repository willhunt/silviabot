#ifndef SILVIA_CONTROLLERS_H
#define SILVIA_CONTROLLERS_H

#define PROFILE_STEPS 5
#define PUB_CONTROLLER_INTERVAL 200

#include <Arduino.h>
#include <ros.h>
#include <RBDdimmer.h>
#include <django_interface/SilviaSettings.h>
#include <django_interface/SilviaController.h>
#include <std_msgs/Float64.h>
#include "silvia_pid.h"
#include "silvia_ros.h"

/* 
PID controller for boiler temperature
*/
class TemperatureController : public PidController, public  SilviaPublisher {
    private:
        int relay_pin_;
        unsigned long tpc_window_start_;  // Time proportional control start time [millis]
        unsigned long tpc_window_size_;  // Time proportional control window size [millis]
        bool temporary_override_;
        django_interface::SilviaController msg_;
        ros::Subscriber<std_msgs::Float64, TemperatureController> override_subscriber_;
        void overrideCallback(const std_msgs::Float64& msg);
        void populateMessage();

    public:
        TemperatureController(double* input, int relay_pin);
        void setup(NodeHandle* nh);
        void controlOutput();
        int getRelayPin();
        bool getTemporaryOverride();
        void temporaryOverride(bool override, double output);
        double getDuty();  // Get duty 0-100
        void setDuty(double duty);  // Set duty 0-100
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
};

/* 
PID controller for pump pressure
*/
class PressureController : public PidController, public  SilviaPublisher {
    private:
        // int dimmer_pin_;
        // dimmerLamp dimmer_;
        dimmerLamp* dimmer_;
        unsigned long t_profile_[PROFILE_STEPS];
        double pressure_profile_[PROFILE_STEPS];
        django_interface::SilviaController msg_;
        ros::Subscriber<std_msgs::Float64, PressureController> override_subscriber_;
        void overrideCallback(const std_msgs::Float64& msg);
        void setProfile(unsigned long t_profile_[], double pressure_profile_[]);
        void populateMessage();

    public:
        // PressureController(double* input, int relay_pin);
        PressureController(double* input, dimmerLamp* dimmer);
        void setup(NodeHandle* nh);
        void controlOutput();
        int getDimmerPin();
        double getProfilePressure(unsigned long t);
        void enableOutput();
        void disableOutput();
        // void deactivate();
        void compute(unsigned long t);
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
};

#endif // SILVIA_CONTROLLERS_H
