#ifndef SILVIA_FUNCTION_H
#define SILVIA_FUNCTION_H

#define PUB_STATUS_INTERVAL 200

#include <Arduino.h>
#include <django_interface/SilviaStatus.h>
#include <django_interface/SilviaStatusChange.h>
#include "silvia_modes.h"
#include "silvia_output.h"
#include "silvia_controllers.h"
#include "silvia_sensors.h"
#include "silvia_timer.h"
#include "silvia_ros.h"
#include "silvia_clean.h"

class SilviaStatus : public SilviaPublisher {
    typedef django_interface::SilviaStatusChange::Request StatusChangeRequest;
    typedef django_interface::SilviaStatusChange::Response StatusChangeResponse;

    private:
        void populateMessage();
        django_interface::SilviaStatus status_report_msg_;
        ros::Subscriber<django_interface::SilviaStatus, SilviaStatus> status_subscriber_;
        ros::ServiceServer<StatusChangeRequest, StatusChangeResponse, SilviaStatus> status_change_server_;
        ros::Subscriber<django_interface::SilviaSettings, SilviaStatus> settings_subscriber_;
        void statusChangeCallback(const django_interface::SilviaStatus& msg);
        void statusChangeSrvCallback(const StatusChangeRequest& request, StatusChangeResponse& response);
        void setSettingsCallback(const django_interface::SilviaSettings& msg);
        int mode_;

    public:
        SilviaStatus();
        void setup(NodeHandle* nh);
        void changeMode(int new_mode);
        int getMode() {return mode_;};
        void changeBrew(bool brew);
        bool getBrew();
};


extern RelayOutput power_output;
extern RelayOutput brew_output;
extern WaterLevelSensor water_sensor;
extern TemperatureController heater;
extern PressureController pump;
extern BrewTimer brew_timer;
extern CleaningProcess cleaner;

#endif  // SILVIA_FUNCTION_H
