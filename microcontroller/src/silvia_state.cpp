#include "silvia_state.h"
#include <ros.h>
#include <std_msgs/Float64.h>
#include "django_interface/SilviaStatus.h"

SilviaStatus::SilviaStatus() 
    : SilviaPublisher("status", &status_report_msg_, PUB_STATUS_INTERVAL)
    , status_subscriber_("status_change", &SilviaStatus::statusChangeCallback, this)
    // , status_change_server_("status_change_srv", &SilviaStatus::statusChangeSrvCallback, this)
    , settings_subscriber_("settings", &SilviaStatus::setSettingsCallback, this)
    , settings_request_publisher_("settings_request", &settings_request_msg_)
    , mode_(MODE_OFF)
    , settings_updated_(false)
    , last_brew_status_(false)
{
    status_report_msg_.header.frame_id = "micro";
}

void SilviaStatus::setup(NodeHandle* nh) {
    nh->subscribe(status_subscriber_);
    nh->subscribe(settings_subscriber_);
    // nh->advertiseService(status_change_server_);
    nh->advertise(settings_request_publisher_);
    SilviaPublisher::setup(nh);
    update();
}

void SilviaStatus::update() {
    if (!settings_updated_) {
        settings_request_publisher_.publish(&settings_request_msg_);
    }
    // Brew might be changed by cleaner. Check to see if state needs to be published.
    if (last_brew_status_ != getBrew()) {
        publish(true);
    }
    last_brew_status_ = getBrew();
}

void SilviaStatus::populateMessage() {
    status_report_msg_.header.stamp = nh_->now();
    status_report_msg_.mode = mode_;
    status_report_msg_.brew = getBrew();
}

void SilviaStatus::statusChangeCallback(const django_interface::SilviaStatus& msg) {
    update();
    if (msg.mode == MODE_OFF) changeBrew(false);  // Deal with brew first if machine is being turned off
    if (msg.mode != MODE_IGNORE) changeMode(msg.mode);
    if (msg.mode != MODE_OFF) changeBrew(msg.brew);
    publish(true);  // Force publish
}

// void SilviaStatus::statusChangeSrvCallback(const StatusChangeRequest& request, StatusChangeResponse& response) {
//     statusChangeCallback(request.status);
//     response.status.mode = mode_;
//     response.status.brew = getBrew();
// }

void SilviaStatus::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    heater.setSettingsCallback(msg);
    cleaner.setSettingsCallback(msg);
    pump.setSettingsCallback(msg);
    settings_updated_ = true;
}

void SilviaStatus::changeMode(int new_mode) {
    // Check if changing out of a mode (turn things off)
    if (new_mode != MODE_PID && mode_ == MODE_PID) {
        heater.deactivate();
        heater.setDuty(0);
    } else if (new_mode != MODE_MANUAL && mode_ == MODE_MANUAL) {
        heater.setOutput(0);
    } else if (new_mode != MODE_CLEAN && mode_ == MODE_CLEAN) {
        cleaner.stop();
    }
    // Check if changing into a mode
    if (new_mode == MODE_OFF && mode_ != MODE_OFF) {  // Turning off
        power_output.off();
    } else {
        if (mode_ == MODE_OFF && new_mode != MODE_OFF) {  // Turning on
            power_output.on();
            brew_timer.reset();
        }
        if (new_mode == MODE_PID && mode_ != MODE_PID) { // Change to PID
            heater.reset();
            heater.activate();
        } else if (new_mode == MODE_MANUAL && mode_ != MODE_MANUAL) { // Change to manual
            heater.setOutput(0);
        } else if (new_mode == MODE_CLEAN && mode_ != MODE_CLEAN) {  // CHange to cleaning
            cleaner.start(mode_);
        }
    }
    mode_ = new_mode;
}

void SilviaStatus::changeBrew(bool brew) {
    if (brew) {
        // Not in off mode as well as in manual mode or water in tank
        if ( mode_ != MODE_OFF && ( (mode_ == MODE_MANUAL) || !water_sensor.lowWater() ) ) {  
            brew_output.on();
            brew_timer.reset();
            brew_timer.start();
            pump.enableOutput();
            if (mode_ == MODE_PID) {
                pump.activate();
                // Change to manual mode to max power and no integral windup
                changeMode(MODE_MANUAL);
                heater.temporaryOverride(true, 100);
            }
        }
    } else {
        brew_output.off();
        pump.disableOutput();
        pump.deactivate();
        if (heater.getTemporaryOverride()) {
            changeMode(MODE_PID);
            heater.temporaryOverride(false, 0);
        }
    }
}

bool SilviaStatus::getBrew() {
    return brew_output.getStatus();
}
