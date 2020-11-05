#include "silvia_state.h"
#include <ros.h>
#include <std_msgs/Float64.h>
#include "django_interface/SilviaStatus.h"

SilviaStatus::SilviaStatus() 
    : SilviaPublisher("status", &status_report_msg_, PUB_STATUS_INTERVAL)
    , status_subscriber_("status_request", &SilviaStatus::statusRequestCallback, this)
    , settings_subscriber_("settings", &SilviaStatus::setSettingsCallback, this)
    , mode_(MODE_OFF)
{
    status_report_msg_.header.frame_id = "micro";
}

void SilviaStatus::setup(NodeHandle* nh) {
    nh->subscribe(status_subscriber_);
    nh->subscribe(settings_subscriber_);
    SilviaPublisher::setup(nh);
}

void SilviaStatus::populateMessage() {
    status_report_msg_.header.stamp = nh_->now();
    status_report_msg_.mode = mode_;
    status_report_msg_.brew = getBrew();
}

void SilviaStatus::statusRequestCallback(const django_interface::SilviaStatus& msg) {
    if (msg.mode == MODE_OFF) changeBrew(false);  // Deal with brew first if machine is being turned off
    if (msg.mode != MODE_IGNORE) changeMode(msg.mode);
    if (msg.mode != MODE_OFF) changeBrew(msg.brew);
    publish(true);  // Force publish
}

void SilviaStatus::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    cleaner.setSettingsCallback(msg);
    heater.setSettingsCallback(msg);
    pump.setSettingsCallback(msg);
}

void SilviaStatus::changeMode(int new_mode) {
    // Check if changing out of a mode (turn things off)
    if (new_mode != MODE_PID && mode_ == MODE_PID) {
        heater.deactivate();
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
        } else if (new_mode == MODE_CLEAN && mode_ != MODE_CLEAN) {
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
            if (mode_ == MODE_PID) {
                // Change to manual mode to max power and no integral windup
                changeMode(MODE_MANUAL);
                heater.temporaryOverride(true, 100);
            }
        }
    } else {
        brew_output.off();
        if (heater.getTemporaryOverride()) {
            changeMode(MODE_PID);
            heater.temporaryOverride(false, 0);
        }
    }
}

bool SilviaStatus::getBrew() {
    return brew_output.getStatus();
}