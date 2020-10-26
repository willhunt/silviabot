#include "silvia_function.h"
#include "silvia_modes.h"
#include "django_interface/SilviaStatus.h"

status_publisher = ros::Publisher("status");
status_subscriber = ros::Subscriber<django_interface::SilviaStatus>("status_request", statusRequestCallback);
heater_override_subscriber = ros::Subscriber<ros:Float>("heater_override", heaterOverrideCallback);

void statusRequestCallback(django_interface::SilviaStatus& msg) {
    changeMode(msg.mode);
    changeBrew(msg.brew);
}

void changeMode(unsigned char new_mode) {
    if (new_mode == MODE_OFF) {
        if (mode = MODE_PID) heater.deactivate();
        if (mode = MODE_MANUAL) heater.setOutput(0);
        // Turning off
        if (mode != MODE_OFF) power_output.off();
        mode = MODE_OFF;
    } else if (mode == MODE_OFF) {  // Turning on
        power_output.on();
        brew_timer.reset();
    } else if (new_mode == MODE_PID) { // Change to PID or PID settings
        // Reset PID if previously in a different mode
        if (mode != MODE_PID) heater.reset();  
        mode = MODE_PID;
        heater.activate();
    } else if (new_mode == MODE_MANUAL && mode != MODE_MANUAL) { // Change to manual
        if (mode = MODE_PID) heater.deactivate();
        mode = MODE_MANUAL;
        heater.setOutput(0);
    } 
}

void changeBrew(bool brew) {
    if (brew) {
        // Not in off mode as well as in manual mode or water in tank
        if ( mode != MODE_OFF && ( (mode == MODE_MANUAL) || water_sensor.getLevel() ) ) {  
            brew_output.on();
            brew_timer.reset();
            brew_timer.start();
            if (mode == MODE_PID) {
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
