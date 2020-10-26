#include "silvia_controllers.h"

// TEMPERATURE CONTROLLER ----------------------------------------------------
TemperatureController::TemperatureController(double* input, int relay_pin)
    : PidController(input)
    , settings_subscriber_("settings", &TemperatureController::setSettingsCallback, this)
    , relay_pin_(relay_pin)
    , tpc_window_start_(millis())
    , tpc_window_size_(1000)
{
    setSetpoint(98);
    pinMode(relay_pin_, OUTPUT);
    setOutputLimits(0, tpc_window_size_);
    setUpdateInterval(500);  // Increase sample time to 500ms rather than 200 (default)
}

void TemperatureController::controlOutput() {
    unsigned long t_now = millis();
    // Time to shift the Relay Window
    if (t_now - tpc_window_start_ > tpc_window_size_) {
        // Increment in step of window size
        tpc_window_start_ += tpc_window_size_;
    }
    if (output_ >= t_now - tpc_window_start_) {
        digitalWrite(relay_pin_, HIGH);
    } else {
        digitalWrite(relay_pin_, LOW);
    }
}

int TemperatureController::getRelayPin() {
    return relay_pin_;
}

bool TemperatureController::getTemporaryOverride() {
    return temporary_override_;
}
void TemperatureController::temporaryOverride(bool override, double output) {
    if (override) {
        output_ = output;
        deactivate();
    } else if (temporary_override_) {
        reset();
        activate();
    }
    temporary_override_ = override;
}

void TemperatureController::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    setSetpoint(msg.temperature_setpoint);
    setGains(msg.heater_kp, msg.heater_ki, msg.heater_kd);
}

void TemperatureController::overrideCallback(const std_msgs::Float64& msg) {
    setOutput((double)msg.data);
}

// PRESSURE CONTROLLER ----------------------------------------------------
PressureController::PressureController(double* input, int dimmer_pin)
    : PidController(input)
    , settings_subscriber_("settings", &PressureController::setSettingsCallback, this)
    , dimmer_pin_(dimmer_pin)
    , dimmer_(dimmer_pin)
{
    setSetpoint(9);
    pinMode(dimmer_pin_, OUTPUT);
    setOutputLimits(0, 100);
    setUpdateInterval(200);  // Increase sample time to 500ms rather than 200 (default)
}

void PressureController::controlOutput() {
    dimmer_.setPower(output_);
}

int PressureController::getDimmerPin() {
    return dimmer_pin_;
}

void PressureController::deactivate() {
    PidController::deactivate();
    dimmer_.setState(OFF);
}

/*
    Get pressure at time t based upon given profile
    Assumes linear transitions
*/
double PressureController::getProfilePressure(unsigned long t) {
    for (int i = 1; i < PROFILE_STEPS; i++) {
        if (t >= (float)t_profile_[i-1] && t < (float)t_profile_[i]) {
            return pressure_profile_[i] + (pressure_profile_[i] - pressure_profile_[i-1]) / 
                (double)(t_profile_[i] - t_profile_[i-1]) * (t - (double)t_profile_[i]);
        }
    }
    if (t > t_profile_[PROFILE_STEPS]) return pressure_profile_[PROFILE_STEPS];
    else return 0.0;
}

void PressureController::compute(unsigned long t) {
    setpoint_ = getProfilePressure(t);
    PidController::compute();
}

void PressureController::setProfile(unsigned long t_profile[], double pressure_profile[]) {
    int profile_length = sizeof(t_profile) / sizeof(t_profile[0]);
    // Dont' do anything if inputs are different lengths or less than 1 (no entries)
    if (profile_length != sizeof(pressure_profile) / sizeof(pressure_profile[0])
        || profile_length < 1) return;

    for (int i = 0; i < PROFILE_STEPS; i++) {
        if (i < profile_length) {  // Don't overrun allocated memory
            t_profile_[i] = t_profile[i];
            pressure_profile_[i] = pressure_profile[i];
        } else {  // Fill trailing values with previous
            t_profile_[i] = t_profile_[i-1];
            pressure_profile_[i] = pressure_profile_[i-1];
        }
    }
}

void PressureController::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    // Dont' do anything if inputs are different lengths or less than 1 (no entries)
    if (msg.pressure_t_setpoints_length != msg.pressure_setpoints_length
        || msg.pressure_setpoints_length < 1) return;

    int length = (msg.pressure_setpoints_length < PROFILE_STEPS) ? msg.pressure_setpoints_length : PROFILE_STEPS;
    unsigned long t_setpoints_millis[length];
    double pressure_setpoints[length];
    for (int i = 0; i < length; i++) {
        t_setpoints_millis[i] = (unsigned long)(msg.pressure_t_setpoints[i] * 1000);
        pressure_setpoints[i] = (double)(msg.pressure_setpoints[i]);
    }
    setProfile(t_setpoints_millis, pressure_setpoints);
    setGains(msg.pump_kp, msg.pump_ki, msg.pump_kd);
}