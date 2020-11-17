#include "silvia_controllers.h"

// TEMPERATURE CONTROLLER ----------------------------------------------------
TemperatureController::TemperatureController(double* input, int relay_pin)
    : PidController(input)
    , SilviaPublisher("heater_controller", &msg_, PUB_CONTROLLER_INTERVAL)
    , override_subscriber_("heater_duty", &TemperatureController::overrideCallback, this)
    , relay_pin_(relay_pin)
    , tpc_window_start_(millis())
    , tpc_window_size_(1000)
{
    setSetpoint(98);
    setGains(40.0, 0.02, 500.0);
    pinMode(relay_pin_, OUTPUT);
    setOutputLimits(0, tpc_window_size_);
    setUpdateInterval(500);  // Increase sample time to 500ms rather than 200 (default)
    msg_.header.frame_id = "micro";
}

void TemperatureController::setup(NodeHandle* nh) {
    nh->subscribe(override_subscriber_);
    SilviaPublisher::setup(nh);
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

double TemperatureController::getDuty() {
    return 100 * output_ / tpc_window_size_;
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

void TemperatureController::setDuty(double duty) {
    if (duty < 0 or duty > 100.0) return;
    setOutput(tpc_window_size_ * duty / 100);
}

void TemperatureController::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    setSetpoint(msg.temperature_setpoint);
    setGains(msg.heater_kp, msg.heater_ki, msg.heater_kd);
}

void TemperatureController::overrideCallback(const std_msgs::Float64& msg) {
    setDuty((double)msg.data);
}

void TemperatureController::populateMessage() {
    msg_.header.stamp = nh_->now();
    msg_.input = *input_;
    msg_.setpoint = setpoint_;
    msg_.output = getDuty();
    msg_.output_p = output_p_;
    msg_.output_i = output_i_;
    msg_.output_d = output_d_;
    msg_.kd = kd_;
    msg_.ki = ki_;
    msg_.kd = kd_;
    msg_.active = active_;
}

// PRESSURE CONTROLLER ----------------------------------------------------
// PressureController::PressureController(double* input, int dimmer_pin)
PressureController::PressureController(double* input, dimmerLamp* dimmer)
    : PidController(input)
    , SilviaPublisher("pump_controller", &msg_, PUB_CONTROLLER_INTERVAL)
    , override_subscriber_("pump_duty", &PressureController::overrideCallback, this)
    // , dimmer_pin_(dimmer_pin)
    // , dimmer_(dimmer_pin)
{
    setSetpoint(9e5);
    // pinMode(dimmer_pin_, OUTPUT);
    setOutputLimits(0, 98);
    setUpdateInterval(200);
    msg_.header.frame_id = "micro";
     // Fill default profile values
    for (int i = 0; i < PROFILE_STEPS; i++) {
        t_profile_[i] = i * 5000;
        pressure_profile_[i] = 9e5;
    }
    dimmer_ = dimmer;
}

void PressureController::setup(NodeHandle* nh) {
    nh->subscribe(override_subscriber_);
    SilviaPublisher::setup(nh);
    // dimmer_.begin(NORMAL_MODE, ON);
}

void PressureController::controlOutput() {
    // dimmer_.setPower(output_);
    if (output_ <= 0) {
        dimmer_->setState(OFF);
    } else {
        if (dimmer_->getState() == OFF) dimmer_->setState(ON);
        dimmer_->setPower(output_);
    }
}

// int PressureController::getDimmerPin() {
//     return dimmer_pin_;
// }

void PressureController::enableOutput() {
    // dimmer_.setState(ON);
}
void PressureController::disableOutput() {
    // dimmer_.setState(OFF);
}

// void PressureController::deactivate() {
//     PidController::deactivate();
// }

/*
    Get pressure at time t based upon given profile
    Assumes linear transitions
*/
double PressureController::getProfilePressure(unsigned long t) {
    if (!active_) return 0.0;
    for (int i = 1; i < PROFILE_STEPS; i++) {
        if (t >= (float)t_profile_[i-1] && t < (float)t_profile_[i]) {
            return pressure_profile_[i] + (pressure_profile_[i] - pressure_profile_[i-1]) / 
                (double)(t_profile_[i] - t_profile_[i-1]) * (t - (double)t_profile_[i]);
        }
    }
    if (t > t_profile_[PROFILE_STEPS-1]) return pressure_profile_[PROFILE_STEPS-1];
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
    if (msg.profile_time_setpoints_length != msg.profile_pressure_setpoints_length
        || msg.profile_pressure_setpoints_length < 1) return;

    int length = (msg.profile_pressure_setpoints_length < PROFILE_STEPS) ? msg.profile_pressure_setpoints_length : PROFILE_STEPS;
    unsigned long t_setpoints_millis[length];
    double pressure_setpoints[length];
    for (int i = 0; i < length; i++) {
        t_setpoints_millis[i] = (unsigned long)(msg.profile_time_setpoints[i] * 1000);
        pressure_setpoints[i] = (double)(msg.profile_pressure_setpoints[i]);
    }
    setProfile(t_setpoints_millis, pressure_setpoints);
    setGains(msg.pump_kp, msg.pump_ki, msg.pump_kd);
}

void PressureController::populateMessage() {
    msg_.header.stamp = nh_->now();
    msg_.input = *input_;
    msg_.setpoint = setpoint_;
    msg_.output = output_;
    msg_.output_p = output_p_;
    msg_.output_i = output_i_;
    msg_.output_d = output_d_;
    msg_.kd = kd_;
    msg_.ki = ki_;
    msg_.kd = kd_;
    msg_.active = active_;
}

void PressureController::overrideCallback(const std_msgs::Float64& msg){
    setOutput((double)msg.data);
}