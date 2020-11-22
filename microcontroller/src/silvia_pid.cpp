#include "silvia_pid.h"

PidController::PidController(double* input) 
    : update_interval_(100)
    , kp_(0), ki_(0), kd_(0)
    , setpoint_(0)
    , output_p_(0), output_i_(0), output_d_(0)
    , active_(false)
    , input_(input)
    , last_input_(*input)
    , safety_limit_active_(false)
{
    t_last_update_ = millis() - update_interval_;
}

void PidController::setSetpoint(double setpoint) {
    setpoint_ = setpoint;
}
void PidController::setOutput(double output) {
    output_ = output;
    limitOutput(output_);
}
void PidController::setGains(double kp, double ki, double kd) {
    // if (kp < 0 || ki < 0 || kd < 0) return;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
void PidController::setOutputLimits(double low, double high) {
    if (low > high) return;
    output_min_ = low;
    output_max_ = high;
    limitOutput(output_);
}
void PidController::setUpdateInterval(unsigned long interval) {
    update_interval_ = interval;
}

void PidController::setSafetyLimit(double min_limit, double max_limit) {
    safety_limit_min_ = min_limit;
    safety_limit_max_ = max_limit;
    safety_limit_active_ = true;
}
void PidController::unsetSafetyLimit() {
    safety_limit_active_ = false;
}

void PidController::activate() {
    if (!active_) reset();
    active_ = true;
}
void PidController::deactivate() {
    active_ = false;
}
void PidController::reset() {
    setOutput(0);  // May not be best for all scenarios
    output_i_ = output_;
    limitOutput(output_i_);
    last_input_ = *input_;
}

void PidController::compute() {
    // Check safety limit
    if (safety_limit_active_) {
        if (*input_ <= safety_limit_min_ || *input_ >= safety_limit_max_) {
            output_ = 0;
            reset();  // Avoid windup
            return;
        }
    }
    // Do nothing if not set to active
    if (!active_) return;

    unsigned long t_now = millis();
    unsigned long t_delta = t_now - t_last_update_;
    if (t_delta >= update_interval_) {
        double error = setpoint_ - *input_;
        // Proportional term
        output_p_ = error * kp_;
        // Integral term (additive) - with limits
        output_i_ += error * ki_ * ((double)t_delta / 1000.0);
        limitOutput(output_i_);
        // Derivative term
        output_d_ = (last_input_ - *input_) * kd_ / ((double)t_delta / 1000.0);
        // Sum outputs
        setOutput(output_p_ + output_i_ + output_d_);

        last_input_ = *input_;
        t_last_update_ = t_now;
    }

}

void PidController::limitOutput(double& output) {
    if (output > output_max_) output = output_max_;
    else if (output < output_min_) output = output_min_;
}

double PidController::getSetpoint() { return setpoint_; }
double PidController::getKp() { return kp_; }
double PidController::getKi() { return ki_; }
double PidController::getKd() { return kd_; }
double PidController::getError() { return setpoint_ - *input_; }
double PidController::getOutput() { return output_; }
double PidController::getProportinalOutput() { return output_p_; }
double PidController::getIntegralOutput() { return output_i_; }
double PidController::getDerivativeOutput() { return output_d_; }
bool PidController::isActive() { return active_; }
