#include "silvia_clean.h"
#include "silvia_modes.h"

// Cleaning cycle
CleaningProcess::CleaningProcess() 
    : SilviaPublisher("cleaner", &msg_, PUB_CLEANER_INTERVAL)
    , active_(false)
    , n_cycles_(5)
    , t_on_(10)
    , t_off_(50)
{}

void CleaningProcess::setup(NodeHandle* nh) {
    SilviaPublisher::setup(nh);
}

void CleaningProcess::start(int previous_mode) {
    active_ = true;
    previous_mode_ = previous_mode;
    t_total_ = n_cycles_ * (t_on_ + t_off_);
    duration_ = 0;
    last_update_ = 0;
    update();
}

void CleaningProcess::pumpOn() {
    if (!water_sensor.lowWater()) {  
        brew_output.on();
    }
}
void CleaningProcess::pumpOff() {
    brew_output.on();
}

void CleaningProcess::stop() {
    active_ = false;
    pumpOff();
}

int CleaningProcess::update() {
    if (!active_) return MODE_IGNORE;

    duration_ = millis() - last_update_;
    if (duration_ > UPDATE_INTERVAL) {
        if (duration_ > t_total_) {
            stop();
            return previous_mode_;
        } else if ((int)(duration_ / 1000) % (t_on_ + t_off_) < t_on_) {
            pumpOn();
            return MODE_CLEAN;
        } else {
            pumpOff();
            return MODE_CLEAN;
        }
    }
}

int CleaningProcess::getTimeRemaining() {
    return t_total_ - (int)(duration_ / 1000);
}
int CleaningProcess::getTimeRemainingMins() {
    return getTimeRemaining() / 60;
}
int CleaningProcess::getTimeRemainingSecs() {
    return getTimeRemaining() % 60;
}

void CleaningProcess::setTimings(int n_cycles, int t_on, int t_off) {
    n_cycles_ = n_cycles;
    t_on_ = t_on;
    t_off_ = t_off;
}

void CleaningProcess::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    setTimings(msg.n_clean_cycles, msg.t_clean_on, msg.t_clean_off);
}

void CleaningProcess::populateMessage() {
    msg_.header.stamp = nh_->now();
    int duration_s = (uint32_t)(duration_ / 1000);
    int duration_ns = (uint32_t)(1000000 * duration_ % 1000);
    msg_.duration = ros::Duration(duration_s, duration_ns);
    msg_.n_clean_cycles = n_cycles_;
    if (active_) msg_.current_cycle = (int)( duration_ / (t_on_ + t_off_) ) + 1;
    else msg_.current_cycle = 0;
    msg_.t_clean_on = t_on_;
    msg_.t_clean_off = t_off_;
}