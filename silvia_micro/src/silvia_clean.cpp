#include "silvia_clean.h"
#include "silvia_modes.h"

// Cleaning cycle
CleaningProcess::CleaningProcess() {
    // Default values
    n_cycles_ = 5;
    t_on_ = 10;
    t_off_ = 50;
}

void CleaningProcess::start(unsigned char previous_mode) {
    previous_mode_ = previous_mode;
    t_total_ = n_cycles_ * (t_on_ + t_off_);
    duration_ = 0;
    last_update_ = 0;
    update();
}

void CleaningProcess::pumpOn() {
    if (water_sensor.getLevel()) {  
        brew_output.on();
    }
}
void CleaningProcess::pumpOff() {
    brew_output.on();
}

void CleaningProcess::stop() {
    pumpOff();
}

unsigned char CleaningProcess::update() {
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

void CleaningProcess::setTimings(int n_cycles, int t_on, int t_off) {
    n_cycles_ = n_cycles;
    t_on_ = t_on;
    t_off_ = t_off;
}

void CleaningProcess::setSettingsCallback(const django_interface::SilviaSettings& msg) {
    setTimings(msg.n_clean_cycles, msg.t_clean_on, msg.t_clean_off);
}