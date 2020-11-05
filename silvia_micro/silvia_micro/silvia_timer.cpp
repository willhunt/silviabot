#include "silvia_timer.h"
#include <std_msgs/Duration.h>

BrewTimer::BrewTimer()
    : SilviaPublisher("brew_duration", &timer_msg_, PUB_TIMER_INTERVAL)
{
    timer_msg_.header.frame_id = "micro";
}

void BrewTimer::reset() {
    millis_passed_ = 0;
}

void BrewTimer::start() {
    start_time_ = millis();
}

unsigned long BrewTimer::update(bool brewing) {
    if (brewing) {
        millis_passed_ = millis() - start_time_;
    }
    return millis_passed_;
}

void BrewTimer::populateMessage() {
    timer_msg_.header.stamp = nh_->now();
    int duration_s = (uint32_t)(millis_passed_ / 1000);
    int duration_ns = (uint32_t)(1000000 * millis_passed_ % 1000);
    timer_msg_.duration = ros::Duration(duration_s, duration_ns);
}

int BrewTimer::getDurationMins() {
    return millis_passed_ / 60000;
}
int BrewTimer::getDurationSecs() {
    return (millis_passed_ / 1000) % 60;
}