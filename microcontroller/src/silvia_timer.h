#ifndef SILVIA_TIMER_H
#define SILVIA_TIMER_H

#define PUB_TIMER_INTERVAL 250

#include <Arduino.h>
#include <ros.h>
#include <django_interface/SilviaBrewTimer.h>
#include "silvia_ros.h"

class BrewTimer : public SilviaPublisher {
    private:
        django_interface::SilviaBrewTimer timer_msg_;
        unsigned long start_time_;  // Time brew started [ms]
        unsigned long millis_passed_;  // Brew time passed [ms]

    public:
        BrewTimer();
        void reset();
        void start();
        unsigned long update(bool brewing);
        void populateMessage();
        int getDurationMins();
        int getDurationSecs();
};

#endif // SILVIA_TIMER_H
