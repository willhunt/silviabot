#include "silvia_sensors.h"

// GENERIC SENSOR
SilviaSensor::SilviaSensor(int filter_type)
    : averaging_interval_(AVERAGING_INTERVAL)
    , sensor_coefficient_(SENSOR_COEFFICIENT)
    , smoothing_filter_val_(SMOOTHING_FILTER_VAL)
    , ma_index_(0)
    , filter_type_(filter_type)
{
    // Rest filter variables
    reset();
    // Setup moving average filter
    reading_last_ = readSensor();
    for (int i = 0; i < MA_FILTER_WINDOW_SIZE; i++) {
        ma_readings_[i] = reading_last_;
    }
    ma_sum_ = reading_last_ * MA_FILTER_WINDOW_SIZE;
}

void SilviaSensor::updateAverage() {
    reading_sum_ += readSensor();
    reading_count_ += 1;
}

void SilviaSensor::reset() {
    reading_sum_ = 0.0;
    reading_count_ = 0;
    reading_time_ = millis();
}

double SilviaSensor::update() {
    /*
    Get temperature. Either updates average or gets new reading depending on interval since last update
    */
    if ((millis() - reading_time_) < averaging_interval_) {
        updateAverage();
    } else {
        double reading_average;
        // Check for division by zero, was causing nan problem
        if (reading_count_ == 0) {
            reading_average = readSensor();
        } else {
            reading_average = reading_sum_ / (double)reading_count_;
        }
        // Apply smoothing
        if (filter_type_ == FILTER_LAG) {
            reading_last_ = reading_average * (1 - smoothing_filter_val_) + \
            reading_last_ * smoothing_filter_val_;
        } else if (filter_type_ == FILTER_MOVINGAVERAGE) { // moving average
            ma_sum_ -= ma_readings_[ma_index_];  // Remove the oldest entry from the sum
            ma_sum_ += reading_average;  // Add new value to sum
            ma_readings_[ma_index_] = reading_average;  // Replace with new value in store
            ma_index_ = (ma_index_ + 1) % MA_FILTER_WINDOW_SIZE;  // Increment index
            reading_last_ = ma_sum_ / MA_FILTER_WINDOW_SIZE;
        } else {  // No filter
            reading_last_ = reading_average;
        }
        // Reset
        reset();
    }
    return reading_last_;
}

double SilviaSensor::getLatestReading() {
    return reading_last_;
}

void SilviaSensor::updateAveragingInterval(int interval) {
    averaging_interval_ = interval;
}



// TEMPERATURE SENSOR --------------------------------------------------------------
TemperatureSensor::TemperatureSensor(int sensor_pin)
    : SilviaPublisher("/micro/boiler_temperature", &temperature_msg_, PUB_TEMPERATURE_INTERVAL)
    , SilviaSensor(FILTER_MOVINGAVERAGE)
    , sensor_pin_(sensor_pin)
{
    temperature_msg_.header.frame_id = "micro";
    temperature_msg_.variance = 0.1;
}

double TemperatureSensor::readSensor() {
    // return analogRead(sensor_pin_) * sensor_coefficient_;
    return 37.0;
}

void TemperatureSensor::populateMessage() {
    temperature_msg_.header.stamp = nh_->now();
    temperature_msg_.temperature = reading_last_;
}



// WATER SENSOR --------------------------------------------------------------
WaterLevelSensor::WaterLevelSensor(int sensor_pin) 
    : SilviaPublisher("/micro/water_level", &water_msg_, PUB_WATERLEVEL_INTERVAL)
    , sensor_pin_(sensor_pin)
{
    pinMode(sensor_pin_, INPUT);
    water_msg_.header.frame_id = "micro";
}

bool WaterLevelSensor::lowWater() {
  // Returns True if water in tank
  // return digitalRead(sensor_pin_);
  return false; // Temporary fix
}

void WaterLevelSensor::populateMessage() {
    water_msg_.header.stamp = nh_->now();
    water_msg_.low_water = lowWater();
}