#ifndef SILVIA_SENSORS_H
#define SILVIA_SENSORS_H

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <django_interface/SilviaWater.h>
#include "silvia_ros.h"

#define FILTER_LAG 0
#define FILTER_MOVINGAVERAGE 1

#define MA_FILTER_WINDOW_SIZE 10
#define AVERAGING_INTERVAL 200  // ms
#define TEMPERATURE_SENSOR_COEFFICIENT 0.48828125  // LM35 sensor [(1.0/1024.0)*5.0*100.0], gradient degC/V
#define PRESSURE_SENSOR_COEFFICIENT 2526.88172  // gradient Pa/int [2.068e+6 / (0.8 * 1023)]
#define PRESSURE_SENSOR_OFFSET 102.3
#define SMOOTHING_FILTER_VAL 0.8  // How much smoothing (1=100% filtered(no update), 0=no filtering)
#define PUB_TEMPERATURE_INTERVAL 250
#define PUB_PRESSURE_INTERVAL 250
#define PUB_WATERLEVEL_INTERVAL 1000

/* 
Generic sensor with filter
*/
class SilviaSensor {
    private:
        double reading_sum_;  // Sum readings to take average
        int reading_count_;  // Count readings to take average
        double smoothing_filter_val_;  // Between 0 and 1 for smoothing function (small=more smooth)
        double averaging_interval_;  // Interval over which to average [millis]
        unsigned long reading_time_;  // Time at which last reading was taken
        double ma_readings_[MA_FILTER_WINDOW_SIZE]; // Store of previous readings for moving average filter
        int ma_index_; // Moving average index
        double ma_sum_; // Moving average Sum
        void reset();  // Reset variables
        int filter_type_;

    protected:
        double reading_last_;  // Last temperature last_reading
        virtual double readSensor() {return 0.0;};  // Read sensor

    public:
        SilviaSensor(int filter_type);
        void updateAverage();  // Updates sum and reading count
        double update();  // Get temperature, either last or new depending on interval
        double getLatestReading(); // Get latest without updating
        void updateAveragingInterval(int interval);  // Update interval [seconds]
        double& reading = reading_last_;
};


/* 
Temperature sensor - LM35
*/
class TemperatureSensor : public SilviaPublisher, public SilviaSensor {
  
    private:
        sensor_msgs::Temperature temperature_msg_;
        int sensor_pin_;  // Analog pin number
        void populateMessage();
        double readSensor();

    public:
        TemperatureSensor(int sensor_pin);
};

/* 
Pressure sensor - linear response
0.5V = 0PSI, 4.5V = 300PSI
*/
class PressureSensor : public SilviaPublisher, public SilviaSensor {
  
    private:
        sensor_msgs::FluidPressure pressure_msg_;
        int sensor_pin_;  // Analog pin number
        double zero_offset_;
        void populateMessage();
        double readSensor();

    public:
        PressureSensor(int sensor_pin);
        void setup(NodeHandle* nh);
};

/* 
Water level sensor class
Liquid Level Sensor-XKC-Y25-T12V
Outputs either true or false for water detected
*/
class WaterLevelSensor : public SilviaPublisher {
  
    private:
        django_interface::SilviaWater water_msg_;
        int sensor_pin_;  // Analog pin number
    
    public:
        WaterLevelSensor(int sensor_pin);
        bool lowWater();  // Get level
        void populateMessage();
};


#endif //SILVIA_SENSORS_H