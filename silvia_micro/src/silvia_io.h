#ifndef SILVIA_OUTPUT_H
#define SILVIA_OUTPUT_H

#include <Arduino.h>
#include <django_interface/SilviaStatus.h>
#include <ros.h>

class RelayOutput {
    private:
        int pin_;  // Pin number
        bool status_;  // Output state
  
    public:
        RelayOutput(int pin);
        void on();
        void off();
        bool getStatus();
        bool& status = status_;
};

class SwitchInput {
    private:
        int pin_;  // Pin number
        bool status_;  // Last recorded status
        ros::Publisher publisher_;
        django_interface::SilviaStatus status_msg_;

    protected:
        ros::NodeHandle* nh_;  // ROS nodehandle
        virtual void populateMessage(bool status) = 0;  // Inherited class must override

    public:
        SwitchInput(int pin);
        void setup(ros::NodeHandle* nh)
        void update();
        bool getStatus();
        void publish(bool status);
};

class BrewSwitch : public SwitchInput {
    private:
        populateMessage(bool status);
}

class PowerSwitch : public SwitchInput {
    private:
        populateMessage(bool status);
}

#endif // SILVIA_OUTPUT_H