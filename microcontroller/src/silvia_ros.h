#ifndef SILVIA_ROS_H
#define SILVIA_ROS_H

#include <Arduino.h>
#include <ros.h>

/*
Node handle
*/
// MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE
typedef ros::NodeHandle_<ArduinoHardware, 5, 10, 256, 256> NodeHandle;

/* 
Rosserial publisher class
*/
class SilviaPublisher {
  
    private:
        ros::Publisher publisher_;  // ROS publisher
        ros::Msg* msg_;
        unsigned long pub_time_;  // Time last published
        int pub_interval_;  // Interval between publishings [ms]

    protected:
        NodeHandle* nh_;  // ROS nodehandle
        virtual void populateMessage() = 0;  // Inherited class must override

    public:
        SilviaPublisher(char* topic_name, ros::Msg* msg, unsigned long pub_interval);
        void setup(NodeHandle* nh);
        void setPubInterval(int interval);  // Update interval [seconds]
        int getPubInterval();
        void publish(bool force=false);  // Publish temperature
};

#endif //SILVIA_ROS_H