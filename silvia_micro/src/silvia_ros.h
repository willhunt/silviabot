#ifndef SILVIA_ROS_H
#define SILVIA_ROS_H

#include <Arduino.h>
#include <ros.h>

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
        ros::NodeHandle* nh_;  // ROS nodehandle
        virtual void populateMessage() = 0;  // Inherited class must override

    public:
        SilviaPublisher(char* topic_name, ros::Msg* msg, unsigned long pub_interval);
        void setup(ros::NodeHandle* nh);
        void setPubInterval(int interval);  // Update interval [seconds]
        int getPubInterval();
        void publish();  // Publish temperature
};

#endif //SILVIA_ROS_H