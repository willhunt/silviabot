#include "silvia_ros.h"

SilviaPublisher::SilviaPublisher(char* topic_name, ros::Msg* msg, unsigned long pub_interval)
    : publisher_(topic_name, msg)
    , msg_(msg)
    , pub_interval_(pub_interval)
    , pub_time_(0)
{}

void SilviaPublisher::setup(ros::NodeHandle* nh) {
    nh_ = nh;
    nh_->advertise(publisher_);
}

void SilviaPublisher::publish() {
    if (millis() - pub_time_ < pub_interval_) {
        populateMessage();
        publisher_.publish(msg_);
        pub_time_ = millis();
    }
}

int SilviaPublisher::getPubInterval() {
    return pub_interval_;
}
void SilviaPublisher::setPubInterval(int interval) {
    pub_interval_ = interval;
}