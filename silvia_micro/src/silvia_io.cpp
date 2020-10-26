#include "silvia_io.h"

// RELAY ------------------------------------------------------
RelayOutput::RelayOutput(int pin) 
    : pin_(pin), status_(false) {
    pinMode(pin_, OUTPUT);
};

bool RelayOutput::getStatus() {
    return status_;
};

void RelayOutput::on() {
    status_ = true;
    digitalWrite(pin_, HIGH);
};

void RelayOutput::off() {
    status_ = false;
    digitalWrite(pin_, LOW);
};


// SWITCH ------------------------------------------------------
SwitchInput::SwitchInput(int pin) 
    : pin_(pin)
{
    pinMode(pin, INPUT_PULLUP);
    status_ = getStatus();
    status_msg_.header.frame_id = "micro";
}

void SwitchInput::setup(ros::NodeHandle* nh) {
    nh_ = nh;
    nh_->advertise(publisher_);
}

bool SwitchInput::getStatus() {
    bool status = (digitalRead(pin_) == LOW) ? true : false;
    return status;
}

void SwitchInput::update() {
    bool this_status = getStatus();
    if (this_status != status_) {  // If switch is changed
        publish(this_status);
        status_ = this_status;
  }
}

void SwitchInput::publish(bool status) {
    status_msg_.header.stamp = nh_->now();
    populateMessage(status);
    publisher_.publish(msg_);
}

void BrewSwitch::populateMessage(bool status) {
    status_msg_.brew = status;
}
void PowerSwitch::populateMessage(bool status) {
    if (status) status_msg_.mode = MODE_PID;
    else status_msg_.mode = MODE_OFF;
}