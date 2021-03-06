#include "silvia_output.h"

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
