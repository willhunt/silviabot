#include "silvia_input.h"

// SWITCH ------------------------------------------------------
SwitchInput::SwitchInput(int pin) 
    : pin_(pin)
    , status_(false)
{
    pinMode(pin, INPUT_PULLUP);
}

bool SwitchInput::getStatus() {
    bool status = (digitalRead(pin_) == LOW) ? true : false;
    return status;
}

void SwitchInput::update() {
    bool this_status = getStatus();
    if (this_status != status_) {  // If switch is changed
        changeState(this_status);
        status_ = this_status;
  }
}

void BrewSwitch::changeState(bool status) {
    silvia_status.changeBrew(status);
}
void PowerSwitch::changeState(bool status) {
    if (status) {
        silvia_status.changeMode(MODE_PID);
    } else {
        silvia_status.changeMode(MODE_OFF);
    }
}
