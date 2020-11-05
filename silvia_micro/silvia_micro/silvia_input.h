#ifndef SILVIA_INTPUT_H
#define SILVIA_INTPUT_H

#include <Arduino.h>
#include "silvia_modes.h"
#include "silvia_state.h"

extern SilviaStatus silvia_status;

class SwitchInput {
    private:
        int pin_;  // Pin number
        bool status_;  // Last recorded status

    protected:
        virtual void changeState(bool status) = 0;

    public:
        SwitchInput(int pin);
        void update();
        bool getStatus();
};

class BrewSwitch : public SwitchInput {
    private:
        void changeState(bool status);
    public:
        BrewSwitch(int pin) : SwitchInput(pin) {};
};

class PowerSwitch : public SwitchInput {
    private:
        void changeState(bool status);
    public:
        PowerSwitch(int pin) : SwitchInput(pin) {};
};

#endif // SILVIA_INPUT_H
