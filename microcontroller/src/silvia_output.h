#ifndef SILVIA_OUTPUT_H
#define SILVIA_OUTPUT_H

#include <Arduino.h>
#include "silvia_modes.h"

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

#endif // SILVIA_OUTPUT_H
