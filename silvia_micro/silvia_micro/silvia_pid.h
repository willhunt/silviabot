#ifndef SILVIA_PID_H
#define SILVIA_PID_H

#include <Arduino.h>


class PidController {
  
    protected:
        double kp_;
        double ki_;
        double kd_;
        double output_;
        double output_p_;
        double output_i_;
        double output_d_;
        double output_min_;
        double output_max_;
        double setpoint_;
        double* input_;
        double last_input_;
        unsigned long update_interval_;
        unsigned long t_last_update_;
        bool active_;
        bool safety_limit_active_;  // Apply putput cutoff when input limit is active_
        double safety_limit_min_;
        double safety_limit_max_;

    public:
        PidController(double* input);
        virtual void controlOutput() = 0;
        void compute();
        void setSetpoint(double setpoint);
        void setGains(double kp, double ki, double kd);
        void setOutputLimits(double low, double high);
        void setUpdateInterval(unsigned long interval);
        void setOutput(double output);
        void activate();
        void deactivate();
        void reset();
        void limitOutput(double& output);
        void setSafetyLimit(double min_limit, double max_limit);
        void unsetSafetyLimit();

        double getSetpoint();
        double getKp();
        double getKi();
        double getKd();
        double getError();
        double getOutput();
        double getProportinalOutput();
        double getIntegralOutput();
        double getDerivativeOutput();
        bool isActive();
        bool hasSafetyLimit();
};

#endif //SILVIA_TEMP_SENSOR_H
