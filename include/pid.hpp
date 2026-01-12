#pragma once

class PID {
    public:
    PID(double kp, double ki, double kd);
    double compute(double setpoint, double input, double dt);
    void set_gains(double kp, double ki, double kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }
    void reset() {
        _prev_error = 0;
        _integral = 0;
    }
    private:
        double _kp;
        double _ki;
        double _kd;
        double _prev_error;
        double _integral;
        double _max_integral;
        double _max_output;
};