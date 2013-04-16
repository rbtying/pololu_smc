#ifndef PID_H_
#define PID_H_

#include <string>

class PID {
    public:
        PID();
        PID(double kP, double kI, double kD, double intLimit, double setpoint, std::string name);
        void setPID(double kP, double kI, double kD);
        void update(double input, double dt);
        void setSetpoint(double sp);
        double getSetpoint();
        double getOutput();
        double getError();
        double getAccError();

    private:
        double m_kP, m_kI, m_kD, m_intLimit;
        double m_error, m_perror, m_accerror;
        double m_setpoint, m_output;
        std::string m_name;
};

#endif /* PID_H_ */
