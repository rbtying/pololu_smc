#include "PID.h"

PID::PID() {
    PID(0, 0, 0, 100, 0, "pid");
}

PID::PID(double kP, double kI, double kD, double intLimit, double setpoint, std::string name) {
    setPID(kP, kI, kD);
    m_intLimit = intLimit;
    m_setpoint = setpoint;
    m_name = name;
}

void PID::setPID(double kP, double kI, double kD) {
    m_error = 0;
    m_perror = 0;
    m_accerror = 0;
    m_kP = kP;
    m_kI = kI;
    m_kD = kD;
}

void PID::setSetpoint(double sp) {
    m_setpoint = sp;
}

void PID::update(double input, double dt) {
    m_error = m_setpoint - input;
    m_accerror += m_error * dt;
    double derror = (m_error - m_perror) / dt;

    if (m_accerror > m_intLimit) {
        m_accerror = m_intLimit;
    } else if (m_accerror < -m_intLimit) {
        m_accerror = -m_intLimit;
    }
    m_output = m_kP * m_error + m_kI * m_accerror + m_kD * derror;
    m_perror = m_error;
}

double PID::getSetpoint() {
    return m_setpoint;
}

double PID::getOutput() {
    return m_output;
}

double PID::getError() {
    return m_error;
}

double PID::getAccError() {
    return m_accerror;
}
