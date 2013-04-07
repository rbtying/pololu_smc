#include "PololuSMC.h"

#include <string>
#include <ros/ros.h>
#include <serial/serial.h>

namespace SMCMD {
    enum command_t {
        CMD_EXIT_SAFE_START = 0x03,
        CMD_MOTOR_FWD = 0x05,
        CMD_MOTOR_REV = 0x06,
        CMD_MOTOR_BRAKE = 0x12,
        CMD_GET_VAR = 0x21,
        CMD_SET_MOTOR_LIMIT = 0x22,
        CMD_FW_VERSION = 0x42,
        CMD_MOTOR_ESTOP = 0x60,
    };

    enum var_id_t {
        VAR_ERR_STATUS = 0x00,
        VAR_ERR_OCCURED = 0x01,
        VAR_SER_ERR_OCC = 0x02,
        VAR_LIMIT_STATUS = 0x03,
        VAR_RESET_FLAGS = 0x7f,
        
        VAR_RC1_RAW_UNLIM = 0x04,
        VAR_RC1_RAW_VALUE = 0x05,
        VAR_RC1_SCL_VALUE = 0x06,
        VAR_RC2_RAW_UNLIM = 0x08,
        VAR_RC2_RAW_VALUE = 0x09,
        VAR_RC2_SCL_VALUE = 0x0a,

        VAR_AN1_RAW_UNLIM = 0x0c,
        VAR_AN1_RAW_VALUE = 0x0d,
        VAR_AN1_SCL_VALUE = 0x0e,
        VAR_AN2_RAW_UNLIM = 0x10,
        VAR_AN2_RAW_VALUE = 0x11,
        VAR_AN2_SCL_VALUE = 0x12,

        VAR_TARGET_SPD = 20,
        VAR_CURRENT_SPD = 21,
        VAR_BRAKE_AMT = 22,
        VAR_INPUT_VOLTAGE = 23,
        VAR_TEMPERATURE = 24,
        VAR_RC_PERIOD = 26,
        VAR_BAUD_RATE = 27,
        VAR_SYS_TIME_L = 28,
        VAR_SYS_TIME_H = 29,

        VAR_MAX_SPEED_FWD = 30,
        VAR_MAX_ACCEL_FWD = 31,
        VAR_MAX_DECEL_FWD = 32,
        VAR_MAX_BRAKE_FWD = 33,
        VAR_MAX_SPEED_REV = 36,
        VAR_MAX_ACCEL_REV = 37,
        VAR_MAX_DECEL_REV = 38,
        VAR_MAX_BRAKE_REV = 39,
    };

    enum limit_id_t {
        LIM_SPEED_SYM = 0x00,
        LIM_ACCEL_SYM = 0x01,
        LIM_DECEL_SYM = 0x02,
        LIM_BRAKE_SYM = 0x03,
        LIM_SPEED_FWD = 0x04,
        LIM_ACCEL_FWD = 0x05,
        LIM_DECEL_FWD = 0x06,
        LIM_BRAKE_FWD = 0x07,
        LIM_SPEED_REV = 0x08,
        LIM_ACCEL_REV = 0x09,
        LIM_DECEL_REV = 0x0A,
        LIM_BRAKE_REV = 0x0B
    };

    enum error_t {
        ERR_SAFE_START = 1 << 0,
        ERR_INVALID_CHANNEL = 1 << 1,
        ERR_SERIAL = 1 << 2,
        ERR_CMD_TIMEOUT = 1 << 3,
        ERR_LIMIT_SWITCH = 1 << 4,
        ERR_LOW_VIN = 1 << 5,
        ERR_HIGH_VIN = 1 << 6,
        ERR_OVER_TEMP = 1 << 7,
        ERR_DRIVER = 1 << 8,
        ERR_LINE_HIGH = 1 << 9
    };

    enum ser_error_t {
        SERR_FRAME = 1 << 1,
        SERR_NOISE = 1 << 2,
        SERR_RX_OVERRUN = 1 << 3,
        SERR_FORMAT = 1 << 4,
        SERR_CRC = 1 << 5
    };

    enum lim_status_t {
        STAT_ERR = 1 << 0,
        STAT_TEMP = 1 << 1,
        STAT_SPD_MAX = 1 << 2,
        STAT_SPD_START = 1 << 3,
        STAT_SPD_NEQL = 1 << 4,
        STAT_RC1 = 1 << 5,
        STAT_RC2 = 1 << 6,
        STAT_AN1 = 1 << 7,
        STAT_AN2 = 1 << 8,
        STAT_USB = 1 << 9
    };

    static const uint8_t START_BYTE = 0xAA;
};

PololuSMC::PololuSMC(std::string ser, uint8_t id) { 
    m_serial = new serial::Serial(ser, BAUDRATE);
    m_id = id;
}

PololuSMC::~PololuSMC() {
    // stop all of the controllers
    close();
    delete m_serial;
}

void PololuSMC::open() {
    // configure serial for 19200 8-N-1
    m_serial->setBaudrate(BAUDRATE);
    m_serial->setStopbits(serial::stopbits_one);
    m_serial->setParity(serial::parity_none);
    m_serial->setBytesize(serial::eightbits);
    m_serial->setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    m_serial->setTimeout(timeout);
    ROS_INFO("Serial port %s configured", this->getPortName().c_str());

    if (!m_serial->isOpen()) {
        m_serial->open();
        ROS_INFO("Serial port %s opened", this->getPortName().c_str());
    } else {
        ROS_INFO("Serial port %s already opened", this->getPortName().c_str());
    }
}

void PololuSMC::initialize() {
    ROS_INFO("Initializing Parallax Position Controller with id 0x%x", m_id);
}

void PololuSMC::close() {
    if (m_serial->isOpen()) {
        m_serial->close();
        ROS_INFO("Serial port %s closed", this->getPortName().c_str());
    } else {
        ROS_INFO("Serial port %s already closed", this->getPortName().c_str());
    }
}

std::string PololuSMC::getPortName() {
    return m_serial->getPort();
}

int16_t PololuSMC::construct_int16(uint8_t * buf) {
    // assume high byte first
    return (buf[0] << 8u) | buf[1];
}

char* PololuSMC::byteArrayToString(uint8_t * buf, uint8_t buflen) {
    char * str = reinterpret_cast<char*>(malloc(buflen * 5 + 3));
    memset(str, 0, buflen * 5 + 1);

    char * ptr = str + 1;
    for (uint8_t i = 0; i < buflen; i++) {
        ptr += sprintf(ptr, "0x%02x ", buf[i]);
    }
    str[0] = '[';
    str[buflen * 5 + 1] = ']';
    str[buflen * 5 + 2] = '\0';
    return str;
}

uint16_t PololuSMC::format_uint16_3600(uint16_t std) {
    uint8_t byte1 = std & 0x1f;
    uint8_t byte2 = std >> 5;
    return byte2 << 8u | byte1;
}

uint16_t PololuSMC::format_uint16_16384(uint16_t std) {
    uint8_t byte1 = std & 0x7f;
    uint8_t byte2 = std >> 7;
    return byte2 << 8u | byte1;
}

void PololuSMC::send_command(uint8_t cmd, uint8_t *data, uint8_t datalen) {
    uint8_t header[] = { SMCMD::START_BYTE, m_id };
    m_serial->write(header, 2);
    m_serial->write(data, datalen);
}

/* API COMMANDS */
void PololuSMC::exitSafeStart() {
    send_command(SMCMD::CMD_EXIT_SAFE_START, NULL, 0);
}

void PololuSMC::setMotorSpeed(double amt) {
    uint16_t desired_speed = std::min<double>(abs(amt), 1.0) * 3600; 
    desired_speed = format_uint16_3600(desired_speed);

    exitSafeStart();

    if (amt < 0) {
        send_command(SMCMD::CMD_MOTOR_REV, reinterpret_cast<uint8_t*>(&desired_speed), 2);
    } else {
        send_command(SMCMD::CMD_MOTOR_FWD, reinterpret_cast<uint8_t*>(&desired_speed), 2);
    }
}

void PololuSMC::setMotorBrake(double amt) {
    uint8_t j = std::min<double>(abs(amt), 1.0) * 32;
    send_command(SMCMD::CMD_MOTOR_BRAKE, &j, 1);
}

uint16_t PololuSMC::get_variable(uint8_t var) {
    uint8_t read_buf[2];
    send_command(SMCMD::CMD_GET_VAR, &var, 1);
    m_serial->read(read_buf, 2);
    int16_t convert = construct_int16(read_buf);
    return convert;
}

uint8_t PololuSMC::setMotorLimit(uint8_t lim, uint16_t data) {
    uint8_t read_buf = 0;
    uint8_t send_buf[3];
    send_buf[0] = lim;
    data = format_uint16_16384(data);
    memcpy(send_buf + 1, &data, 2);
    send_command(SMCMD::CMD_SET_MOTOR_LIMIT, send_buf, 3);
    m_serial->read(&read_buf, 1);
    return read_buf;
}

void PololuSMC::estop() {
    send_command(SMCMD::CMD_MOTOR_ESTOP, NULL, 0);
}

double PololuSMC::getRC1usec() {
    uint16_t val = get_variable(SMCMD::VAR_RC1_RAW_UNLIM);
    return val * 0.25;
}

double PololuSMC::getRC2usec() {
    uint16_t val = get_variable(SMCMD::VAR_RC2_RAW_UNLIM);
    return val * 0.25;
}

double PololuSMC::getRC1() {
    int16_t val = get_variable(SMCMD::VAR_RC1_SCL_VALUE);
    return val * 1.0 / 3600;
}

double PololuSMC::getRC2() {
    int16_t val = get_variable(SMCMD::VAR_RC2_SCL_VALUE);
    return val * 1.0 / 3600;
}

double PololuSMC::getAN1() {
    uint16_t val = get_variable(SMCMD::VAR_AN1_RAW_UNLIM);
    return val * 3.3 / 4096;
}

double PololuSMC::getAN2() {
    uint16_t val = get_variable(SMCMD::VAR_AN2_RAW_UNLIM);
    return val * 3.3 / 4096;
}

int16_t PololuSMC::getTargetSpeed() {
    return get_variable(SMCMD::VAR_TARGET_SPD);
}

int16_t PololuSMC::getCurrentSpeed() {
    return get_variable(SMCMD::VAR_CURRENT_SPD);
}

int16_t PololuSMC::getBrakeAmount() {
    return get_variable(SMCMD::VAR_BRAKE_AMT);
}

double PololuSMC::getTemperature() {
    return get_variable(SMCMD::VAR_TEMPERATURE) * 0.1;
}

double PololuSMC::getVoltage() {
    return get_variable(SMCMD::VAR_INPUT_VOLTAGE) * 0.001;
}
