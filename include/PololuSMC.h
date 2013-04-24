#ifndef POLOLU_SMC_H_
#define POLOLU_SMC_H_

#include <string>
#include <serial/serial.h>

class PololuSMC{
    public:
        /*!
         * Constructs a PololuSMC driver
         * @param ser the port to use
         * @param id the motor id
         */
        PololuSMC(serial::Serial *serialport, uint8_t id);

        /*!
         * Destructs the controller. 
         */
        ~PololuSMC();

        /**
         * Opens the serial port
         */
        void open();

        /**
         * Closes the serial port
         */
        void close();

        /**
         * Checks if the port is open
         * @return 0 if the port is not open
         */
        uint8_t isOpen();

        /**
         * Gets the port name
         * @return name of the current serial port
         */
        std::string getPortName();

        /* API COMMANDS */

        /*!
         * Exits safe start mode and enables motor control
         */
        void exitSafeStart();

        /*!
         * Sets the motor speed
         * @param amt -1.0 to +1.0 as a percentage
         */
        void setMotorSpeed(double amt);

        /*!
         * Sets the motor to brake immediately
         * @param amt 0 to 1.0
         */
        void setMotorBrake(double amt);

        /*!
         * Sets a motor limit
         * @param lim the limit to set
         * @param data its value
         */
        uint8_t setMotorLimit(uint8_t lim, uint16_t data);

        /*!
         * Emergency-stops the motor
         */
        void estop();

        /*!
         * Gets the raw RC1 reading in microseconds
         * @return rc1 in us
         */
        double getRC1usec();

        /*
         * Gets the raw RC2 reading in microseconds
         * @return RC2 in us
         */
        double getRC2usec();

        /*!
         * Gets the scaled RC1 reading (-1.0 to 1.0)
         * @return RC1 as a percentage
         */
        double getRC1();

        /*!
         * Gets the scaled RC2 reading (-1.0 to 1.0)
         * @return RC2 as a percentage
         */
        double getRC2();

        /*!
         * Gets the analog reading from AN1 in volts
         * @return AN1 in volts
         */
        double getAN1();
        /*!
         * Gets the analog reading from AN2 in volts
         * @return AN2 in volts
         */
        double getAN2();

        /*!
         * Gets the target speed
         * @return the target speed (-3200 to 3200)
         */
        int16_t getTargetSpeed();

        /*!
         * Gets the current speed
         * @return the current speed (-3200 to 3200)
         */
        int16_t getCurrentSpeed();

        /*!
         * Gets the brake amount
         * @return the break amount (0 to 32)
         */
        int16_t getBrakeAmount();

        /*!
         * Gets the input voltage
         * @return the input voltage
         */
        double getVoltage();

        /*!
         * Gets the temperature
         * @return the temperature
         */
        double getTemperature();

    private:
        /**
         * Initializes the position controller
         */
        void initialize();

        /**
         * Deserializes a 16-bit signed integer. Assumes that the high byte is
         * first.
         * @param buf the buffer to read from. Precondition: buf holds at least
         * two bytes
         * @return a 16-bit signed integer formed by shifting the bytes in buf
         */
        int16_t construct_int16(uint8_t * buf);

        /*!
         * Reformats from standard 0-3600 to Pololu's
         * low-5-bits << 8 | high 7 bits
         *
         * @param std the original input
         * @return the new output
         */
        uint16_t format_uint16_3600(uint16_t std);

        /*!
         * Reformats from standard 0-16384 to Pololu's
         * low-7-bits << 8 | high 7 bits
         *
         * @param std the original input
         * @return the new output
         */
        uint16_t format_uint16_16384(uint16_t std);

        /**
         * Allocates a new string to hold all members of the byte array,
         * printed in the format 0xXX where XX is the hexadecimal value of the
         * byte.
         * @param buf the byte array to print
         * @param buflen the length of the byte array
         * @return pointer to string (needs to be freed)
         */
        char* byteArrayToString(uint8_t * buf, uint8_t buflen);

        /*!
         * Sends a command with the attached data
         * @param cmd the command to send
         * @param data the additional data to send
         * @param datalen the length of the data to send
         */
        void send_command(uint8_t cmd, uint8_t *data, uint8_t datalen);

        /*!
         * Gets a variable identified by var from the controller (blocking)
         *
         * @param var
         * @return the value of the variable
         */
        uint16_t get_variable(uint8_t var);

        // members
        std::string m_portname;
        serial::Serial * m_serial;
        uint8_t m_id;

        // constants
        static const int BAUDRATE = 1000000;
        static const int TIMEOUT = 100;
};

#endif /* POLOLU_SMC_H_ */
