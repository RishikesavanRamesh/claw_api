#ifndef CLAW_API_HPP
#define CLAW_API_HPP

#include <iostream>
#include <cstdint>
#include <cstdarg>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <rclcpp/logger.hpp>
#include <string>

#define CLAW_API_M1FORWARD 0
#define CLAW_API_M1BACKWARD 1
#define CLAW_API_M2FORWARD 4
#define CLAW_API_M2BACKWARD 5
#define CLAW_API_DRIVEM1 6
#define CLAW_API_DRIVEM2 7
#define CLAW_API_DRIVEFORWARD 8
#define CLAW_API_DRIVEBACKWARD 9
#define CLAW_API_TURNRIGHT 10
#define CLAW_API_TURNLEFT 11
#define CLAW_API_DRIVEFORWARDORBACKWARD 12
#define CLAW_API_TURNLEFTORRIGHT 13
#define CLAW_API_MAXRETRY 3
#define CLAW_API_TIMEOUT 10 // Milliseconds
#define CLAW_API_READ_FIRMWARE_VERSION 21
class RoboClaw {
public:
     // A convenience struction to pass around configuration information.
    typedef struct {
        float p;
        float i;
        float d;
        uint32_t qpps;
        float max_current;
    } TPIDQ;
    RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent,
            float m2MaxCurrent, std::string device_name, uint8_t device_port,
            uint8_t vmin, uint8_t vtime);

    ~RoboClaw();

    void openPort(const std::string& device_name);
    void closePort();


    void doM1M2AccelSpeed(uint32_t accel_m1_quad_pulses_per_second,
                                        int32_t m1_quad_pulses_per_second,
                                    uint32_t accel_m2_quad_pulses_per_second,
                                        int32_t m2_quad_pulses_per_second) ;
    // For a custom exception message.
    struct TRoboClawException : public std::exception {
        std::string s;
        TRoboClawException(std::string ss) : s(ss) {}
        ~TRoboClawException() throw() {}
        const char *what() const throw() { return s.c_str(); }
    };



    // Get RoboClaw software versions.
	std::string getVersion();

      // Stop motion.
     void stop();


private:
  int device_port_;  // Unix file descriptor for RoboClaw connection.
  float m1p_;
  float m1i_;
  float m1d_;
  int m1qpps_;
  float m2p_;
  float m2i_;
  float m2d_;
  int m2qpps_;
  int maxCommandRetries_;    // Maximum number of times to retry a RoboClaw
                             // command.
  float maxM1Current_;       // Maximum allowed M1 current.
  float maxM2Current_;       // Maximum allowed M2 current.
  int motorAlarms_;          // Motors alarms. Bit-wise OR of contributors.
  std::string device_name_;  // Device name of RoboClaw device.
  int portAddress_;          // Port number of RoboClaw device under control
  int vmin_;                 // Terminal control value.
  int vtime_;                // Terminal control value.


    void restartPort();
    uint8_t readByteWithTimeout();
      // Update the running CRC result.
    void updateCrc(uint16_t &crc, uint8_t data);

    // Write one byte to the device.
    void writeByte(uint8_t byte);

    // Write a stream of bytes to the device.
    void writeN(bool sendCRC, uint8_t cnt, ...);
};


#endif // CLAW_API_HPP
