#include "claw_api/claw_api.hpp"

#include <iostream>
#include <cstdint>
#include <cstdarg>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <string>


#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <rcutils/logging_macros.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <boost/assign.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>


#define SetDWORDval(arg) \
  (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg

RoboClaw::RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent,
                   float m2MaxCurrent, std::string device_name,
                   uint8_t device_port, uint8_t vmin, uint8_t vtime)
    : device_port_(device_port),
      maxCommandRetries_(3),
      maxM1Current_(m1MaxCurrent),
      maxM2Current_(m2MaxCurrent),
      device_name_(device_name),
      portAddress_(128),
      vmin_(vmin),
      vtime_(vtime) {
  openPort(device_name);
  RCUTILS_LOG_INFO("[RoboClaw::RoboClaw] RoboClaw software version: %s",
                   getVersion().c_str());
//   setM1PID(m1Pid.p, m1Pid.i, m1Pid.d, m1Pid.qpps);
//   setM2PID(m2Pid.p, m2Pid.i, m2Pid.d, m2Pid.qpps);
}

RoboClaw::~RoboClaw() {}


std::string RoboClaw::getVersion() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      uint16_t crc = 0;
      updateCrc(crc, portAddress_);
      updateCrc(crc, 21);
      writeN(false, 2, portAddress_, 21);

      uint8_t i;
      uint8_t datum;
      std::stringstream version;

      for (i = 0; i < 48; i++) {
        datum = readByteWithTimeout();
        updateCrc(crc, datum);
        if (datum == 0) {
          uint16_t responseCrc = 0;
          datum = readByteWithTimeout();
          responseCrc = datum << 8;
          datum = readByteWithTimeout();
          responseCrc |= datum;
          if (responseCrc == crc) {
            return version.str();
          } else {
            RCUTILS_LOG_ERROR(
                "[RoboClaw::getVersion] invalid CRC expected: 0x%02X, "
                "got: 0x%02X",
                crc, responseCrc);
          }
        } else {
          version << (char)datum;
        }
      }

      RCUTILS_LOG_ERROR("[RoboClaw::getVersion] unexpected long string");
      throw new TRoboClawException(
          "[RoboClaw::getVersion] unexpected long string");
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::getVersion] Exception: %s, retry number: %d", e->what(),
          retry);
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::getVersion] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::getVersion] RETRY COUNT EXCEEDED");
  throw new TRoboClawException("[RoboClaw::getVersion] RETRY COUNT EXCEEDED");
}



void RoboClaw::openPort(const std::string& device_name_) {

    RCUTILS_LOG_INFO("[RoboClaw::openPort] about to open port: %s",
                device_name_.c_str());

    device_port_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (device_port_ < 0) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::openPort] Unable to open USB port: %s, errno: (%d) "
            "%s",
            device_name_.c_str(), errno, strerror(errno));
        throw new TRoboClawException(
            "[RoboClaw::openPort] Unable to open USB port");
    }

    // Fetch the current port settings.
    struct termios portOptions;
    int ret = 0;

    ret = tcgetattr(device_port_, &portOptions);
    if (ret < 0) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::openPort] Unable to get terminal options "
            "(tcgetattr), error: %d: %s",
            errno, strerror(errno));
        // throw new TRoboClawException("[RoboClaw::openPort] Unable to get
        // terminal options (tcgetattr)");
    }

    // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
    //   this program from "owning" the port and to enable receipt of data.
    //   Also, it holds the settings for number of data bits, parity, stop bits,
    //   and hardware flow control.
    portOptions.c_cflag |= CLOCAL;    // Prevent changing ownership.
    portOptions.c_cflag |= CREAD;     // Enable reciever.
    portOptions.c_cflag &= ~CRTSCTS;  // Disable hardware CTS/RTS flow control.
    portOptions.c_cflag |= CS8;       // Enable 8 bit characters.
    portOptions.c_cflag &= ~CSIZE;    // Remove size flag.
    portOptions.c_cflag &= ~CSTOPB;   // Disable 2 stop bits.
    portOptions.c_cflag |=
        HUPCL;  // Enable lower control lines on close - hang up.
    portOptions.c_cflag &= ~PARENB;  // Disable parity.

    // portOptions.c_iflag |= BRKINT;
    portOptions.c_iflag &= ~IGNBRK;             // Disable ignoring break.
    portOptions.c_iflag &= ~IGNCR;              // Disable ignoring CR;
    portOptions.c_iflag &= ~(IGNPAR | PARMRK);  // Disable parity checks.
                                                // portOptions.c_iflag |= IGNPAR;
    portOptions.c_iflag &= ~(INLCR | ICRNL);    // Disable translating NL <-> CR.
    portOptions.c_iflag &= ~INPCK;              // Disable parity checking.
    portOptions.c_iflag &= ~ISTRIP;             // Disable stripping 8th bit.
    portOptions.c_iflag &= ~(IXON | IXOFF);     // disable XON/XOFF flow control

    portOptions.c_lflag &= ~ECHO;    // Disable echoing characters.
    portOptions.c_lflag &= ~ECHONL;  // ??
    portOptions.c_lflag &= ~ICANON;  // Disable canonical mode - line by line.
    portOptions.c_lflag &= ~IEXTEN;  // Disable input processing
    portOptions.c_lflag &= ~ISIG;    // Disable generating signals.
    portOptions.c_lflag &= ~NOFLSH;  // Disable flushing on SIGINT.

    portOptions.c_oflag &= ~OFILL;            // Disable fill characters.
    portOptions.c_oflag &= ~(ONLCR | OCRNL);  // Disable translating NL <-> CR.
    portOptions.c_oflag &= ~OPOST;            // Disable output processing.

    portOptions.c_cc[VKILL] = 8;
    portOptions.c_cc[VMIN] = vmin_;
    portOptions.c_cc[VTIME] = vtime_;

    if (cfsetispeed(&portOptions, B38400) < 0) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::openPort] Unable to set terminal speed "
            "(cfsetispeed)");
        throw new TRoboClawException(
            "[RoboClaw::openPort] Unable to set terminal speed "
            "(cfsetispeed)");
    }

    if (cfsetospeed(&portOptions, B38400) < 0) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::openPort] Unable to set terminal speed "
            "(cfsetospeed)");
        throw new TRoboClawException(
            "[RoboClaw::openPort] Unable to set terminal speed "
            "(cfsetospeed)");
    }

    // Now that we've populated our options structure, let's push it back to the
    // system.
    if (tcsetattr(device_port_, TCSANOW, &portOptions) < 0) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::openPort] Unable to set terminal options "
            "(tcsetattr)");
        throw new TRoboClawException(
            "[RoboClaw::openPort] Unable to set terminal options "
            "(tcsetattr)");
    }
}

void RoboClaw::closePort() {
    close(device_port_);
}

void RoboClaw::restartPort() {
  close(device_port_);
  usleep(200000);
  openPort(device_name_);
}


uint8_t RoboClaw::readByteWithTimeout() {
  struct pollfd ufd[1];
  ufd[0].fd = device_port_;
  ufd[0].events = POLLIN;

  int retval = poll(ufd, 1, 11);
  if (retval < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout] Poll failed (%d) %s",
                      errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout] Read error");
  } else if (retval == 0) {
    std::stringstream ev;
    ev << "[RoboClaw::readByteWithTimeout] TIMEOUT revents: " << std::hex
       << ufd[0].revents;
    RCUTILS_LOG_ERROR(ev.str().c_str());
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout] TIMEOUT");
  } else if (ufd[0].revents & POLLERR) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout] Error on socket");
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout] Error on socket");
  } else if (ufd[0].revents & POLLIN) {
    unsigned char buffer[1];
    ssize_t bytesRead = ::read(device_port_, buffer, sizeof(buffer));
    if (bytesRead != 1) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::readByteWithTimeout] Failed to read 1 byte, read: "
          "%d",
          (int)bytesRead);
      throw TRoboClawException(
          "[RoboClaw::readByteWithTimeout] Failed to read 1 byte");
    }

    return buffer[0];
  } else {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout] Unhandled case");
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout] Unhandled case");
  }

  return 0;
}

void RoboClaw::doM1M2AccelSpeed(uint32_t accel_m1_quad_pulses_per_second,
                                     int32_t m1_quad_pulses_per_second,
                                uint32_t accel_m2_quad_pulses_per_second,
                                     int32_t m2_quad_pulses_per_second) {
  writeN(true, 18, portAddress_, 40,
         SetDWORDval(accel_m1_quad_pulses_per_second),
         SetDWORDval(m1_quad_pulses_per_second),
         SetDWORDval(accel_m2_quad_pulses_per_second),
         SetDWORDval(m2_quad_pulses_per_second)
  );
}

void RoboClaw::updateCrc(uint16_t &crc, uint8_t data) {
  crc = crc ^ ((uint16_t)data << 8);
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
}

void RoboClaw::writeByte(uint8_t byte) {
  ssize_t result = ::write(device_port_, &byte, 1);
  // RCUTILS_LOG_INFO("--> write: 0x%02X, result: %ld", byte, result); //####
  // fprintf(stderr, "--> write: 0x%02X, result: %ld", byte, result); //####
  if (result != 1) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::writeByte] Unable to write one byte, result: %d, "
        "errno: %d)",
        (int)result, errno);
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::writeByte] Unable to write one byte");
  }
}

void RoboClaw::writeN(bool sendCRC, uint8_t cnt, ...) {
  uint16_t crc = 0;
  va_list marker;
  va_start(marker, cnt);

  int origFlags = fcntl(device_port_, F_GETFL, 0);
  fcntl(device_port_, F_SETFL, origFlags & ~O_NONBLOCK);

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t byte = va_arg(marker, int);
    writeByte(byte);
    updateCrc(crc, byte);
  }

  va_end(marker);

  if (sendCRC) {
    writeByte(crc >> 8);
    writeByte(crc);

    uint8_t response = readByteWithTimeout();
    if (response != 0xFF) {
      RCUTILS_LOG_ERROR("[RoboClaw::writeN] Invalid ACK response");
      throw new TRoboClawException("[RoboClaw::writeN] Invalid ACK response");
    }
  }

  fcntl(device_port_, F_SETFL, origFlags);
}


void RoboClaw::stop() {
  int retry;

  for (retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      writeN(true, 6, portAddress_, 34, 0, 0, 0, 0);
      RCUTILS_LOG_INFO("[RoboClaw::stop] Stop requested");  //#####
      return;
    } catch (TRoboClawException *e) {
      RCUTILS_LOG_ERROR("[RoboClaw::stop] Exception: %s, retry number: %d",
                        e->what(), retry);
      restartPort();
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::stop] Uncaught exception !!!");
    }
  }

  RCUTILS_LOG_ERROR("[RoboClaw::stop] RETRY COUNT EXCEEDED");
  throw new TRoboClawException("[RoboClaw::stop] RETRY COUNT EXCEEDED");
}