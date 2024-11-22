#include "claw_api/claw_api.hpp"

#include <iostream>
#include <cstdint>
#include <cstdarg>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <string>


RoboClaw::RoboClaw() : crc(0) {}

void RoboClaw::openPort(const std::string& device_name) {
    port_ = device_name;
    serialStream_.Open(port_);
    if (!serialStream_.IsOpen()) {
        throw std::runtime_error("Unable to open USB port");
    }
    serialStream_.SetBaudRate(LibSerial::BaudRate::BAUD_38400);
    serialStream_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serialStream_.SetParity(LibSerial::Parity::PARITY_NONE);
    serialStream_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void RoboClaw::closePort() {
    if (serialStream_.IsOpen()) {
        serialStream_.Close();
    }
}

bool RoboClaw::isPortOpen() {
    return serialStream_.IsOpen();
}

bool RoboClaw::ForwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_M1FORWARD, speed);
}

bool RoboClaw::BackwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_M1BACKWARD, speed);
}

bool RoboClaw::ForwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_M2FORWARD, speed);
}

bool RoboClaw::BackwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_M2BACKWARD, speed);
}

bool RoboClaw::DriveM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_DRIVEM1, speed);
}

bool RoboClaw::DriveM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_DRIVEM2, speed);
}

bool RoboClaw::DriveForward(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_DRIVEFORWARD, speed);
}

bool RoboClaw::DriveBackward(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_DRIVEBACKWARD, speed);
}

bool RoboClaw::TurnRight(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_TURNRIGHT, speed);
}

bool RoboClaw::TurnLeft(uint8_t address, uint8_t speed) {
    return write_n(3, address, CLAW_API_TURNLEFT, speed);
}

bool RoboClaw::TurnLeftOrRight(uint8_t address, uint8_t speed) {
    if (speed > 127) speed = 127; 
    return write_n(3, address, CLAW_API_TURNLEFTORRIGHT, speed);
}

bool RoboClaw::DriveForwardOrBackward(uint8_t address, uint8_t speed) {
    if (speed > 127) speed = 127; 
    return write_n(3, address, CLAW_API_DRIVEFORWARDORBACKWARD, speed);
}

bool RoboClaw::ReadFirmwareVersion(uint8_t address, std::string& firmwareVersion) {
    uint8_t data;
    uint8_t tries = CLAW_API_MAXRETRY;
    char response[50];
    size_t index = 0;

    do {
        crc_clear(); // Clear the CRC before sending the command
        write(address);
        crc_update(address);
        write(CLAW_API_READ_FIRMWARE_VERSION);
        crc_update(CLAW_API_READ_FIRMWARE_VERSION);

        while (index < sizeof(response) - 2) { // Leave space for line feed and null
            data = read(1000); // Timeout in milliseconds
            if (data == -1) {
                std::cerr << "Read timeout" << std::endl;
                break;
            }

            response[index++] = data;
            crc_update(data); // Update CRC with the byte read

            if (data == 0) { // If we hit the null terminator
                uint16_t receivedCrc = (static_cast<uint16_t>(read(1000)) << 8) | read(1000);
                firmwareVersion.assign(response, index); // Convert buffer to string
                return crc_get() == receivedCrc; // Check if the CRC matches
            }
        }

        index = 0; // Reset index for the next try
    } while (tries--);

    return false;
}

bool RoboClaw::DriveM1WithSignedSpeed(uint8_t address, int32_t speed){
        // Create data groups for the address, command, speed, and CRC
    std::vector<std::vector<uint8_t>> dataGroups;
    std::vector<std::pair<int64_t, size_t>> args = {
        {address, 8}, // Address (1 byte = 8 bits)
        {35, 8},      // Command (1 byte = 8 bits)
        {speed, 32}   // Speed (4 bytes = 32 bits, signed)
    };
    // Convert each argument to its corresponding byte representation
    for (const auto& arg : args) {
        dataGroups.push_back(to_bytes(arg.first, arg.second));  // Pass to to_bytes
    }

    return write_n2(dataGroups);  // Pass the data groups to write_n
}


bool RoboClaw::DriveM2WithSignedSpeed(uint8_t address, int32_t speed){
        // Create data groups for the address, command, speed, and CRC
    std::vector<std::vector<uint8_t>> dataGroups;
    std::vector<std::pair<int64_t, size_t>> args = {
        {address, 8}, // Address (1 byte = 8 bits)
        {36, 8},      // Command (1 byte = 8 bits)
        {speed, 32}   // Speed (4 bytes = 32 bits, signed)
    };
    // Convert each argument to its corresponding byte representation
    for (const auto& arg : args) {
        dataGroups.push_back(to_bytes(arg.first, arg.second));  // Pass to to_bytes
    }

    return write_n2(dataGroups);  // Pass the data groups to write_n
}

bool RoboClaw::DriveM1M2WithSignedSpeed(uint8_t address, int32_t speedM1, int32_t speedM2)
{

    // Create data groups for the address, command, speed, and CRC
    std::vector<std::vector<uint8_t>> dataGroups;
    std::vector<std::pair<int64_t, size_t>> args = {
        {address, 8}, // Address (1 byte = 8 bits)
        {37, 8},      // Command (1 byte = 8 bits)
        {speedM1, 32},   // Speed (4 bytes = 32 bits, signed)
        {speedM2, 32}   // Speed (4 bytes = 32 bits, signed)
    };
    // Convert each argument to its corresponding byte representation
    for (const auto& arg : args) {
        dataGroups.push_back(to_bytes(arg.first, arg.second));  // Pass to to_bytes
    }

    return write_n2(dataGroups);  // Pass the data groups to write_n

}


bool RoboClaw::DriveM1M2WithSignedSpeedIndividualAcceleration(uint8_t address, int32_t acclM1, int32_t speedM1, int32_t acclM2, int32_t speedM2)
{
    std::vector<std::vector<uint8_t>> dataGroups;
    std::vector<std::pair<int64_t, size_t>> args = {
        {address, 8}, // Address (1 byte = 8 bits)
        {50, 8},      // Command (1 byte = 8 bits)

        {acclM1, 32},   // Acceleration M1 (4 bytes = 32 bits, signed)
        {speedM1, 32},   // Speed M1 (4 bytes = 32 bits, signed)
        {acclM2, 32},   // Acceleration M2 (4 bytes = 32 bits, signed)
        {speedM2, 32}   // Speed M2 (4 bytes = 32 bits, signed)
    };
    // Convert each argument to its corresponding byte representation
    for (const auto& arg : args) {
        dataGroups.push_back(to_bytes(arg.first, arg.second));  // Pass to to_bytes
    }

    return write_n2(dataGroups);  // Pass the data groups to write_n
}

bool RoboClaw::SetSerialTimeout(uint8_t address, int32_t value)
{
    std::vector<std::vector<uint8_t>> dataGroups;
    std::vector<std::pair<int64_t, size_t>> args = {
        {address, 8}, // Address (1 byte = 8 bits)
        {14, 8},      // Command (1 byte = 8 bits)

        {value, 8}  // Acceleration M1 (4 bytes = 32 bits, signed)
    };
    // Convert each argument to its corresponding byte representation
    for (const auto& arg : args) {
        dataGroups.push_back(to_bytes(arg.first, arg.second));  // Pass to to_bytes
    }

    return write_n2(dataGroups);  // Pass the data groups to write_n
}



bool RoboClaw::Stop() {


    ForwardM1(0x80,0);
    BackwardM2(0x80,0);
    return TurnRight(0x80,0);

}

RoboClaw::~RoboClaw() {
    closePort();
}



std::vector<uint8_t> RoboClaw::to_bytes(int64_t value, size_t bit_size)
{
    // Calculate the number of bytes needed for the given bit size
    size_t byte_size = (bit_size + 7) / 8;  // Round up to the nearest byte

    // Create a vector to hold the bytes
    std::vector<uint8_t> bytes(byte_size, 0);

    // Handle signed integers using two's complement for negative values
    for (size_t i = 0; i < byte_size; ++i) {
        bytes[byte_size - 1 - i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
    }

    return bytes;
}



bool RoboClaw::write_n(uint8_t cnt, ...) {


    uint8_t trys = CLAW_API_MAXRETRY;
    do {
        crc_clear();
        va_list marker;
        va_start(marker, cnt);

        for (uint8_t index = 0; index < cnt; index++) {
            uint8_t data = static_cast<uint8_t>(va_arg(marker, int));
            crc_update(data);
            write(data);
        }
        va_end(marker);

        uint16_t crcValue = crc_get();
        write(crcValue >> 8);
        write(crcValue);

        if (read(CLAW_API_TIMEOUT) == 0xFF) {
            return true;
        }
    } while (trys--);
    return false;
}

bool RoboClaw::write_n2(const std::vector<std::vector<uint8_t>>& dataGroups)
{
    uint8_t tries = CLAW_API_MAXRETRY;
    do {
        crc_clear();
        
        // Loop through each group of data (address, command, speed, etc.)
        for (const auto& group : dataGroups) {
            for (uint8_t byte : group) {
                crc_update(byte);  // Update CRC for each byte
                write(byte);       // Write each byte to the serial port
            }
        }

        // After writing all groups, calculate and send CRC
        uint16_t crcValue = crc_get();
        write(crcValue >> 8);
        write(crcValue);

        // Read the response and check if it's 0xFF (success)
        if (read(10) == 0xFF) {
            return true;
        }

    } while (tries--);
    return false;
}

void RoboClaw::crc_update(uint8_t data) {
    crc ^= (static_cast<uint16_t>(data) << 8);
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
}

void RoboClaw::crc_clear() {
    crc = 0;
}

uint16_t RoboClaw::crc_get() {
    return crc;
}

size_t RoboClaw::write(uint8_t byte) {
    serialStream_.write(reinterpret_cast<const char*>(&byte), 1);
    return 1; // Indicate that 1 byte was written
}

uint8_t RoboClaw::read(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    uint8_t byte;

    while (true) {
        if (serialStream_.read(reinterpret_cast<char*>(&byte), 1)) {
            return byte; // Return the byte if read successfully
        }

        // Check for timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= timeout_ms) {
            throw std::runtime_error("Read timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Small delay to prevent busy waiting
    }
}
