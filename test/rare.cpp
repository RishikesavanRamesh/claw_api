#include <gtest/gtest.h>
#include "claw_api/claw_api.hpp"
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <fstream>  // For error logging
#include <random>

#define GTEST_COUT std::cerr << "[          ] [ INFO ]"

class RoboClawTest : public ::testing::Test {
public:
    RoboClaw roboClaw;
    const uint8_t address = 0x80;
    std::atomic<bool> testFailed{false};
    std::ofstream logFile; // Log file for recording errors

    // SetUp is called before each test
    void SetUp() override {
        try {
            roboClaw.openPort("/dev/DEV-ROBOCLAW");
            logFile.open("/workspace/error_log.txt", std::ios::out | std::ios::app);  // Open log file for appending errors
            if (!logFile.is_open()) {
                GTEST_FAIL() << "Failed to open log file!";
            }
        } catch (const std::runtime_error& e) {
            GTEST_FAIL() << "Failed to open port: " << e.what();
        }
    }

    // TearDown is called after each test
    void TearDown() override {
        // Send a stop command to RoboClaw
        roboClaw.Stop(); // Make sure to stop the motor

        // Close the port after each test
        roboClaw.closePort();
        if (logFile.is_open()) {
            logFile.close();
        }
    }

    void logError(const std::string& message) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        logFile << std::ctime(&time) << " - " << message << std::endl;
    }

    bool driveContinuously() {
        std::random_device rd;
        std::mt19937 gen(rd());

        // Define the range for the first random number (25000 to 60000)
        std::uniform_int_distribution<> dist1(35000, 70000);
        
        
        // Define the range for the second random number (3000 to 8000)
        std::uniform_int_distribution<> dist2(500, 800);

        try {
            // Continuously drive M1 and M2 with signed speed and acceleration
            while (!testFailed) {

                // Try driving with a fixed speed and acceleration
                if (!roboClaw.doM1M2AccelSpeed( dist1(gen), 5000, dist1(gen), 5000)) {
                    logError("Error during DriveM1M2WithSignedSpeedIndividualAcceleration (M1 Forward)");
                    testFailed = true;
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(dist2(gen)));  // Simulate a short pause between commands

                if (!roboClaw.DriveM1M2WithSignedSpeedIndividualAcceleration(address, dist1(gen), -5000, dist1(gen), -5000)) {
                    logError("Error during DriveM1M2WithSignedSpeedIndividualAcceleration (M1 Backward)");
                    testFailed = true;
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(dist2(gen)));  // Simulate a short pause between commands
            }
        } catch (const std::runtime_error& e) {
            // Handle any runtime errors (e.g., serial communication issues)
            logError("Runtime Error: " + std::string(e.what()));
            testFailed = true;
            return false;
        }
        return true;
    }
};

// Continuous test running until error occurs in serial communication
TEST_F(RoboClawTest, driveContinuouslyUntilError) {
    GTEST_COUT << "Starting continuous drive test. This will run until a communication error occurs..." << std::endl;

    // Start the continuous drive test in the background
    ASSERT_TRUE(driveContinuously());  // If the drive continues without errors, this will pass.
    
    // Check if test failed
    ASSERT_FALSE(testFailed) << "Test failed due to communication error.";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
