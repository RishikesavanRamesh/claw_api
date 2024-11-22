#include <gtest/gtest.h>
#include "claw_api/claw_api.hpp"
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>

#define GTEST_COUT std::cerr << "[          ] [ INFO ]"

class RoboClawTest : public ::testing::Test {
public:
    RoboClaw roboClaw;
    const uint8_t address = 0x80;
    std::atomic<bool> userConfirmed{false};
    std::atomic<bool> testFailed{false};
    // SetUp is called before each test
    void SetUp() override {
        try {
            roboClaw.openPort("/dev/DEV-ROBOCLAW");
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
    }


    void monitorUserInput() {
        char response;
        while (true) {
            std::cin >> response;

            if (response == 'Y' || response == 'y') {
                userConfirmed = true;
                break;
            } else if (response == 'N' || response == 'n') {
                testFailed = true;
                break;
            } else {
                std::cout << "Invalid input. Defaulting to Y." << std::endl;
                std::cout.flush(); // Flush again after output
                userConfirmed = true; // Default to confirmed
                break;
            }
        }
    }

    void waitForUserInputDuringAction(int duration) {
        std::thread inputThread(&RoboClawTest::monitorUserInput, this);
        inputThread.detach(); // Detach the thread to run independently

        auto startTime = std::chrono::steady_clock::now();

        // Wait for the specified duration
        while (std::chrono::steady_clock::now() - startTime < std::chrono::seconds(duration)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Check every 100 ms
            if (testFailed) {
                break; // Exit if test failed
            }
        }
        
        // Optionally, you can join the thread here if you want to ensure cleanup
        // Otherwise, the detached thread will continue running
    }
};

TEST_F(RoboClawTest, isForwardM1WorkingWithResponse) {
    GTEST_COUT << "Enter Y to confirm or N to fail the test (default is Y): " << std::endl;
    ASSERT_TRUE(roboClaw.ForwardM1(address, 64));
    waitForUserInputDuringAction(4); // Wait for 4 seconds for user input

    ASSERT_FALSE(testFailed); // Check if the test was marked as failed
}

TEST_F(RoboClawTest, isReadFirmwareVersionWorking) {
    std::string firmwareVersion;
    ASSERT_TRUE(roboClaw.ReadFirmwareVersion(address, firmwareVersion));
    GTEST_COUT << "Firmware version: " << firmwareVersion << std::endl;
}

TEST_F(RoboClawTest, isForwardM1Working) {
    ASSERT_TRUE(roboClaw.ForwardM1(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));

}
TEST_F(RoboClawTest, isBackwardM1Working) {
    ASSERT_TRUE(roboClaw.BackwardM1(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isForwardM2Working) {
    ASSERT_TRUE(roboClaw.ForwardM2(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isBackwardM2Working) {
    ASSERT_TRUE(roboClaw.BackwardM2(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isDriveM1Working) {
    ASSERT_TRUE(roboClaw.DriveM1(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isDriveM2Working) {
    ASSERT_TRUE(roboClaw.DriveM2(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isDriveForwardWorking) {
    ASSERT_TRUE(roboClaw.DriveForward(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isDriveBackwardWorking) {
    ASSERT_TRUE(roboClaw.DriveBackward(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isTurnRightWorking) {
    ASSERT_TRUE(roboClaw.TurnRight(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isTurnLeftWorking) {
    ASSERT_TRUE(roboClaw.TurnLeft(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isDriveForwardOrBackwardWorking) {
    ASSERT_TRUE(roboClaw.DriveForwardOrBackward(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
TEST_F(RoboClawTest, isTurnLeftOrRightWorking) {
    ASSERT_TRUE(roboClaw.TurnLeftOrRight(address, 64));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}


TEST_F(RoboClawTest, isAccleratingTest1) {
    ASSERT_TRUE(roboClaw.DriveM1M2WithSignedSpeedIndividualAcceleration(address, 500000, 50000, 500000, 50000));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_TRUE(roboClaw.DriveM1M2WithSignedSpeedIndividualAcceleration(address, 500000, -50000, 500000, -50000));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

TEST_F(RoboClawTest, isAcceleratingTest2) {
    ASSERT_TRUE(roboClaw.DriveM1M2WithSignedSpeedIndividualAcceleration(address, 100000, -50000, 100000, -50000));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_TRUE(roboClaw.DriveM1M2WithSignedSpeedIndividualAcceleration(address, 100000, 50000, 100000, 50000));
    std::this_thread::sleep_for(std::chrono::seconds(2));
}


// TEST_F(RoboClawTest, isReadEncoderSpeedM1Working) {
//     std::string firmwareVersion;
//     ASSERT_TRUE(roboClaw.ReadFirmwareVersion(address, firmwareVersion));
//     GTEST_COUT << "Firmware version: " << firmwareVersion << std::endl;
// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
