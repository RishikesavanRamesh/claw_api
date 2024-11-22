#include <gtest/gtest.h>
#include "claw_api/claw_api.hpp"
#include <thread>
#include <chrono>

RoboClaw roboClaw;




TEST(claw_api_test, OpenConnection) {

    // Attempt to open the port and expect no exceptions
    ASSERT_NO_THROW(roboClaw.openPort("/dev/DEV-ROBOCLAW"));
    
}


TEST(claw_api_test, IsPortOpen) {
    
    // If the openPort function has a way to check if the port is open
    ASSERT_TRUE(roboClaw.isPortOpen()); // Assuming you have such a method
    // Optionally, you can add further checks, such as testing send/receive
    // For example, send a command and check the response
    // ASSERT_NO_THROW(roboClaw.sendValues()); // Uncomment if sendValues is defined
}

TEST(claw_api_test, IsMovingForward){
    ASSERT_TRUE(roboClaw.ForwardM1(0x80,30));
    ASSERT_TRUE(roboClaw.ForwardM2(0x80,30));
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

TEST(claw_api_test, IsMovingBackward){
    ASSERT_TRUE(roboClaw.BackwardM1(0x80,30));
    ASSERT_TRUE(roboClaw.BackwardM2(0x80,30));
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

TEST(claw_api_test, IsTurningRight){
    ASSERT_TRUE(roboClaw.BackwardM1(0x80,30));
    ASSERT_TRUE(roboClaw.ForwardM2(0x80,30));
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ASSERT_TRUE(roboClaw.ForwardM1(0x80,0));
    ASSERT_TRUE(roboClaw.BackwardM2(0x80,0));
    std::this_thread::sleep_for(std::chrono::seconds(1));

}


TEST(claw_api_test, IsTurningLeft){
    ASSERT_TRUE(roboClaw.ForwardM1(0x80,30));
    ASSERT_TRUE(roboClaw.BackwardM2(0x80,30));
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

TEST(claw_api_test, IsStoping){
    ASSERT_TRUE(roboClaw.ForwardM1(0x80,0));
    ASSERT_TRUE(roboClaw.BackwardM2(0x80,0));
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
