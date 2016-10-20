#include <string>
#include <memory>
#include "System.h"
#include "gtest/gtest.h"

namespace TestingConstants
{
    const std::string ValidVocabluary = "Vocabulary/ORBvoc.txt";
    const std::string InvalidVocabluary = "notafile";
    const std::string ValidSettings = "Examples/Monocular/TUM1.yaml";
    const std::string InvalidSettings = "notafile";
}

TEST(SystemTest, testAllocateAndDeallocate)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
}

TEST(SystemTest, testStartUpAndShutdownWithoutViewer)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::ValidSettings, false);
    subject->Shutdown();
}

TEST(SystemTest, testStartUpAndShutdownWithViewer)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::ValidSettings, true);
    subject->Shutdown();
}

TEST(SystemTest, testDestructorAutomaticShutdown)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::ValidSettings, false);
}

TEST(SystemTest, testIsRunning)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    EXPECT_FALSE(subject->IsRunning());
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::ValidSettings, false);
    EXPECT_TRUE(subject->IsRunning());
    subject->Shutdown();
    EXPECT_FALSE(subject->IsRunning());
}

TEST(SystemTest, testRepeatedStartUpAndShutdown)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::ValidSettings, false);
    subject->Shutdown();
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::ValidSettings, false);
    subject->Shutdown();
}

TEST(SystemTest, testCannotFindVocab)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    EXPECT_FALSE(subject->IsRunning());
    subject->StartUp(TestingConstants::InvalidVocabluary, TestingConstants::ValidSettings, false);
    EXPECT_FALSE(subject->IsRunning());
}

TEST(SystemTest, testCannotFindSettings)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    EXPECT_FALSE(subject->IsRunning());
    subject->StartUp(TestingConstants::ValidVocabluary, TestingConstants::InvalidSettings, false);
    EXPECT_FALSE(subject->IsRunning());
}

/*int main(int argc, char** argv)
{
    {
        //testing::InitGoogleTest(&argc, argv);
        //return RUN_ALL_TESTS();
        std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    
    }
    
    {
        std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
        subject->StartUp(argv[1], argv[2], false);
        subject->Shutdown();
    }
    
    {
        std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
        subject->StartUp(argv[1], argv[2], true);
        subject->Shutdown();
    }
}*/
