#include <memory>
#include "System.h"
#include "gtest/gtest.h"

TEST(SystemTest, testAllocateAndDeallocate)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
}

TEST(SystemTest, testStartUpAndShutdown)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    subject->StartUp("Vocabluary/ORBvoc.txt", "Examples/Monocular/TUM1.yaml", false);
    subject->Shutdown();
}

TEST(SystemTest, testDestructorAutomaticShutdown)
{
    std::shared_ptr<ORB_SLAM2::System> subject = std::make_shared<ORB_SLAM2::System>(ORB_SLAM2::System::MONOCULAR);
    subject->StartUp("Vocabluary/ORBvoc.txt", "Examples/Monocular/TUM1.yaml", false);
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
