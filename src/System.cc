/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

System::System(const eSensor sensor):
        mbIsRunning(false),
        mSensor(sensor),
        mbReset(false),
        mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{

}

System::~System()
{
    if (mbIsRunning)
    {
        // This is for safety. Really, call Shutdown before this.
        Shutdown();
    }
}

bool System::StartUp(const std::string &strVocFile, const std::string &strSettingsFile, const bool bUseViewer)
{
    if (mbIsRunning) {  // System is already running, don't start it up again
        return true;
    }
    
#ifdef DEBUG_MESSAGE
    // Output welcome message
    std::cout << std::endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << std::endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
    "This is free software, and you are welcome to redistribute it" << std::endl <<
    "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

    std::cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR) {
        std::cout << "Monocular" << std::endl;
    } else if(mSensor==STEREO) {
        std::cout << "Stereo" << std::endl;
    } else if(mSensor==RGBD) {
        std::cout << "RGB-D" << std::endl;
    }
#endif //DEBUG_MESSAGE

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
#ifdef DEBUG_MESSAGE
       std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
#endif //DEBUG_MESSAGE
       return false;
    }

#ifdef DEBUG_MESSAGE
    //Load ORB Vocabulary
    std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;
#endif //DEBUG_MESSAGE

    mpVocabulary = std::make_shared<ORBVocabulary>();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
#ifdef DEBUG_MESSAGE
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Falied to open at: " << strVocFile << std::endl;
#endif //DEBUG_MESSAGE
        mpVocabulary = nullptr;
        return false;
    }
#ifdef DEBUG_MESSAGE
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;
#endif //DEBUG_MESSAGE

    //Create KeyFrame Database
    mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase>(mpVocabulary);

    //Create the Map
    mpMap = std::make_shared<Map>();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = std::make_shared<FrameDrawer>(mpMap);
    mpMapDrawer = std::make_shared<MapDrawer>(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = std::make_shared<Tracking>(shared_from_this(), mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = std::make_shared<LocalMapping>(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = std::unique_ptr<std::thread>(new std::thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper));

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = std::make_shared<LoopClosing>(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = std::unique_ptr<std::thread>(new std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser));

    //Initialize the Viewer thread and launch
    mpViewer = std::make_shared<Viewer>(shared_from_this(), mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
    if(bUseViewer) {
        mptViewer = std::unique_ptr<std::thread>(new std::thread(&Viewer::Run, mpViewer));
    }

    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
    
    mbIsRunning = true;
    return mbIsRunning;
}

bool System::IsRunning()
{
    return mbIsRunning;
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if (!mbIsRunning) {
        return cv::Mat();
    }
    if(mSensor!=STEREO)
    {
#ifdef DEBUG_MESSAGE
        std::cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << std::endl;
#endif //DEBUG_MESSAGE
        return cv::Mat();
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
#ifdef DEBUG_MESSAGE
        std::cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << std::endl;
#endif //DEBUG_MESSAGE
        return cv::Mat();
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if (!mbIsRunning) {
        return cv::Mat();
    }
    if(mSensor!=MONOCULAR)
    {
#ifdef DEBUG_MESSAGE
        std::cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << std::endl;
#endif //DEBUG_MESSAGE
        return cv::Mat();
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageMonocular(im,timestamp);
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    if (!mbIsRunning) {
        return;
    }
    
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
    
    // Blocking wait for threads to stop.
    if (mptLocalMapping && mptLocalMapping->joinable()) {
        mptLocalMapping->join();
    }
    if (mptLoopClosing && mptLoopClosing->joinable()) {
        mptLoopClosing->join();
    }
    if (mptViewer && mptViewer->joinable()) {
        mptViewer->join();
    }

    //pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    //pangolin::Quit();
    
    mbIsRunning = false;
}

void System::SaveTrajectoryTUM(const std::string &filename)
{
#ifdef DEBUG_MESSAGE
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
#endif //DEBUG_MESSAGE

    vector<std::shared_ptr<KeyFrame>> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<std::shared_ptr<KeyFrame>>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        std::shared_ptr<KeyFrame> pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
    }
    f.close();
#ifdef DEBUG_MESSAGE
    std::cout << std::endl << "trajectory saved!" << std::endl;
#endif //DEBUG_MESSAGE
}


void System::SaveKeyFrameTrajectoryTUM(const std::string &filename)
{
#ifdef DEBUG_MESSAGE
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;
#endif //DEBUG_MESSAGE

    vector<std::shared_ptr<KeyFrame>> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        std::shared_ptr<KeyFrame> pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;

    }

    f.close();
#ifdef DEBUG_MESSAGE
    std::cout << std::endl << "trajectory saved!" << std::endl;
#endif //DEBUG_MESSAGE
}

void System::SaveTrajectoryKITTI(const std::string &filename)
{
#ifdef DEBUG_MESSAGE
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
#endif //DEBUG_MESSAGE

    vector<std::shared_ptr<KeyFrame>> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<std::shared_ptr<KeyFrame>>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        std::shared_ptr<KeyFrame> pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  std::cout << "bad parent" << std::endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << std::endl;
    }
    f.close();
#ifdef DEBUG_MESSAGE
    std::cout << std::endl << "trajectory saved!" << std::endl;
#endif //DEBUG_MESSAGE
}

vector<std::shared_ptr<KeyFrame>> System::GetKeyFrames() const
{
    return mpMap->GetAllKeyFrames();
}

const std::shared_ptr<Tracking> System::GetTracking() const
{
    return mpTracker;
}

} //namespace ORB_SLAM
