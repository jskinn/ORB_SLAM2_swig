%module orbslam2
%{
#define SWIG_FILE_WITH_INIT
#include <list>
#include <vector>
#include "include/Tracking.h"
#include "include/KeyFrame.h"
#include "include/MapPoint.h"
#include "include/System.h"
#include "include/VocabularyBuilder.h"
#include "InterfaceTypes.h"
//using namespace ORB_SLAM2;
%}

//%include std_common.i
%include <std_string.i>
%include <exception.i>
//%include std_vector.i
%include <std_list.i>

// Auto-document, see http://www.swig.org/Doc4.0/Python.html#Python_nn67
%feature("autodoc", "2");

// handle numpy
%include numpy.i
%init %{
  import_array();
%}

// Templates for list types (used in Tracking)
//%template(float_list) std::list<float>;
//%template(bool_list) std::list<bool>;

// Handle exceptions in the constructor. API could be better.
// see https://stackoverflow.com/questions/1394484/how-do-i-propagate-c-exceptions-to-python-in-a-swig-wrapper-library
%exception System { 
    try {
        $action
    } catch (...) {
        SWIG_exception(SWIG_RuntimeError, "Exception, see std::err for details");
    }
}

// Typemapping for numpy arrays to char pointer arrays
%apply (unsigned char* IN_ARRAY2, int DIM1, int DIM2) {
  (unsigned char* image1, int rows1, int cols1),
  (unsigned char* image2, int rows2, int cols2)
}
%apply (float* IN_ARRAY2, int DIM1, int DIM2) { (float* depthImage, int depthRows, int depthCols) }

// Ignore some of the methods with complex return types
%ignore ORB_SLAM2::System::TrackMonocular;
%ignore ORB_SLAM2::System::TrackStereo;
%ignore ORB_SLAM2::System::TrackRGBD;
%ignore ORB_SLAM2::System::GetKeyFrames;
%ignore ORB_SLAM2::System::GetTracker;
%ignore ORB_SLAM2::System::GetTrackedMapPoints;
%ignore ORB_SLAM2::System::GetTrackedKeyPointsUn;

// Rename other methods to python naming conventions
%rename ActivateLocalizationMode activate_localization_mode;
%rename DeactivateLocalizationMode deactivate_localization_mode;
%rename GetTrackingState get_tracking_state;
%rename MapChanged map_changed;
%rename Reset reset;
%rename Shutdown shutdown;
%rename SaveKeyFrameTrajectoryTUM save_key_frame_trajectory_TUM;
%rename SaveTrajectoryKITTI save_trajectory_KITTI;
%rename SaveTrajectoryTUM save_trajectory_TUM;

// Ignore most of the tracking object
%ignore ORB_SLAM2::Tracking::Tracking;
%ignore ORB_SLAM2::Tracking::GrabImageStereo;
%ignore ORB_SLAM2::Tracking::GrabImageRGBD;
%ignore ORB_SLAM2::Tracking::GrabImageMonocular;
%ignore ORB_SLAM2::Tracking::SetLocalMapper;
%ignore ORB_SLAM2::Tracking::SetLoopClosing;
%ignore ORB_SLAM2::Tracking::SetViewer;
%ignore ORB_SLAM2::Tracking::ChangeCalibration;
%ignore ORB_SLAM2::Tracking::InformOnlyTracking;
%ignore ORB_SLAM2::Tracking::Reset;
%ignore ORB_SLAM2::Tracking::mCurrentFrame;
%ignore ORB_SLAM2::Tracking::mImGray;
%ignore ORB_SLAM2::Tracking::mvIniLastMatches;
%ignore ORB_SLAM2::Tracking::mvIniMatches;
%ignore ORB_SLAM2::Tracking::mvbPrevMatched;
%ignore ORB_SLAM2::Tracking::mvIniP3D;
%ignore ORB_SLAM2::Tracking::mInitialFrame;
%ignore ORB_SLAM2::Tracking::mlRelativeFramePoses;
%ignore ORB_SLAM2::Tracking::mlpReferences;
%ignore ORB_SLAM2::Tracking::mlFrameTimes;
%ignore ORB_SLAM2::Tracking::mlbLost;

// Mark some tracking properties read-only
%immutable ORB_SLAM2::Tracking::mState;
%immutable ORB_SLAM2::Tracking::mLastProcessedState;
%immutable ORB_SLAM2::Tracking::mSensor;
//%immutable ORB_SLAM2::Tracking::mlpReferences;
//%immutable ORB_SLAM2::Tracking::mlFrameTimes;
//%immutable ORB_SLAM2::Tracking::mlbLost;
%immutable ORB_SLAM2::Tracking::mbOnlyTracking;

// Tweak VocabularyBuilder methods to python conventions
%ignore ORB_SLAM2::VocabularyBuilder::addImage;
%rename buildVocabulary build_vocabulary;

// Template types for the interface types
%template(point_list) std::list<Point3D>;
%template(pose_list) std::list<PoseEstimate>;
%include "InterfaceTypes.h"

// Wrap the ORBSLAM headers
%include "include/System.h"
%include "include/Tracking.h"
%include "include/VocabularyBuilder.h"

// Add some extra methods to ORBSLAM system, to extract data from it.
%extend ORB_SLAM2::System {

  // Allow process_mono to be called with greyscale numpy images
  bool process_mono(unsigned char* image1, int rows1, int cols1, const double& timestamp)
  {
    cv::Mat im(rows1, cols1, cv::DataType<unsigned char>::type, image1);
    cv::Mat pose = $self->TrackMonocular(im, timestamp);
    return !pose.empty();
  }

  // Allow process_stereo to be called with greyscale numpy images
  bool process_stereo(unsigned char* image1, int rows1, int cols1, unsigned char* image2, int rows2, int cols2, const double& timestamp)
  {
    cv::Mat leftIm(rows1, cols1, cv::DataType<unsigned char>::type, image1);
    cv::Mat rightIm(rows2, cols2, cv::DataType<unsigned char>::type, image2);
    cv::Mat pose = $self->TrackStereo(leftIm, rightIm, timestamp);
    return !pose.empty();
  }

  // Allow process_rgbd to be called with greyscale numpy images and float depthmaps
  bool process_rgbd(unsigned char* image1, int rows1, int cols1, float* depthImage, int depthRows, int depthCols, const double& timestamp)
  {
    cv::Mat im(rows1, cols1, CV_8U, image1);
    cv::Mat depthIm(depthRows, depthCols, cv::DataType<float>::type, depthImage);
    cv::Mat pose = $self->TrackRGBD(im, depthIm, timestamp);
    return !pose.empty();
  }

  // add a function to get the number of detected features from the current frame
  unsigned int get_num_features() const
  {
    return $self->GetTracker()->mCurrentFrame.mvKeys.size();
  }

  // Get the number of feature matches for the current frame
  unsigned int get_num_matches() const
  {
    // This code is based on the display code in FrameDrawer.cc, with a little extra safety logic to check the length of the vectors.
    ORB_SLAM2::Tracking* pTracker = $self->GetTracker();
    unsigned int matches = 0;
    unsigned int num = pTracker->mCurrentFrame.mvKeys.size();
    if (pTracker->mCurrentFrame.mvpMapPoints.size() < num)
    {
      num = pTracker->mCurrentFrame.mvpMapPoints.size();
    }
    if (pTracker->mCurrentFrame.mvbOutlier.size() < num)
    {
      num = pTracker->mCurrentFrame.mvbOutlier.size();
    }
    for(unsigned int i = 0; i < num; ++i)
    {
      ORB_SLAM2::MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
      if(pMP && !pTracker->mCurrentFrame.mvbOutlier[i] && pMP->Observations() > 0)
      {
        ++matches;
      }
    }
    return matches;
  }

  // Get all the tracked map points, as 3-vectors.
  std::list<Point3D> get_tracked_map_points()
  {
    std::list<Point3D> map_points;
    vector<ORB_SLAM2::MapPoint*> Mps = $self->GetTrackedMapPoints();
    for (size_t i = 0; i < Mps.size(); ++i)
    {
      cv::Mat wp = Mps[i]->GetWorldPos();
      Point3D loc;
      loc.x = wp.at<float>(0, 0);
      loc.y = wp.at<float>(1, 0);
      loc.z = wp.at<float>(2, 0);
      map_points.push_back(loc);
    }
    return map_points;
  }

  // Get the estimated poses of keyframes only
  std::list<PoseEstimate> get_keyframe_points() const
  {
    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = $self->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::list<PoseEstimate> points;
    for (size_t i=0; i < vpKFs.size(); ++i)
    {
      PoseEstimate pose;
      ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
      if(pKF->isBad())
      {
        continue;
      }
      cv::Mat R = pKF->GetRotation().t();
      cv::Mat t = pKF->GetCameraCenter();
      pose.timestamp = pKF->mTimeStamp;
      pose.r00 = R.at<float>(0, 0);
      pose.r01 = R.at<float>(0, 1);
      pose.r02 = R.at<float>(0, 2);
      pose.r10 = R.at<float>(1, 0);
      pose.r11 = R.at<float>(1, 1);
      pose.r12 = R.at<float>(1, 2);
      pose.r20 = R.at<float>(2, 0);
      pose.r21 = R.at<float>(2, 1);
      pose.r22 = R.at<float>(2, 2);
      pose.t0 = t.at<float>(0);
      pose.t1 = t.at<float>(1);
      pose.t2 = t.at<float>(2);
      points.push_back(pose);
    }
    return points;
  }

  // Get the estimates for all frames, not just key frames
  std::list<PoseEstimate> get_trajectory_points() const
  {
    // This is copied from the ORB_SLAM2 System.SaveTrajectoryKITTI function, with some changes to output a list.
    std::vector<ORB_SLAM2::KeyFrame*> vpKFs = $self->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // Of course, if we have no keyframes, then just use the identity matrix.
    cv::Mat Two = cv::Mat::eye(4,4,CV_32F);
    if (vpKFs.size() > 0) {
      cv::Mat Two = vpKFs[0]->GetPoseInverse();
    }

    std::list<PoseEstimate> trajectory;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM2::KeyFrame*>::iterator lRit = $self->GetTracker()->mlpReferences.begin();
    std::list<double>::iterator lT = $self->GetTracker()->mlFrameTimes.begin();
    for(std::list<cv::Mat>::iterator lit=$self->GetTracker()->mlRelativeFramePoses.begin(), lend=$self->GetTracker()->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
      ORB_SLAM2::KeyFrame* pKF = *lRit;

      cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

      while(pKF != NULL && pKF->isBad())
      {
        ORB_SLAM2::KeyFrame* pKFParent;

        // std::cout << "bad parent" << std::endl;
        Trw = Trw*pKF->mTcp;
        pKFParent = pKF->GetParent();
        if (pKFParent == pKF)
        {
          // We've found a frame that is it's own parent, presumably a root or something. Break out
          break;
        } else {
          pKF = pKFParent;
        }
      }
      if (pKF != NULL && !pKF->isBad())
      {
        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        PoseEstimate pose;
        pose.timestamp = *lT;
        pose.r00 = Rwc.at<float>(0, 0);
        pose.r01 = Rwc.at<float>(0, 1);
        pose.r02 = Rwc.at<float>(0, 2);
        pose.r10 = Rwc.at<float>(1, 0);
        pose.r11 = Rwc.at<float>(1, 1);
        pose.r12 = Rwc.at<float>(1, 2);
        pose.r20 = Rwc.at<float>(2, 0);
        pose.r21 = Rwc.at<float>(2, 1);
        pose.r22 = Rwc.at<float>(2, 2);
        pose.t0 = twc.at<float>(0);
        pose.t1 = twc.at<float>(1);
        pose.t2 = twc.at<float>(2);
        trajectory.push_back(pose);
      }
    }
    return trajectory;
  }
}

%extend ORB_SLAM2::VocabularyBuilder {
  void add_image(unsigned char* image1, int rows1, int cols1)
  {
    cv::Mat im(rows1, cols1, cv::DataType<unsigned char>::type, image1);
    $self->addImage(im);
  }
}
