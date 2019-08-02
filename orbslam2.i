%module orbslam2
%{
#define SWIG_FILE_WITH_INIT
#include "include/System.h"
#include "include/KeyFrame.h"
#include "include/MapPoint.h"
using namespace ORB_SLAM2;
%}

%include std_string.i
%include exception.i
//%include std_vector.i

// Auto-document, see http://www.swig.org/Doc4.0/Python.html#Python_nn67
%feature("autodoc", "1");

// handle numpy
%include numpy.i
%init %{
  import_array();
%}

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

%include "include/System.h"

// apply the numpy typemap to enable a more comforable call with 2D images
%extend ORB_SLAM2::System {

  bool process_mono(unsigned char* image1, int rows1, int cols1, const double& timestamp)
  {
    cv::Mat im(rows1, cols1, cv::DataType<unsigned char>::type, image1);
    cv::Mat pose = $self->TrackMonocular(im, timestamp);
    return !pose.empty();
  }

  bool process_stereo(unsigned char* image1, int rows1, int cols1, unsigned char* image2, int rows2, int cols2, const double& timestamp)
  {
    cv::Mat leftIm(rows1, cols1, cv::DataType<unsigned char>::type, image1);
    cv::Mat rightIm(rows2, cols2, cv::DataType<unsigned char>::type, image2);
    cv::Mat pose = $self->TrackStereo(leftIm, rightIm, timestamp);
    return !pose.empty();
  }

  bool process_rgbd(unsigned char* image1, int rows1, int cols1, float* depthImage, int depthRows, int depthCols, const double& timestamp)
  {
    cv::Mat im(rows1, cols1, CV_8U, image1);
    cv::Mat depthIm(depthRows, depthCols, cv::DataType<float>::type, depthImage);
    cv::Mat pose = $self->TrackRGBD(im, depthIm, timestamp);
    return !pose.empty();
  }
}
