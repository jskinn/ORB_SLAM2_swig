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
%exception System { 
    try {
        $action
    } catch (...) {
        SWIG_exception(SWIG_RuntimeError, "Exception, see std::err for details");
    }
}

// Typemapping for numpy arrays to char pointer arrays
%apply (unsigned char* IN_ARRAY2, int DIM1, int DIM2 ) {
  (unsigned char* image1, int rows1, int cols1),
  (unsigned char* image2, int rows2, int cols2),
  (unsigned char* image, int rows, int cols)
}
%apply (double* INPLACE_ARRAY2, int DIM1, int DIM2) { (double* mat, int rows, int cols) } 

// what interfaces to SWIG?
%ignore ORB_SLAM2::System::TrackMonocular;
%ignore ORB_SLAM2::System::TrackStereo;
%ignore ORB_SLAM2::System::TrackRGBD;
%ignore ORB_SLAM2::System::GetKeyFrames;
%ignore ORB_SLAM2::System::GetTracker;
%ignore ORB_SLAM2::System::GetTrackedMapPoints;
%ignore ORB_SLAM2::System::GetTrackedKeyPointsUn;
%include "include/System.h"

// apply the numpy typemap to enable a more comforable call with 2D images
//%extend ORB_SLAM2::System {

//   bool TrackStereo(unsigned char* image1, int rows1, int cols1, unsigned char* image2, int rows2, int cols2, const double& timestamp)
//   {
//     //int dims[] = {cols1, rows1, cols1};
//     //return $self->process(image1, image2, dims, replace);
//     return false;
//   }

//   bool TrackMono(unsigned char* image, int rows, int cols, const double& timestamp)
//   {
//     //int dims[] = {cols, rows, cols};
//     //return $self->process(image, dims, replace);
//     return false;
//   }
// }
