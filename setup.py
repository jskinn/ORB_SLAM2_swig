#from distutils.core import setup, Extension
from setuptools import setup, Extension
from distutils.sysconfig import get_config_vars
import platform
import numpy
import os
import re
from pathlib import Path


def build_search_paths():
    """
    Get a set of plausible search paths to look for things.
    TODO: Find a way of choosing workable directories on Windows/Mac
    """
    paths = []
    # If we're in an anaconda environment, search it for libraries/include files
    if 'CONDA_PREFIX' in os.environ:
        paths.append(Path(os.environ['CONDA_PREFIX']))
    # Add default system header paths on linux
    if platform.system() == 'Linux':
        paths.append(Path('/usr/local/include'))
        paths.append(Path('/usr/include'))
        paths.append(Path('/usr/lib'))
    return paths


def find_path(search_paths, indicator_file):
    """
    Search for a path where a certain relative path exists.
    Use to find plausible include or library directories,
    e.g. `find_path([..], 'libopencv_core.so')`
    will try and find a directory containing libopencv_core.so, which can be added as a library dir.
    """
    include_dir = None
    for to_search in search_paths:
        if include_dir is None:
            found_path = next(to_search.rglob(indicator_file), None)
            if found_path is not None:
                # Found a directory containing the given relative path
                include_dir = found_path.parents[len(indicator_file.split('/')) - 1] # Strip off the relative path we were searching for
    return include_dir


search_paths = build_search_paths()
eigen_include = find_path(search_paths, 'signature_of_eigen3_matrix_library')
opencv_include = find_path(search_paths, 'opencv2/core/core.hpp')
opencv_lib = find_path(search_paths, 'libopencv_core.so')

# The full list of OpenCV libraries to link to
opencv_libs = [
    "opencv_calib3d",
    "opencv_core",
    "opencv_dnn",
    "opencv_features2d",
    "opencv_flann",
    "opencv_highgui",
    "opencv_imgcodecs",
    "opencv_imgproc",
    "opencv_ml",
    "opencv_objdetect",
    "opencv_photo",
    "opencv_shape",
    "opencv_stitching",
    "opencv_superres",
    "opencv_video",
    "opencv_videoio",
    "opencv_videostab",
    "opencv_aruco",
    "opencv_bgsegm",
    "opencv_bioinspired",
    "opencv_ccalib",
    "opencv_datasets",
    "opencv_dnn_objdetect",
    "opencv_dpm",
    "opencv_face",
    "opencv_freetype",
    "opencv_fuzzy",
    "opencv_hdf",
    "opencv_hfs",
    "opencv_img_hash",
    "opencv_line_descriptor",
    "opencv_optflow",
    "opencv_phase_unwrapping",
    "opencv_plot",
    "opencv_reg",
    "opencv_rgbd",
    "opencv_saliency",
    "opencv_stereo",
    "opencv_structured_light",
    "opencv_surface_matching",
    "opencv_text",
    "opencv_tracking",
    "opencv_xfeatures2d",
    "opencv_ximgproc",
    "opencv_xobjdetect",
    "opencv_xphoto"
]

include_dirs = {
    numpy.get_include(),
    'include',
    '.',
    'Thirdparty/DBoW2/DBoW2',
    'Thirdparty/DBoW2/DUtils',
    'Thirdparty/g2o/g2o/core',
    'Thirdparty/g2o/g2o/types',
    'Thirdparty/g2o/g2o/stuff',
    str(eigen_include),
    str(opencv_include)
}
library_dirs = {
    str(opencv_lib)
}

extra_compile_args = ['-march=native', '-std=c++11']
if platform.system() == 'Windows':
    # Add exception support on windows
    extra_compile_args.append('/EHsc')

setup(
    name="orbslam2",
    version="0.1.0",
    py_modules=["orbslam2"],
    zip_safe=False,
    ext_modules=[
        Extension(
	    "_orbslam2",
            sources=[
                # DBoW2
                "Thirdparty/DBoW2/DBoW2/BowVector.cpp",
                "Thirdparty/DBoW2/DBoW2/FORB.cpp",
                "Thirdparty/DBoW2/DBoW2/FeatureVector.cpp",
                "Thirdparty/DBoW2/DBoW2/ScoringObject.cpp",
                "Thirdparty/DBoW2/DUtils/Random.cpp",
                "Thirdparty/DBoW2/DUtils/Timestamp.cpp",

                # g2o
                # g2o types
                'Thirdparty/g2o/g2o/types/types_sba.cpp',
                'Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp',
                'Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp',

                # g2o core
                'Thirdparty/g2o/g2o/core/hyper_graph_action.cpp',
                'Thirdparty/g2o/g2o/core/hyper_graph.cpp',
                'Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp',
                'Thirdparty/g2o/g2o/core/matrix_structure.cpp',
                'Thirdparty/g2o/g2o/core/batch_stats.cpp',
                'Thirdparty/g2o/g2o/core/parameter.cpp',    
                'Thirdparty/g2o/g2o/core/cache.cpp',
                'Thirdparty/g2o/g2o/core/optimizable_graph.cpp',
                'Thirdparty/g2o/g2o/core/solver.cpp',
                'Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp',
                'Thirdparty/g2o/g2o/core/estimate_propagator.cpp',
                'Thirdparty/g2o/g2o/core/factory.cpp',
                'Thirdparty/g2o/g2o/core/sparse_optimizer.cpp',
                'Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp',
                'Thirdparty/g2o/g2o/core/parameter_container.cpp',
                'Thirdparty/g2o/g2o/core/optimization_algorithm.cpp',
                'Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp',
                'Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp',
                'Thirdparty/g2o/g2o/core/jacobian_workspace.cpp',
                'Thirdparty/g2o/g2o/core/robust_kernel.cpp',
                'Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp',
                'Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp',

                # g2o stuff
                'Thirdparty/g2o/g2o/stuff/timeutil.cpp',
                'Thirdparty/g2o/g2o/stuff/os_specific.c',
                'Thirdparty/g2o/g2o/stuff/string_tools.cpp',
                'Thirdparty/g2o/g2o/stuff/property.cpp',

                # ORBSLAM
                "src/System.cc",
                "src/Tracking.cc",
                "src/LocalMapping.cc",
                "src/LoopClosing.cc",
                "src/ORBextractor.cc",
                "src/ORBmatcher.cc",
                "src/FrameDrawer.cc",
                "src/Converter.cc",
                "src/MapPoint.cc",
                "src/KeyFrame.cc",
                "src/Map.cc",
                "src/MapDrawer.cc",
                "src/Optimizer.cc",
                "src/PnPsolver.cc",
                "src/Frame.cc",
                "src/KeyFrameDatabase.cc",
                "src/Sim3Solver.cc",
                "src/Initializer.cc",
                "src/Viewer.cc",

                # Swig
                "orbslam2.i"
            ],
            language="c++",
            swig_opts=['-c++','-threads', '-py3'],
            extra_compile_args=extra_compile_args,
            include_dirs=list(include_dirs),
            libraries=[] + opencv_libs,
            library_dirs=list(library_dirs)
        )
    ]
)
