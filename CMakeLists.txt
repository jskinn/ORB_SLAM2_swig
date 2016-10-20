cmake_minimum_required(VERSION 2.8.11)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# ---------------
# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# add google test to the build
add_subdirectory(tests/gtest)

set(COMMON_INCLUDE_DIRS
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

# ---------------
# Build the main ORB_SLAM library
# ---------------

# Version numbers, so that the generated .so is numbered and linked correctly.
set(ORBSLAM_VERSION_MAJOR 1)
set(ORBSLAM_VERSION_MINOR 0)
set(ORBSLAM_VERSION_PATCH 0)
set(ORBSLAM_VERSION_STRING ${ORBSLAM_VERSION_MAJOR}.${ORBSLAM_VERSION_MINOR}.${ORBSLAM_VERSION_PATCH})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)
target_include_directories(${PROJECT_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${ORBSLAM_VERSION_STRING} SOVERSION ${ORBSLAM_VERSION_MAJOR})

# Set preprocessor macros for the library based on the build type. Case insensitive.
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_UPPERCASE_BUILD_TYPE)
if(CMAKE_UPPERCASE_BUILD_TYPE MATCHES "DEBUG")
    message("Including debug print statements")
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_MESSAGE=1)
endif()

# ---------------
# Build examples
# ---------------

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_include_directories(mono_tum PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_include_directories(mono_kitti PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(mono_kitti ${PROJECT_NAME})

# ---------------
# Build unit tests
# ---------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tests/bin)
enable_testing()
set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
file(GLOB TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/tests/src/*.cpp)
add_executable(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})
target_include_directories(${PROJECT_TEST_NAME} PUBLIC ${GTEST_INCLUDE_DIRS} ${COMMON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_TEST_NAME}
    ${PROJECT_NAME}
    ${GTEST_LIBS_DIR}/libgtest.a
    ${GTEST_LIBS_DIR}/libgtest_main.a
)

# Specify the tests
add_test(NAME integration_tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND ${PROJECT_TEST_NAME})


# ---------------
# Installation instructions.
# Will install to CMAKE_INSTALL_PREFIX
# This defaults to /usr/local/, specify -DCMAKE_INSTALL_PREFIX when calling cmake to specify an alternate location
# e.g. cmake -DCMAKE_INSTALL_PREFIX="~/local"
# ---------------

install(TARGETS ${PROJECT_NAME}
    DESTINATION  lib
    COMPONENT library)
install(FILES
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    DESTINATION lib
    COMPONENT library)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/ORB_SLAM2
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/
    DESTINATION include/ORB_SLAM2/Thirdparty
    COMPONENT library
    FILES_MATCHING REGEX ".*\\.h(pp)?")
