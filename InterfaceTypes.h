/**
 * Helper structs to assist the bindings.
 * Lets me send points and small matricies back to the target language without using
 * cv::Mat or numpy
 */

#ifndef INTERFACE_TYPES_H
#define INTERFACE_TYPES_H

// A simple point in 3D space
struct Point3D
{
  float x;
  float y;
  float z;
};

// A combination of a 3d pose and a timestamp. many of these make a trajectory.
// Axes are z forward, y down, x right, as is CV convention.
struct PoseEstimate
{
  float timestamp;
  // The rotation matrix, as 9 values, r{row}{column}
  float r00;
  float r01;
  float r02;
  float r10;
  float r11;
  float r12;
  float r20;
  float r21;
  float r22;
  // The translation, as 3 values
  float t0;
  float t1;
  float t2;
};

#endif /* INTERFACE_TYPES_H */
