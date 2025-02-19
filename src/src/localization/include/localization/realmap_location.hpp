#ifndef CAMERA_PROJECTION_HPP
#define CAMERA_PROJECTION_HPP
#pragma once
#include <cmath>
using namespace std;
// Define camera parameters
const double camera_fov = 110.0; // Field of view in degrees
const double camera_width = 640.0; // Image width in pixels
const double camera_height = 360.0; // Image height in pixels

// Function declarations
void get_projection(double depth, double angle_x, double angle_y, double& x, double& y);
void get_realmap_loc(double depth, double angle_x, double angle_y, double& x, double& y, double x_global, double y_global, double theta);
void realmap_loc(double& x, double& y, const double &rel_x, const double &rel_y, const double &x_global, const double &y_global, const double &the);

#endif // CAMERA_PROJECTION_HPP
