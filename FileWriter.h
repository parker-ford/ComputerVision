// FILE WRITER
// Author: Parker Ford
// Helper functions to write camera and point cloud data to csv files
// FILE WRITER
// Author: Parker Ford
// Helper function to print SfM data to csv files

#pragma once
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;
using namespace cv;

// write_point_data - writes each point as a line in a csv file. Format: x,y,z,r,g,b
// precondition: 3D points have been generated, an ostream to a csv file for points has been created
// postcondition: each 3D point is written to the file
void write_point_data(Mat points, ofstream& file);


// write_camera_data - writes camera extrinsic parameters as a csv file. Format: R(0,0),R(0,1),R(0,2),t(0),R(1,0),R(1,1),R(1,2),t(1),R(2,0),R(2,1),R(2,2),t(2),
// precondition: A cameras extrinsic parameters have been generated, an ostream to a csv file for cameras has been created
// postcondition: camera extrinsics have been written to a file
void write_camera_data(Matx34f P, ofstream& file);

