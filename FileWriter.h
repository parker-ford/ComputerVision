#pragma once
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;
using namespace cv;

void write_point_data(Mat points, ofstream& file);

void write_camera_data(Matx34f P, ofstream& file);

