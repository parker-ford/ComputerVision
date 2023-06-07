#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/poisson.h>


#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace pcl;


void load_point_cloud(vector<cv::Point3f> points);

void test_visualizer();