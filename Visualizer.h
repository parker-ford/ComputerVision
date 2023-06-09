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

const string MESH_FILE_PATH = "D:/Unity/HDRP - Point Cloud/Assets/model_greedy_triangle.obj";


// generate_greedy_triangulation_mesh - Uses the Point Cloud Library to generate a greedy triangulation reconstruciton based on the point cloud generated by the SfM module
// precondition: Point cloud contains 3d points
// postcondition: An OBJ file is written to disk containing the surface reconstruction of the point cloud
void generate_greedy_triangulation_mesh(vector<cv::Point3f> points);
