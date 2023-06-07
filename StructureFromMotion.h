#pragma once
#include <opencv2/opencv.hpp>
#include <fstream>
#include "DirectoryReader.h"
#include "ImageMatcher.h"
#include "FileWriter.h"

using namespace cv;
using namespace std;

struct View {
	Mat image;
	Mat descriptors;
	vector<KeyPoint> keypoints;
	int index;
	Matx34f projection_matrix;
};

struct Correspondence_2D2D {
	vector<Point2f> left_points;
	vector<Point2f> right_points;
};

struct Correspondence_2D3D {
	vector<Point2f> points_2D;
	vector<Point3f> points_3D;
};

struct ContributingView {
	int view_index;
	int view_keypoint_index;
};

struct PointCloudPoint {
	Point3f point_3D;
	vector<ContributingView> contributing_views;
};

struct Intrinsics {
	Mat K;
	double focal_length;
	Point2d principal_point;
};

 
typedef vector<View> SceneViews;
typedef vector<DMatch> Matches;
typedef vector<vector<Matches>> MatchMatrix;
typedef vector<PointCloudPoint> SFMPointCloud;
typedef map<int, Correspondence_2D3D> Corresondence_2D3D_map;

const string IMAGE_DIR = "table";
const int DOWNSCALE_FACTOR = 2;
const bool PRINT_STATUS = true;
const float RANSAC_THRESHOLD = 1;



class StructureFromMotion {
	public:
		StructureFromMotion();

		void read_images();
		void detect_keypoints_and_descriptors();
		void pairwise_match_views();
		void initialize_intrinsics();
		void intitialize_structure();
		 void increment_views();
		
		
		SceneViews get_scene_views();
		MatchMatrix get_match_matrix();
		vector<Point3f> point_cloud_to_vector();

	private:
		SceneViews scene_views;
		MatchMatrix match_matrix;
		SFMPointCloud point_cloud;
		Intrinsics intrinsics;
		Corresondence_2D3D_map correspondence_2D3D_map;

		//For Debugging
		ofstream camera_file;
		ofstream points_file;

		Correspondence_2D2D matches_to_correspondence(Matches matches, View view_left, View view_right);
		pair<View, View> find_baseline_pair();
		void triangulate_points_and_add_to_cloud(View view1, View view2);
		void set_2D3D_correspondence(View new_view);
		void add_view_to_cloud(View view);
		
};