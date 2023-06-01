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
};

struct Correspondence {
	vector<Point2f> left_points;
	vector<Point2f> right_points;
};

struct ContributingView {
	int view_index;
	int view_keypoint_index;
};

struct PointCloudPoint {
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
typedef vector<PointCloudPoint> PointCloud;

const string IMAGE_DIR = "plant2";
const int DOWNSCALE_FACTOR = 2;
const bool PRINT_STATUS = true;

class StructureFromMotion {
	public:
		StructureFromMotion();

		void read_images();
		void detect_keypoints_and_descriptors();
		void pairwise_match_views();
		void initialize_intrinsics();
		void intitialize_structure();
		
		SceneViews get_scene_views();
		MatchMatrix get_match_matrix();

	private:
		SceneViews scene_views;
		MatchMatrix match_matrix;
		PointCloud point_cloud;
		Intrinsics intrinsics;

		Correspondence matches_to_correspondence(Matches matches, View view_left, View view_right);
};