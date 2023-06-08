// STRUCTURE FROM MOTION
// Author: Parker Ford
// Class contain all code relating to the Structure from Motion step of the pipeline. Utilizes OpenCV functionality heavily

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

const string IMAGE_DIR = "dinoSparseRing";
const string PATH_TO_POINTS_CSV = "D:/Unity/HDRP - Point Cloud/Assets/points.csv";
const string PATH_TO_CAMERAS_CSV = "D:/Unity/HDRP - Point Cloud/Assets/cameras.csv";
const int DOWNSCALE_FACTOR = 1;
const bool PRINT_STATUS = true;
const float RANSAC_THRESHOLD = 1;



class StructureFromMotion {
	public:
		StructureFromMotion();

		// read_images - Initializes the SceneViews data structure by reading the images in the specified image directory
		// precondition: Images must be present in the specified IMAGE_DIR
		// postcondition: SceneViews datastructure is initialized and filled with OpenCV mats representing the input images.
		//				  The match matrix data structure is initialized.
		//				  The files that will contain the point cloud data and camera data are opened.
		void read_images();

		// detect_keypoints_and_descriptors - Finds keypoitns and descriptors for each view in the SceneViews data structure. OpenCV AKAZE feature detector is used.
		// precondition: Views in SceneViews datastructure must be initialized and have image associated with them.
		// postcondition: Each View object now contains keypoints and descriptors
		void detect_keypoints_and_descriptors();

		// pairwise_match_views - Performs robust feature matching between every pair of Views in the SceneViews datastructure
		// precondition: Each View in the Scene View data strucuture must have its image field and keypoint and descriptor fields filled
		// postcondition: match_matrix data structure is filled with the pairwise matches
		void pairwise_match_views();

		// initialize_intrinsics - Sets the intrinisic paramaters for the cameras. It is assumed that each photo is taken by the same camera so each view will have a uniform intrinsic matrix.
		//						   Currently the intrinsic matrix is hard coded for my phone camera
		// precondition: Camera used to take photos has been calibrated
		// postcondition: Intrinsics for the cameras have been set
		void initialize_intrinsics();

		// intitialize_structure - Given a pair of Views, it will generate the baseline strucutre for the point cloud. This is accomplished via the findEssentialMat and recoverPose OpenCV functions
		// precondition: Two views have been initialized with images, keypoints and descriptors, and have been determined to be the baseline pair of views. Matches between these to views must have been found
		// postcondition: The extrinsic matrices for the two views is found and set. The matching keypoints of the two images have been triangulated and added to the point cloud data structure.
		void intitialize_baseline_structure();

		// increment_views - Iterates over all views other than those used to create the baseline structure. Adds each view to the point cloud one by one
		// precondition: All views need image, keypoints, and descriptors set. Match matrix must also be set.
		// postcondition: All views are added into the point cloud, finishing the structure from motion pipeline.
		void increment_views();
		
		// get_scene_views - Accessor method for the SceneViews data structure
		// precondition: SceneViews data structure must be intitialized
		// postcondition: SceneViews data structure is returned
		SceneViews get_scene_views();

		// get_match_matrix - Accessor method for the match matrix data structure
		// precondition: match matrix data structure must be initialized
		// postcondition: match matrix data strucuture is returned
		MatchMatrix get_match_matrix();

		// point_cloud_to_vector - returns the point cloud data structure as vector of type Point3f
		// precondition: point cloud data structure must be initialized 
		// postcondition: point cloud data structure is returned as vector of type Point3f
		vector<Point3f> point_cloud_to_vector();

	private:
		SceneViews scene_views;
		MatchMatrix match_matrix;
		SFMPointCloud point_cloud;
		Intrinsics intrinsics;
		Corresondence_2D3D_map correspondence_2D3D_map;
		ofstream camera_file;
		ofstream points_file;

		// matches_to_correspondence - For a given set of matches, returns a pair of vectors representing corresponding 2D points in each view
		// precondition: Match matrix must be filled as well as the keypoints for the left and right views
		// postcondition: A pair of Point2f vectors is returned
		Correspondence_2D2D matches_to_correspondence(Matches matches, View view_left, View view_right);

		// find_baseline_pair - Finds the pair of views to be used as the baseline views for the baseline structure. Currently just returns the first two views in SceneViews data structure.
		// precondition: The first two views of SceneViews data structure must be initialized
		// postcondition: The first twoa views are returned as a pair
		pair<View, View> find_baseline_pair();

		// triangulate_points_and_add_to_cloud - Given two views, triangulate their keypoints into 3D space based on the two views extrinsic matrices
		// precondition: Match matrix between the two the views must be set. Point cloud data structure must be initialized
		// postcondition: Triangulated points are added to the point cloud
		void triangulate_points_and_add_to_cloud(View view1, View view2);

		// set_2D3D_correspondence - For a given view, check every point currently in the point cloud. Based on the keypoitns tied to each point in the cloud, 
		//							 and the keypoints tied to each view, a 2D to 3D corrispondence is made.
		// precondition: The view must be initialized and have its keypoitns set, match matrix must be set, and the point cloud must be initialized and contain atleast the baseline structure.
		// postcondition: A pair of vectors representing the 2D to 3D corrispondence is set in the 2D3DCorrispondence data structure
		void set_2D3D_correspondence(View new_view);

		// add_view_to_cloud - Based on a views 2D to 3D corrispondence, the OpenCV function solvePnPRansac is used to to generate the views extrinsic matrix. The keypoints for this view are then triangulated and added to the cloud
		// precondition: The 2D to 3D corrispondence for this view must have been calculated
		// postcondition: The keypoints of this view are triangulated and added to the point cloud
		void add_view_to_cloud(View view);
		
};