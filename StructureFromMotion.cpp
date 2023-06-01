#include "StructureFromMotion.h"


StructureFromMotion::StructureFromMotion() {

}

void StructureFromMotion::read_images() {
	vector<string> image_paths = get_image_file_paths(IMAGE_DIR);
	for (size_t i = 0; i < image_paths.size(); i++) {
		View v;
		Mat image = imread(image_paths[i]);
		resize(image, image, image.size() / DOWNSCALE_FACTOR);
		v.image = image;
		scene_views.push_back(v);

		if (PRINT_STATUS) {
			cout << "Read Image: " << image_paths[i] << endl;
		}
	}

	match_matrix = vector<vector<Matches>>(image_paths.size(), vector<Matches>(image_paths.size()));

}



void StructureFromMotion::detect_keypoints_and_descriptors() {
	Ptr<FeatureDetector> detector = AKAZE::create();
	for (size_t i = 0; i < scene_views.size(); i++) {
		vector<KeyPoint> keypoints;
		Mat descriptors;
		detector->detectAndCompute(scene_views[i].image, noArray(), keypoints, descriptors);
		scene_views[i].keypoints = keypoints;
		scene_views[i].descriptors = descriptors;

		if (PRINT_STATUS) {
			cout << "Found Keypoints and Detectors for Image: " << i << endl;
		}
	}
}

void StructureFromMotion::pairwise_match_views() {
	for (size_t i = 0; i < scene_views.size() - 1; i++) {
		for (size_t j = i + 1; j < scene_views.size(); j++) {
			View view1 = scene_views[i];
			View view2 = scene_views[j];

			vector<Matches> knn_matches = get_knn_matches(view1.descriptors, view2.descriptors);
			vector<Matches> knn_matches_reciprocal = get_knn_matches(view2.descriptors, view1.descriptors);
			
			Matches ratio_filtered_matches = ratio_filter_matches(knn_matches);
			Matches ratio_filtered_matches_reciprocal = ratio_filter_matches(knn_matches_reciprocal);
			
			Matches reciprocity_filtered_matches = reciprocity_filter_matches(ratio_filtered_matches, ratio_filtered_matches_reciprocal);
			
			Matches epipolar_filtered_matches = epipolar_filter_matches(reciprocity_filtered_matches, view1.keypoints, view2.keypoints);

			match_matrix[i][j] = epipolar_filtered_matches;

			if (PRINT_STATUS) {
				cout << "Found Matches for Pair: " << i << " , " << j << endl;
			}
		}
	}
}

void StructureFromMotion::initialize_intrinsics()
{
	//TODO: May need to change principal_point
	double cx = 1521.316552582157;
	double cy = 2024.8360801192564;
	//double cx = scene_views[0].image.size().width / 2;
	//double cy = scene_views[0].image.size().height / 2;
	double fx = 3031.53030720967;
	double fy = 3033.7600300469517;
	Mat K = (Mat_<double>(3, 3) << (fx / DOWNSCALE_FACTOR), 0, (cx / DOWNSCALE_FACTOR), 0, (fy / DOWNSCALE_FACTOR), (cy / DOWNSCALE_FACTOR), 0, 0, 1);
	double focal_length = K.at<double>(0, 0);
	Point2d principal_point(K.at<double>(0, 2), K.at<double>(1, 2));

	intrinsics.K = K;
	intrinsics.focal_length = focal_length;
	intrinsics.principal_point = principal_point;

}

void StructureFromMotion::intitialize_structure()
{
	//TODO: Find optimal starting views
	View view1 = scene_views[0];
	View view2 = scene_views[1];

	Correspondence correspondence = matches_to_correspondence(match_matrix[0][1], view1, view2);

	//Find Essential Mat
	Mat status;
	Mat E = findEssentialMat(correspondence.left_points, correspondence.right_points, intrinsics.K, RANSAC, 0.999, 1.0, status);

	//Recover Pose
	Mat_<double> R, t;
	recoverPose(E, correspondence.left_points, correspondence.right_points, R, t, intrinsics.focal_length, intrinsics.principal_point, status);

	Matx34f P_left = Matx34f::eye();
	Matx34f P_right = Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
		R(1, 0), R(1, 1), R(1, 2), t(1),
		R(2, 0), R(2, 1), R(2, 2), t(2));

	//Undistort points
	Mat norm_L_points;
	Mat norm_R_points;
	//TODO: Figure out distance coefficient. Should replace Mat()
	undistortPoints(correspondence.left_points, norm_L_points, intrinsics.K, Mat());
	undistortPoints(correspondence.right_points, norm_R_points, intrinsics.K, Mat());

	//Convert Points to Homogeneous
	Mat leftPts_h, rightPts_h;
	convertPointsToHomogeneous(correspondence.left_points, leftPts_h);
	convertPointsToHomogeneous(correspondence.right_points, rightPts_h);

	//Triangulate points
	Mat points4D;
	//triangulatePoints(Pleft, Pright, leftPts_h, rightPts_h, points4D);
	triangulatePoints(P_left, P_right, norm_L_points, norm_R_points, points4D);

	//Convert to 3D
	Mat points3D;
	convertPointsFromHomogeneous(points4D.t(), points3D);
	points3D = points3D.reshape(1);

	cout << "points3D size: " << points3D.size() << endl;
	cout << "keypoints size " << correspondence.left_points.size() << endl;

	for (size_t i = 0; i < points3D.rows; i++) {
		ContributingView contributing_view_left, contributing_view_right;
		contributing_view_left.view_index = 0;
		contributing_view_left.view_keypoint_index = (int)i;

		contributing_view_right.view_index = 1;
		contributing_view_left.view_index = (int)i;

		PointCloudPoint p;
		p.contributing_views.push_back(contributing_view_left);
		p.contributing_views.push_back(contributing_view_left);
	}


	//Write camera file
	ofstream camera_file;
	camera_file.open("D:/Unity/HDRP - Point Cloud/Assets/cameras.csv");
	if (camera_file.is_open()) {
		write_camera_data(P_left, camera_file);
		write_camera_data(P_right, camera_file);
	}
	else {
		cout << "Unable to open camera file\n";
	}
	camera_file.close();

	//Write points file
	ofstream point_file;
	point_file.open("D:/Unity/HDRP - Point Cloud/Assets/points.csv");
	if (point_file.is_open()) {
		write_point_data(points3D, point_file);
	}
	else {
		cout << "Unable to open points file\n";
	}
	point_file.close();

}

SceneViews StructureFromMotion::get_scene_views()
{
	return this->scene_views;
}

MatchMatrix StructureFromMotion::get_match_matrix()
{
	return match_matrix;
}

Correspondence StructureFromMotion::matches_to_correspondence(Matches matches, View view_left, View view_right)
{
	vector<Point2f>leftPts, rightPts;
	for (size_t i = 0; i < matches.size(); i++) {
		leftPts.push_back(view_left.keypoints[matches[i].queryIdx].pt);
		rightPts.push_back(view_right.keypoints[matches[i].trainIdx].pt);
	}
	Correspondence correspondence;
	correspondence.left_points = leftPts;
	correspondence.right_points = rightPts;

	return correspondence;
}





