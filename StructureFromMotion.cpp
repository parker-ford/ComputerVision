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
		v.index = (int)i;
		scene_views.push_back(v);

		if (PRINT_STATUS) {
			cout << "Read Image: " << image_paths[i] << endl;
		}
	}

	//Initialize match matrix size
	match_matrix = vector<vector<Matches>>(image_paths.size(), vector<Matches>(image_paths.size()));

	//Open output files
	camera_file.open(PATH_TO_CAMERAS_CSV);
	points_file.open(PATH_TO_POINTS_CSV);
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

			vector<Matches> knn_matches = get_2nn_matches(view1.descriptors, view2.descriptors);
			vector<Matches> knn_matches_reciprocal = get_2nn_matches(view2.descriptors, view1.descriptors);
			
			Matches ratio_filtered_matches = ratio_filter_matches(knn_matches);
			Matches ratio_filtered_matches_reciprocal = ratio_filter_matches(knn_matches_reciprocal);
			
			Matches reciprocity_filtered_matches = reciprocity_filter_matches(ratio_filtered_matches, ratio_filtered_matches_reciprocal);
			
			//Matches epipolar_filtered_matches = epipolar_filter_matches(reciprocity_filtered_matches, view1.keypoints, view2.keypoints);
			Matches epipolar_filtered_matches = reciprocity_filtered_matches;

			match_matrix[i][j] = epipolar_filtered_matches;
		
			if (PRINT_STATUS) {
				cout << "Found Matches for Pair: " << i << " , " << j << endl;
			}
		}
	}
}

void StructureFromMotion::initialize_intrinsics() {
	//Iphone Camera
	double cx = 1521.316552582157;
	double cy = 2024.8360801192564;
	double fx = 3031.53030720967;
	double fy = 3033.7600300469517;
	

	//Dino Sparse Ring
	/*
	double cx = 316.730000;
	double cy = 200.550000;
	double fx = 3310.400000;
	double fy = 3325.500000;
	*/

	Mat K = (Mat_<double>(3, 3) << (fx / DOWNSCALE_FACTOR), 0, (cx / DOWNSCALE_FACTOR), 0, (fy / DOWNSCALE_FACTOR), (cy / DOWNSCALE_FACTOR), 0, 0, 1);
	double focal_length = K.at<double>(0, 0);
	Point2d principal_point(K.at<double>(0, 2), K.at<double>(1, 2));

	intrinsics.K = K;
	intrinsics.focal_length = focal_length;
	intrinsics.principal_point = principal_point;

}

void StructureFromMotion::intitialize_baseline_structure() {
	pair<View, View> baseline_pair = find_baseline_pair();
	View view1 = baseline_pair.first;
	View view2 = baseline_pair.second;

	Correspondence_2D2D correspondence = matches_to_correspondence(match_matrix[view1.index][view2.index], view1, view2);

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


	scene_views[view1.index].projection_matrix = P_left;
	scene_views[view2.index].projection_matrix = P_right;

	//Write camera file
	if (camera_file.is_open()) {
		write_camera_data(P_left, camera_file);
		write_camera_data(P_right, camera_file);
	}
	else {
		cout << "Unable to open camera file\n";
	}

	triangulate_points_and_add_to_cloud(scene_views[view1.index], scene_views[view2.index]);
}

void StructureFromMotion::increment_views() {
	for (size_t i = 2; i < scene_views.size(); i++) {
		set_2D3D_correspondence(scene_views[i]);
		add_view_to_cloud(scene_views[i]);
	}
}

SceneViews StructureFromMotion::get_scene_views() {
	return this->scene_views;
}

MatchMatrix StructureFromMotion::get_match_matrix() {
	return match_matrix;
}

Correspondence_2D2D StructureFromMotion::matches_to_correspondence(Matches matches, View view_left, View view_right) {
	vector<Point2f>leftPts, rightPts;
	for (size_t i = 0; i < matches.size(); i++) {
		leftPts.push_back(view_left.keypoints[matches[i].queryIdx].pt);
		rightPts.push_back(view_right.keypoints[matches[i].trainIdx].pt);
	}
	Correspondence_2D2D correspondence;
	correspondence.left_points = leftPts;
	correspondence.right_points = rightPts;

	return correspondence;
}

pair<View, View> StructureFromMotion::find_baseline_pair() {
	//TODO: Placeholder - future implementation should have a system for finding optimal baseline pair
	return pair<View, View>(scene_views[0], scene_views[1]);
}

void StructureFromMotion::triangulate_points_and_add_to_cloud(View view1, View view2) {

	//TODO: write check to ensure theat view1 and view2 projection matrix is set

	Correspondence_2D2D correspondence = matches_to_correspondence(match_matrix[view1.index][view2.index], view1, view2);

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
	triangulatePoints(view1.projection_matrix, view2.projection_matrix, norm_L_points, norm_R_points, points4D);

	//Convert to 3D
	Mat points3D;
	convertPointsFromHomogeneous(points4D.t(), points3D);

	points3D = points3D.reshape(1);

	for (size_t i = 0; i < points3D.rows; i++) {
		Point3f p_3D(points3D.at<float>(i, 0), points3D.at<float>(i, 1), points3D.at<float>(i, 2));

		ContributingView contributing_view_left, contributing_view_right;
		contributing_view_left.view_index = view1.index;
		contributing_view_left.view_keypoint_index = match_matrix[view1.index][view2.index][i].queryIdx;

		contributing_view_right.view_index = view2.index;
		contributing_view_right.view_keypoint_index = match_matrix[view1.index][view2.index][i].trainIdx;

		PointCloudPoint p;
		p.point_3D = p_3D;
		p.contributing_views.push_back(contributing_view_left);
		p.contributing_views.push_back(contributing_view_right);

		point_cloud.push_back(p);
	}

	if (points_file.is_open()) {
		write_point_data(points3D, points_file);
	}
	else {
		cout << "Unable to open points file\n";
	}

}

void StructureFromMotion::set_2D3D_correspondence(View new_view)
{
	Correspondence_2D3D correspondence_2D3D;

	//For every Point in the point cloud
	for (size_t i = 0; i < point_cloud.size(); i++) {
		PointCloudPoint point = point_cloud[i];
		//Search points contributing views
		for (size_t j = 0; j < point.contributing_views.size(); j++) {
			ContributingView contributing_view = point.contributing_views[j];
			int found_index_in_new_view = -1;
			for (size_t k = 0; k < match_matrix[contributing_view.view_index][new_view.index].size(); k++) {
				DMatch match = match_matrix[contributing_view.view_index][new_view.index][k];
				if (match.queryIdx == contributing_view.view_keypoint_index) {
					found_index_in_new_view = match.trainIdx;
				}

				if (found_index_in_new_view > 0) {
					correspondence_2D3D.points_2D.push_back(new_view.keypoints[found_index_in_new_view].pt);
					correspondence_2D3D.points_3D.push_back(point.point_3D);
					break;
				}
			}
		}
	}

	correspondence_2D3D_map[new_view.index] = correspondence_2D3D;
}

void StructureFromMotion::add_view_to_cloud(View view) {
	Correspondence_2D3D correspondence_2D3D = correspondence_2D3D_map[view.index];

	if (correspondence_2D3D.points_3D.size() < 4) {
		cout << "not enough points in correspondence for view " << view.index << endl;
		return;
	}	

	double minVal, maxVal;
	minMaxIdx(correspondence_2D3D.points_2D, &minVal, &maxVal);

	Mat rvec, t;
	Mat inliers;
	//TODO: Figure out RANSAC_THRESHOLD and exchange Mat() with distance coeficciant / distortion coefficiient
	solvePnPRansac(correspondence_2D3D.points_3D, correspondence_2D3D.points_2D, intrinsics.K, Mat(), rvec, t, false, 100, 0.006 * maxVal, 0.99, inliers);

	const float numInliers = (float)countNonZero(inliers);
	const float numPoints = (float)correspondence_2D3D.points_2D.size();
	const float inlierRatio = numInliers / numPoints;
	cout << "Inlier ratio: " << inlierRatio << endl;

	Mat_<double> R;
	Rodrigues(rvec, R);

	Matx34f P = Matx34f(R(0, 0), R(0, 1), R(0, 2), t.at<double>(0, 0),
		R(1, 0), R(1, 1), R(1, 2), t.at<double>(1, 0),
		R(2, 0), R(2, 1), R(2, 2), t.at<double>(2, 0));


	scene_views[view.index].projection_matrix = P;

	if (camera_file.is_open()) {
		write_camera_data(P, camera_file);
	}
	else {
		cout << "Unable to open camera file\n";
	}

	//Assumes that previous view is ideal for triangulation. 
	triangulate_points_and_add_to_cloud(scene_views[view.index - 1], scene_views[view.index]);
}

vector<Point3f> StructureFromMotion::point_cloud_to_vector()
{
	vector<Point3f> point_vector;
	for (size_t i = 0; i < point_cloud.size(); i++) {
		point_vector.push_back(point_cloud[i].point_3D);
	}

	return point_vector;
}





