
/*

#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <boost/filesystem.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>


using namespace cv;
using namespace std;

struct Image2D3DMatch {
	Point2f points2D;
	Point3f points3D;
};

typedef std::map<int, Image2D3DMatch> Images2D3DMatches;


vector<DMatch> get_matches_with_ratio_filter(Mat desc1, Mat desc2) {
	//Match Features
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	vector<vector<DMatch>> matches;
	matcher->knnMatch(desc1, desc2, matches, 2);

	//Ratio Filter Matches
	std::vector<DMatch> matches_ratio_filtered;
	for (size_t i = 0; i < matches.size(); i++) {
		if (matches[i][0].distance < 0.8f * matches[i][1].distance) {
			matches_ratio_filtered.push_back(matches[i][0]);
		}
	}

	return matches_ratio_filtered;
}

vector<DMatch> get_matches_reciprocity_filtered(vector<DMatch> matches, vector<DMatch> matches_rcp) {
	vector<DMatch> merged;
	for (size_t i = 0; i < matches_rcp.size(); i++) {
		DMatch match_rcp = matches_rcp[i];
		bool found = false;
		for (size_t j = 0; j < matches.size(); j++) {
			DMatch match = matches[j];
			if (match_rcp.queryIdx == match.trainIdx && match_rcp.trainIdx == match.queryIdx) {
				merged.push_back(match);
				found = true;
				break;
			}
		}
		if (found) {
			continue;
		}
	}

	return merged;
}

vector<DMatch> get_matches_with_epipolar_filter(vector<DMatch> matches, vector<KeyPoint> keypts1, vector<KeyPoint> keypts2) {
	vector<uint8_t> inlier_mask(matches.size());
	vector<Point2f> points1, points2;
	for (size_t i = 0; i < matches.size(); i++) {
		DMatch match = matches[i];
		points1.push_back(keypts1[match.queryIdx].pt);
		points2.push_back(keypts2[match.trainIdx].pt);
	}
	findFundamentalMat(points1, points2, inlier_mask);

	vector<DMatch> filtered_matches;
	for (size_t i = 0; i < matches.size(); i++) {
		if (inlier_mask[i]) {
			filtered_matches.push_back(matches[i]);
		}
	}

	//TODO may need to change 0.5f, this is the pair match survival rate
	if ((float)filtered_matches.size() / (float)matches.size() < 0.5f) {
		std::cout << " does not pass pair match survival rate" << std::endl;
	}

	return filtered_matches;
}

void debug_view_matches(Mat img1, vector<KeyPoint> keypts1, Mat img2, vector<KeyPoint> keypts2, vector<DMatch> matches) {
	Mat img_matches;
	drawMatches(img1, keypts1, img2, keypts2, matches, img_matches);
	imshow("Matches", img_matches);
	waitKey(0);
}

Mat down_sample_img(double factor, Mat img) {
	Mat resized_image;
	resize(img, resized_image, Size(), factor, factor, INTER_LINEAR);
	return resized_image;
}

Mat down_sample_matrix(double factor, Mat m) {
	Mat resized_m;
	divide(m, factor, resized_m);

	return resized_m;
}

void write_camera_data(Matx34f P, ofstream& file) {
	file << P(0, 0) << "," << P(0, 1) << "," << P(0, 2) << "," << P(0, 3) << "," << P(1, 0) << "," << P(1, 1) << "," << P(1, 2) << "," << P(1, 3) << "," << P(2, 0) << "," << P(2, 1) << "," << P(2, 2) << "," << P(2, 3) << "," << endl;
}

void write_point_data(Mat points, ofstream& file) {

	for (int i = 0; i < points.cols; i++) {
		for (int j = 0; j < points.rows; j++) {
			//cout << points.at<float>(j, i);
			file << points.at<float>(j, i) << ",";
		}
		// color
		for (int i = 0; i < 3; i++) {
			file << "1" << ",";
		}
		file << "\n";
	}
}

void write_point_data_2(Mat points, ofstream& file, vector<Vec3b> color) {

	for (int i = 0; i < points.rows; i++) {
		//position
		for (int j = 0; j < points.cols; j++) {
			//cout << points.at<float>(j, i);
			file << points.at<float>(i, j) << ",";
		}
		// color
		file << static_cast<int>(color[i][2]) << ",";
		file << static_cast<int>(color[i][1]) << ",";
		file << static_cast<int>(color[i][0]) << ",";
		file << "\n";
	}
}

int main2() {

	//Load Images
	Mat img1 = imread("goblin/g1.jpg");
	Mat img2 = imread("goblin/g2.jpg");

	//Downscale image
	double scale_factor = .25;
	img1 = down_sample_img(scale_factor, img1);
	img2 = down_sample_img(scale_factor, img2);

	//Intrinsic Matrix (hardcoded for my iphone for now)
	double cx = 1521.316552582157;
	double cy = 2024.8360801192564;
	double fx = 3031.53030720967;
	double fy = 3033.7600300469517;
	Mat K = (Mat_<double>(3, 3) << (fx * scale_factor), 0, (cx * scale_factor), 0, (fy * scale_factor), (cy * scale_factor), 0, 0, 1);
	double focal = K.at<double>(0, 0);
	Point2d pp(K.at<double>(0, 2), K.at<double>(1, 2));

	//Detect Features
	Ptr<FeatureDetector> detector = AKAZE::create();
	vector<KeyPoint> keypts1, keypts2;
	Mat desc1, desc2;
	detector->detectAndCompute(img1, noArray(), keypts1, desc1);
	detector->detectAndCompute(img2, noArray(), keypts2, desc2);

	//Get Matches
	vector<DMatch> matches = get_matches_with_ratio_filter(desc1, desc2);
	vector<DMatch> matches_rcp = get_matches_with_ratio_filter(desc2, desc1);
	vector<DMatch> matches_merged = get_matches_reciprocity_filtered(matches, matches_rcp);
	vector<DMatch> matches_final = get_matches_with_epipolar_filter(matches_merged, keypts1, keypts2);

	//debug_view_matches(img1, keypts1, img2, keypts2, matches_final);

	//Align Matches
	vector<Point2f>leftPts, rightPts;
	for (size_t i = 0; i < matches_final.size(); i++) {
		leftPts.push_back(keypts1[matches_final[i].queryIdx].pt);
		rightPts.push_back(keypts2[matches_final[i].trainIdx].pt);
	}
	//
	//Find Essential Mat
	Mat status;
	Mat E = findEssentialMat(leftPts, rightPts, K, RANSAC, 0.999, 1.0, status);

	//Recover Pose
	Mat_<double> R, t;
	recoverPose(E, leftPts, rightPts, R, t, focal, pp, status);

	//Get point colors

	std::cout << "Image type: " << img1.type() << std::endl;  // Should print 16 for CV_8UC3
	std::cout << "Image channels: " << img1.channels() << std::endl;  // Should print 3 for color image
	std::cout << "Image size: " << img1.size() << std::endl;  // Should print the dimensions of your image

	cv::Vec3b pixel = img1.at<cv::Vec3b>(10, 20);
	std::cout << "Pixel value: " << pixel << std::endl;  // Should print the BGR values of the pixel


	vector<Vec3b> colors;
	for (int i = 0; i < leftPts.size(); i++) {
		int x = static_cast<int>(leftPts[i].x);
		int y = static_cast<int>(leftPts[i].y);
		//cout << "X : " << x << " Y: " << y << endl;
		colors.push_back(img1.at<Vec3b>(y, x));
	}

	for (int i = 0; i < colors.size(); i++) {
		//cout << colors[i][0] << "," << colors[i][1] << "," <<  colors[i][2] << endl;
		//cout << colors[i][0] << endl;

	}

	Matx34f Pleft = Matx34f::eye();
	Matx34f Pright = Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
		R(1, 0), R(1, 1), R(1, 2), t(1),
		R(2, 0), R(2, 1), R(2, 2), t(2));

	//Undistort points
	Mat normLPts;
	Mat normRPts;
	undistortPoints(leftPts, normLPts, K, Mat());
	undistortPoints(rightPts, normRPts, K, Mat());
	//undistortPoints();

	//Convert Points to Homogeneous
	Mat leftPts_h, rightPts_h;
	convertPointsToHomogeneous(leftPts, leftPts_h);
	convertPointsToHomogeneous(rightPts, rightPts_h);

	//Triangulate points
	Mat points4D;
	//triangulatePoints(Pleft, Pright, leftPts_h, rightPts_h, points4D);
	triangulatePoints(Pleft, Pright, normLPts, normRPts, points4D);

	//Convert to 3D
	Mat points3D;
	convertPointsFromHomogeneous(points4D.t(), points3D);
	points3D = points3D.reshape(1);


	Images2D3DMatches matches2D_3D;

	ofstream test;

	//Write camera file
	ofstream camera_file;
	camera_file.open("D:/Unity/HDRP - Point Cloud/Assets/cameras.csv");
	if (camera_file.is_open()) {
		write_camera_data(Pleft, camera_file);
		write_camera_data(Pright, camera_file);
	}
	else {
		cout << "Unable to open camera file\n";
	}
	camera_file.close();

	//Write points file
	ofstream point_file;
	point_file.open("D:/Unity/HDRP - Point Cloud/Assets/points.csv");
	if (point_file.is_open()) {
		//write_point_data(points4D, point_file);
		write_point_data_2(points3D, point_file, colors);
	}
	else {
		cout << "Unable to open points file\n";
	}
	point_file.close();

	//debug_view_matches(img1, keypts1, img2, keypts2, matches_final);

	return 0;
}

*/