#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

const float RATIO_FILTER_LIMIT = 0.6f;
const float EPIPOLAR_MATCH_SURVIVAL_RATE = 0.3f;


vector<vector<DMatch>> get_knn_matches(Mat descriptors1, Mat descriptors2);

vector<DMatch> ratio_filter_matches(vector<vector<DMatch>> matches);

vector<DMatch> reciprocity_filter_matches(vector<DMatch> matches, vector<DMatch> matches_reciprocal);

vector<DMatch> epipolar_filter_matches(vector<DMatch> matches, vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2);

vector<DMatch> get_basic_matches(Mat descriptors1, Mat descriptors2);