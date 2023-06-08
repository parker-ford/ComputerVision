// IMAGE MATCHER
// Author: Parker Ford
// Functions that utilize OpenCV to perform more robust feature matching


#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

const float RATIO_FILTER_LIMIT = 0.9f; //0.6f;
const float EPIPOLAR_MATCH_SURVIVAL_RATE = 0.6f; //0.3f;


// get_2nn_matches - for a set of descriptors, get the the two best matches for each descriptor
// precondition: descriptors have to be found for two images
// postcondition: a vector of the two best matches for each descriptor is returned
vector<vector<DMatch>> get_2nn_matches(Mat descriptors1, Mat descriptors2);

// ratio_filter_matches - given the two best matches for a descriptor, only keep the match if the best match is a certain degree better than the second best match
// precondition: a vector of the two best matches for each descriptor must be made
// postcondition: a vector of ratio filtered matches is returned
vector<DMatch> ratio_filter_matches(vector<vector<DMatch>> matches);

// reciprocity_filter_matches - takes in a vector of matches from image 1 to image 2 as well as a vector of matches from image 2 to image 1. If the match is the same in both directions, it is preseved
// precondition: match vectors have to be made for in both directions between two images
// postcondition: a vector of reciprocity matches is returned
vector<DMatch> reciprocity_filter_matches(vector<DMatch> matches, vector<DMatch> matches_reciprocal);

// epipolar_filter_matches - filters matches based on geometric constraints. Utilizes OpenCVs findFundamentalMat function and only keeps the inlier matches
// precondition: matches between two images as well as their keypoints have to be found
// postcondition: a vector of epipolar filtered matches is returned
vector<DMatch> epipolar_filter_matches(vector<DMatch> matches, vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2);

// get_basic_matches - method to get matches with no filtering. Used for visualization / debugging
// precondition: descriptors for two images have been found
// postcondition: a vector of matches is returned
vector<DMatch> get_basic_matches(Mat descriptors1, Mat descriptors2);