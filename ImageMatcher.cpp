#include "ImageMatcher.h"

//vector<DMatch> get_view_matches(View view1, View view2)
//{
//	vector<vector<DMatch>> knn_matches = get_knn_matches(view1.descriptors, view2.descriptors);
//	vector<vector<DMatch>> knn_matches_reciprocal = get_knn_matches(view2.descriptors, view1.descriptors);
//
//	vector<DMatch> ratio_filtered_matches = ratio_filter_matches(knn_matches);
//	vector<DMatch> ratio_filtered_matches_reciprocal = ratio_filter_matches(knn_matches_reciprocal);
//
//	vector<DMatch> reciprocity_filtered_matches = reciprocity_filter_matches(ratio_filtered_matches, ratio_filtered_matches_reciprocal);
//
//	vector<DMatch> epipolar_filtered_matches = epipolar_filter_matches(reciprocity_filtered_matches, view1.keypoints, view2.keypoints);
//
//
//	return epipolar_filtered_matches;
//}

vector<vector<DMatch>> get_knn_matches(Mat descriptors1, Mat descriptors2)
{
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	vector<vector<DMatch>> matches;
	matcher->knnMatch(descriptors1, descriptors2, matches, 2);
	return matches;
}

vector<DMatch> ratio_filter_matches(vector<vector<DMatch>> matches)
{
	std::vector<DMatch> matches_ratio_filtered;
	for (size_t i = 0; i < matches.size(); i++) {
		if (matches[i][0].distance < RATIO_FILTER_LIMIT * matches[i][1].distance) {
			matches_ratio_filtered.push_back(matches[i][0]);
		}
	}

	return matches_ratio_filtered;
}

vector<DMatch> reciprocity_filter_matches(vector<DMatch> matches, vector<DMatch> matches_reciprocal)
{
	vector<DMatch> merged_matches;
	for (size_t i = 0; i < matches_reciprocal.size(); i++) {
		DMatch match_reciprocal = matches_reciprocal[i];
		bool found = false;
		for (size_t j = 0; j < matches.size(); j++) {
			DMatch match = matches[j];
			if (match_reciprocal.queryIdx == match.trainIdx && match_reciprocal.trainIdx == match.queryIdx) {
				merged_matches.push_back(match);
				found = true;
				break;
			}
		}
		if (found) {
			continue;
		}
	}

	return merged_matches;
}

vector<DMatch> epipolar_filter_matches(vector<DMatch> matches, vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2)
{
	vector<DMatch> filtered_matches;
	vector<uint8_t> inlier_mask(matches.size());
	vector<Point2f> points1, points2;
	for (size_t i = 0; i < matches.size(); i++) {
		DMatch match = matches[i];
		points1.push_back(keypoints1[match.queryIdx].pt);
		points2.push_back(keypoints2[match.trainIdx].pt);
	}

	if (points1.empty() || points2.empty()) {
		return filtered_matches;
	}
	if (points1.size() != points2.size()) {
		return filtered_matches;
	}

	findFundamentalMat(points1, points2, inlier_mask);

	for (size_t i = 0; i < matches.size(); i++) {
		if (inlier_mask[i]) {
			filtered_matches.push_back(matches[i]);
		}
	}

	//TODO: may need to skip this match if it does not pass survival rate
	if ((float)filtered_matches.size() / (float)matches.size() < EPIPOLAR_MATCH_SURVIVAL_RATE) {
		std::cout << " does not pass match survival rate" << std::endl;
	}

	return filtered_matches;
}

vector<DMatch> get_basic_matches(Mat descriptors1, Mat descriptors2)
{
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	vector<DMatch> matches;
	matcher->match(descriptors1, descriptors2, matches);


	return matches;
}
