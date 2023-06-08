#include "Debug.h"


void debug_view_scene_images(SceneViews scene_views) {
	for (size_t i = 0; i < scene_views.size(); i++) {
		imshow("Debug View Scene Images", scene_views[i].image);
		waitKey(0);
	}
}

void debug_view_scene_matches(SceneViews scene_views, MatchMatrix match_matrix) {
	namedWindow("image", WINDOW_NORMAL);

	int width = scene_views[0].image.size().width * DOWNSCALE_FACTOR / RESIZE_FACTOR * 2;
	int height = scene_views[0].image.size().height * DOWNSCALE_FACTOR / RESIZE_FACTOR;
	resizeWindow("image", width, height);

	for (size_t i = 0; i < scene_views.size() - 1; i++) {
		for (size_t j = i + 1; j < scene_views.size(); j++) {
			Mat matches_image;
			drawMatches(scene_views[i].image, scene_views[i].keypoints, scene_views[j].image, scene_views[j].keypoints, match_matrix[i][j], matches_image);
			imshow("image", matches_image);
			waitKey(0);
		}
	}
}

void debug_write_image_matches(SceneViews scene_views, MatchMatrix match_matrix) {
	for (size_t i = 0; i < scene_views.size() - 1; i++) {
		for (size_t j = i + 1; j < scene_views.size(); j++) {
			Mat matches_image;
			drawMatches(scene_views[i].image, scene_views[i].keypoints, scene_views[j].image, scene_views[j].keypoints, match_matrix[i][j], matches_image);
			string file_name = DEBUG_IMAGE_MATCHES_DIR + "/match_" + to_string(i) + "_" + to_string(j) + ".jpg";
			imwrite(file_name, matches_image);
		}
	}
}
