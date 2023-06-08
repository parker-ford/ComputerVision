// DEBUG
// Author: Parker Ford
// Various debug functions mainly to view images and matches between images


#pragma once
#include <opencv2/opencv.hpp>
#include "StructureFromMotion.h"

const int RESIZE_FACTOR = 8;
const string DEBUG_IMAGE_MATCHES_DIR = "view_matches";

// debug_view_scene_images - debug function to see if images have been loaded by program correctly
// precondition: images have been loaded into SceneViews
// postcondition: loaded images are displayed to user
void debug_view_scene_images(SceneViews scene_views);

// debug_view_scene_matches - debug function to view all pairwise matches
// precondition: images have been loaded to SceneViews and match matrix has been computed
// postcondition: matches between all pairs of images are displayed to the screen
void debug_view_scene_matches(SceneViews scene_views, MatchMatrix match_matrix);

// debug_write_image_matches - 
// precondition: images have been loaded to SceneViews and match matrix has been computed
// postcondition: matches between all pairs of images are written into DEBUG_IMAGE_MATCHES_DIR directory
void debug_write_image_matches(SceneViews scene_views, MatchMatrix match_matrix);