#pragma once
#include <opencv2/opencv.hpp>
#include "StructureFromMotion.h"

void debug_view_scene_images(SceneViews scene_views);
void debug_view_scene_matches(SceneViews scene_views, MatchMatrix match_matrix);