#pragma once
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

using namespace std;

const vector<string> PHOTO_EXTENSIONS = { ".png", ".jpg", ".ppm", ".JPG" };

bool is_valid_extension(string file_name);
vector<string> get_image_file_paths(string file_dir);
