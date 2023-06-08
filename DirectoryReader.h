// DIRECTORY READER
// Author: Parker Ford
// Helper functions to read image file names from a directories

#pragma once
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

using namespace std;

const vector<string> PHOTO_EXTENSIONS = { ".png", ".jpg", ".ppm", ".JPG" };

// is_valid_extension - determines if a file is a photo
// precondition: a file name is found
// postcondition: the file is found to be either a photo or not
bool is_valid_photo_extension(string file_name);

// get_image_file_paths - finds all photos in a directory
// precondition: a directory is given
// postcondition: all file names that are determined to be photos are returned
vector<string> get_image_file_paths(string file_dir);
