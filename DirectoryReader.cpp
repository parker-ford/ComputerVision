#include "DirectoryReader.h"

bool is_valid_extension(string file_name) {
	string extension = boost::filesystem::extension(file_name);
	return std::find(PHOTO_EXTENSIONS.begin(), PHOTO_EXTENSIONS.end(), extension) != PHOTO_EXTENSIONS.end();
}

vector<string> get_image_file_paths(string file_dir)
{
	vector<string> paths;

	for (const auto& file : boost::filesystem::directory_iterator(file_dir)) {
		if (is_valid_extension(file.path().string())) {
			paths.push_back(file.path().string());
		}
	}

	return paths;
}
