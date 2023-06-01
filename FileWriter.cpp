#include "FileWriter.h"

void write_point_data(Mat points, ofstream& file) {

	for (int i = 0; i < points.rows; i++) {
		//position
		for (int j = 0; j < points.cols; j++) {
			//cout << points.at<float>(j, i);
			file << points.at<float>(i, j) << ",";
		}
		file << 255 << "," << 255 << "," << 255 << "," << endl;
	}
}

void write_camera_data(Matx34f P, ofstream& file) {
	file << P(0, 0) << "," << P(0, 1) << "," << P(0, 2) << "," << P(0, 3) << "," << P(1, 0) << "," << P(1, 1) << "," << P(1, 2) << "," << P(1, 3) << "," << P(2, 0) << "," << P(2, 1) << "," << P(2, 2) << "," << P(2, 3) << "," << endl;
}