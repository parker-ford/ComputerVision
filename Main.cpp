//#include<openmvs/MVS.h>
#include "StructureFromMotion.h"
#include "Visualizer.h"
#include "Debug.h"
#include "Visualizer.h"

int main() {
	std::cout << "test" << std::endl;

	StructureFromMotion sfm;
	sfm.read_images();
	sfm.detect_keypoints_and_descriptors();
	sfm.pairwise_match_views();
	sfm.initialize_intrinsics();
	sfm.intitialize_structure();
	sfm.increment_views();

	//test_visualizer();
	debug_write_image_matches(sfm.get_scene_views(), sfm.get_match_matrix());


	load_point_cloud(sfm.point_cloud_to_vector());

	//debug_view_scene_images(sfm.get_scene_views());

	return 0;
}