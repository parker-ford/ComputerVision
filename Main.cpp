#include "StructureFromMotion.h"
#include "Debug.h"

int main() {
	std::cout << "test" << std::endl;
	StructureFromMotion sfm;
	sfm.read_images();
	sfm.detect_keypoints_and_descriptors();
	sfm.pairwise_match_views();
	sfm.initialize_intrinsics();
	sfm.intitialize_structure();

	//debug_view_scene_matches(sfm.get_scene_views(), sfm.get_match_matrix());

	//debug_view_scene_images(sfm.get_scene_views());

	return 0;
}