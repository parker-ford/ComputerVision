#include "Visualizer.h"

void generate_greedy_triangulation_mesh(vector<cv::Point3f> points){

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	for (size_t i = 0; i < points.size(); i++) {
		PointXYZ point;
		point.x = points[i].x;
		point.y = points[i].y;
		point.z = points[i].z;
		cloud->push_back(point);
	}

	NormalEstimation<PointXYZ, Normal> n;
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *normals, *cloud_with_normals);

	search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//Greedy Triangulation
	GreedyProjectionTriangulation<PointNormal> gp3;
	PolygonMesh triangles;

	gp3.setSearchRadius(3);

	gp3.setMu(2.5);

	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 2);
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	io::saveOBJFile(MESH_FILE_PATH, triangles);


}

