#include "Visualizer.h"

void load_point_cloud(vector<cv::Point3f> points)
{

	cout << "points in cloud: " << points.size() << endl;

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
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	io::savePLYFile("output.ply", triangles);
	io::saveOBJFile("D:/Unity/HDRP - Point Cloud/Assets/model_greedy_triangle.obj", triangles);

	cout << "created greedy triangle" << endl;

	//Marching Cubes
	//MarchingCubes<PointNormal> mc;
	MarchingCubesHoppe<PointNormal> mc;
	PolygonMesh triangles_marching_cubes;

	mc.setIsoLevel(0);
	mc.setGridResolution(50, 50, 50);
	mc.setPercentageExtendGrid(0.02);

	mc.setInputCloud(cloud_with_normals);
	mc.setSearchMethod(tree2);
	mc.reconstruct(triangles_marching_cubes);

	io::saveOBJFile("D:/Unity/HDRP - Point Cloud/Assets/model_marching_cubes.obj", triangles_marching_cubes);

	cout << "created marching_cubes triangle" << endl;

	Poisson<pcl::PointNormal> poisson;
	PolygonMesh triangles_poisson;

	poisson.setDepth(9);
	poisson.setSolverDivide(8);
	poisson.setIsoDivide(8);
	poisson.setPointWeight(4.0f);
	poisson.setSamplesPerNode(3.0f);
	poisson.setScale(1.25f);

	poisson.setInputCloud(cloud_with_normals);
	poisson.setSearchMethod(tree2);
	poisson.performReconstruction(triangles_poisson);

	io::saveOBJFile("D:/Unity/HDRP - Point Cloud/Assets/model_poisson.obj", triangles_poisson);

	cout << "created poisson triangle" << endl;

	/*
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

	PointCloud<PointNormal> mls_points;

	MovingLeastSquares<PointXYZ, PointNormal> mls;

	mls.setComputeNormals(true);

	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(3);

	mls.process(mls_points);

	cout << "original points size: " << points.size() << endl;
	cout << "mls points size: " << mls_points.size() << endl;

	ofstream output_file;
	output_file.open("D:/Unity/HDRP - Point Cloud/Assets/dense_points.csv");
	for (const auto& point : mls_points) {
		output_file << point.x << "," << point.y << "," << point.z << "," << 255 << "," << 255 << "," << 255 << "," << endl;
	}

	output_file.close();
	*

	/*
	FastBilateralFilterOMP<PointXYZ> fbFilter;
	fbFilter.setInputCloud(cloud);
	fbFilter.setSigmaS(10);
	fbFilter.setSigmaR(0.05f);

	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
	fbFilter.applyFilter(*cloud_filtered);

	cout << "original points size: " << points.size() << endl;
	cout << "mls points size: " << cloud_filtered->points.size() << endl;

	ofstream output_file;
	output_file.open("D:/Unity/HDRP - Point Cloud/Assets/dense_points.csv");
	for (const auto& point : cloud_filtered->points) {
		output_file << point.x << "," << point.y << "," << point.z << "," << 255 << "," << 255 << "," << 255 << "," << endl;
	}

	output_file.close();*/
}

void test_visualizer()
{
	//MVS::Interface interface;
	//MVS::Interface::Platform p;

	//MVS::Scene scene(8);

	////scene.Load("scene.mvs");
	//std::string s = "scene.mvs";
	//scene.Load(s);

	//scene.DenseReconstruction();
	//s = "scene_dense.mvs";
	//scene.Save(s);



}
