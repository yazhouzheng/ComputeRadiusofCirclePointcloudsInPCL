#include <iostream>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>

#define CIRCLE 1
#define TEST1  0
#define TEST2  1

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->initCameraParameters();
	return (viewer);
}

int main(int argc, char** argv)
{
#if CIRCLE
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = 1000;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
    #if TEST1
		cloud->points[i].x = (rand() / (RAND_MAX + 1.0) - 0.5);
		cloud->points[i].y = (rand() / (RAND_MAX + 1.0) - 0.5);
		cloud->points[i].z = (rand() / (RAND_MAX + 1.0) - 0.5);
		if (i % 2 == 0)
		{
			cloud->points[i].y = (rand() / (RAND_MAX + 1.0) - 0.5);
			cloud->points[i].z = 0;
		}
    #endif
    #if TEST2
		cloud->points[i].x = (rand() / (RAND_MAX + 1.0) - 0.5);
		if (i % 2 == 0)
			cloud->points[i].y = sqrt(0.25 - (cloud->points[i].x * cloud->points[i].x));
		else
			cloud->points[i].y = - sqrt(0.25 - (cloud->points[i].x * cloud->points[i].x));

		if (i % 5 == 0)
		{ 
			cloud->points[i].y = (rand() / (RAND_MAX + 1.0) - 0.5);
			cloud->points[i].z = (rand() / (RAND_MAX + 1.0) - 0.5);
		}
    #endif
	}
	//pcl::io::savePLYFile("spherePointCloudCircle.ply", *cloud);
#endif

	std::vector<int> inliers;

	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr
		model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));

	pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
		model_circle2D(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud));

	if (pcl::console::find_argument(argc, argv, "-c2d") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle2D);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		Eigen::VectorXf tmpSphereMatrix;
		ransac.getModelCoefficients(tmpSphereMatrix);
		std::cout << tmpSphereMatrix << "\n\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-c3d") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle3D);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);

		Eigen::VectorXf tmpSphereMatrix;
		ransac.getModelCoefficients(tmpSphereMatrix);
		std::cout << tmpSphereMatrix << "\n\n";
	}

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

	pcl::visualization::PCLVisualizer::Ptr viewer;
	if (pcl::console::find_argument(argc, argv, "-c2d") >= 0 || pcl::console::find_argument(argc, argv, "-c3d") >= 0)
		viewer = simpleVis(final);
	else
		viewer = simpleVis(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
	return 0;
}