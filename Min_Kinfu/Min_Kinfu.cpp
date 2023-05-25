// Min_Kinfu.cpp : Defines the entry point for the console application.

#define _CRT_SECURE_NO_DEPRECATE
#include "stdafx.h"
#include <iostream>
#include <vector>

#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/exceptions.h>
#include <pcl/io/png_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/angles.h>
#include "common.h"

#include <direct.h>

using namespace std;
using namespace pcl;
using namespace Eigen;
namespace pc = pcl::console;

#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_io_ply_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_visualization_release.lib")

pcl::visualization::PCLVisualizer::Ptr viewer;

int
main (int argc, char* argv[])
{  
	std::string ply_path;
	pc::parse_argument (argc, argv, "-ply", ply_path);
	pcl::PolygonMesh mesh;
	pcl::PolygonMeshPtr mesh_ptr(new pcl::PolygonMesh());
	pcl::io::loadPLYFile(ply_path, mesh);
	pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud_in);
	Eigen::Vector3f vCenter(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < cloud_in.points.size(); ++i)
	{
		vCenter = (i*vCenter + cloud_in.points[i].getVector3fMap()) / (i + 1);
	}
	float maxDist = 0.0f;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_new;
	for (size_t i = 0; i < cloud_in.points.size(); ++i)
	{
		Eigen::Vector3f vert = cloud_in.points[i].getVector3fMap();
		Eigen::Vector3f vec3 = vert - vCenter;
		if (maxDist < vec3.norm())
			maxDist = vec3.norm();
		vert -= vCenter;
		pcl::PointXYZRGB p;
		p.x = vert[0];
		p.y = vert[1];
		p.z = vert[2];
		p.r = cloud_in.points[i].r;
		p.g = cloud_in.points[i].g;
		p.b = cloud_in.points[i].b;
		cloud_new.points.push_back(p);
	}
	mesh_ptr->polygons.swap(mesh.polygons);
	pcl::toPCLPointCloud2(cloud_new, mesh_ptr->cloud);
	Eigen::Affine3f camPose;
	camPose.translation() = Eigen::Vector3f(0.0f, 0.0f, -maxDist);
	camPose.linear() = Eigen::Matrix3f::Identity();
	std::string caption = "picking";
	drawScene(camPose, 0, 0, 640, 480, caption, mesh_ptr);
	return 0;
}

