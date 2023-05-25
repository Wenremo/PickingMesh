#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/ply_io.h>
void drawScene(Eigen::Affine3f &camPose, int x, int y, int width, int height, std::string &caption, boost::shared_ptr<pcl::PolygonMesh> mesh_ptr);
