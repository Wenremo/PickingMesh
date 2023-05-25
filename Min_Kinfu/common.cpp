#include "common.h"
using namespace pcl;
bool loop = true;
pcl::visualization::PCLVisualizer::Ptr _viewer;
boost::shared_ptr<pcl::PolygonMesh> _mesh_ptr;

bool intersectTri(Eigen::Vector3f &v0, Eigen::Vector3f &v1, Eigen::Vector3f &v2,
	Eigen::Vector3f &vRayPos, Eigen::Vector3f &vRayDir, float &u, float &v, float &dist, Eigen::Vector3f &interPos)
{
	Eigen::Vector3f e1 = v1 - v0;
	Eigen::Vector3f e2 = v2 - v0;
	Eigen::Vector3f q = vRayDir.cross(e2);
	float a = e1.dot(q);//D3DXVec3Dot(&e1,&q);//e1.dot(q);
	Eigen::Vector3f s = vRayPos - v0;
	Eigen::Vector3f r = s.cross(e1);
	//D3DXVec3Cross(&r,&s,&e1);//s.cross(e1);
	// Barycentric vertex weights
	u = s.dot(q) / a;
	v = vRayDir.dot(r) / a;
	float w = 1.0f - (u + v);//weight[0] = 1.0f - (weight[1] + weight[2]);
	dist = e2.dot(r) / a;
	static const float epsilon = 1e-7f;
	static const float epsilon2 = 1e-10f;
	if ((a <= epsilon) || (u < -epsilon2) ||
		(v < -epsilon2) || (w < -epsilon2) ||
		(dist <= 0.0f)) {
		// The ray is nearly parallel to the triangle, or the
		// intersection lies outside the triangle or behind
		// the ray origin: "infinite" distance until intersection.
		return false;
	}
	else {
		interPos = v0 + u*e1 + v*e2;
		return true;
	}
}

int getIntersection(Eigen::Vector3f &vRayPos, Eigen::Vector3f &vRayDir, pcl::PolygonMesh &mesh, Eigen::Vector3f &intersection)
{
	bool bIntersect = false;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud_in);
	float u, v;
	Eigen::Vector3f pts[3];
	float minDist = std::numeric_limits<float>::max();
	int pickId = -1;
	for (size_t i = 0; i < mesh.polygons.size(); ++i)
	{
		//int indices[3];
		Eigen::Vector3f points[3];
		for (int j = 0; j < 3; ++j)
		{
			int id = mesh.polygons[i].vertices[j];
			points[j] = cloud_in.points[id].getVector3fMap();
		}
		float u1, v1, dist;
		Eigen::Vector3f interPos;
		if (intersectTri(points[0], points[1], points[2], vRayPos, vRayDir, u1, v1, dist, interPos))
		{
			if (minDist > dist)
			{
				for (int k = 0; k < 3; ++k)
				{
					pts[k] = points[k];
				}
				u = u1;
				v = v1;
				pickId = i;
				minDist = dist;
				intersection = interPos;
				bIntersect = true;
			}
		}
	}
	return pickId;
}

int pickingMesh(int px, int py, pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PolygonMesh &mesh)
{
	std::vector<pcl::visualization::Camera> cams;
	viewer->getCameras(cams);
	int width = cams[0].window_size[0];
	int height = cams[0].window_size[1];
	float cx = (float)width / 2;
	float cy = (float)height / 2;
	float y_max = tan(cams[0].fovy / 2);
	float x_max = y_max*(float)width / height;
	float x = (float)px - cx;
	float y = (float)py - cy;
	Eigen::Vector3f p;
	float w = 0.5f*(float)width;
	float h = 0.5f*(float)height;
	p[0] = -x_max*(float)x / w;
	p[1] = y_max*(float)y / h;
	p[2] = 1.0f;
	Eigen::Affine3f pose = viewer->getViewerPose();
	Eigen::Vector3f vRayDir = pose.linear()*p;
	Eigen::Vector3f vRayPos = pose.translation();
	vRayDir.normalize();
	Eigen::Vector3f intersection;
	int faceId = getIntersection(vRayPos, vRayDir, mesh, intersection);
	if ( faceId > -1)
	{
		PointCloud<PointXYZRGB> cloud;
		pcl::fromPCLPointCloud2(mesh.cloud, cloud);
		Vertices vts = mesh.polygons.at(faceId);
		for (int i = 0; i < 3; ++i)
		{
			cloud.points[vts.vertices[i]].r = 255;
			cloud.points[vts.vertices[i]].g = 0;
			cloud.points[vts.vertices[i]].b = 0;
		}
		pcl::toPCLPointCloud2(cloud, mesh.cloud);
	}
	return faceId;
}

void process_key_down(const pcl::visualization::KeyboardEvent& event, void* v)
{
	if ((event.getKeyCode() == 'q' || event.getKeyCode() == 'Q'))
	{
		loop = false;
	}
}

void process_mouse(const pcl::visualization::MouseEvent& event, void* v)
{
	if (event.getButton() == pcl::visualization::MouseEvent::RightButton)
	{
		if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress)
		{
			int px = event.getX();
			int py = event.getY();
			int facePicked = pickingMesh(px, py, _viewer, *_mesh_ptr);
			//segmentMesh(facePicked, *_mesh_ptr);
		}
	}
}

void setViewerPose(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::Affine3f &pose)
{
	Eigen::Vector3f pos_vector = pose.translation();
	Eigen::Vector3f vLookAt = pos_vector + pose.linear()*Eigen::Vector3f::UnitZ();
	Eigen::Vector3f up_vec = pose.linear()*(-Eigen::Vector3f::UnitY());
	viewer->setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2], vLookAt[0], vLookAt[1], vLookAt[2], up_vec[0], up_vec[1], up_vec[2]);
}

pcl::visualization::PCLVisualizer::Ptr createViewer(Eigen::Affine3f &camPose, int x, int y, int width, int height, std::string &caption)
{
	pcl::visualization::PCLVisualizer::Ptr viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(caption));
	viewer->setWindowName(caption);
	viewer->setBackgroundColor(0, 0, 0.15);
	viewer->addCoordinateSystem(1.0, "world");
	viewer->initCameraParameters();
	viewer->setPosition(x, y);
	viewer->setSize(width, height);
	viewer->setCameraClipDistances(0.0, 10.0);
	viewer->registerKeyboardCallback(&process_key_down);
	viewer->registerMouseCallback(&process_mouse);
	setViewerPose(viewer, camPose);
	return viewer;
}

void drawScene(Eigen::Affine3f &camPose, int x, int y, int width, int height, std::string &caption, boost::shared_ptr<pcl::PolygonMesh> mesh_ptr)
{
	_viewer = createViewer(camPose, x, y, width, height, caption);
	_mesh_ptr = mesh_ptr;
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(_mesh_ptr->cloud, *cloud_in);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	//std::vector<std::pair<int, int>> clusters;
	segmentPointCloud(cloud_in, *cloud_total, clusters);
	creatingClusters();*/
	while (loop)
	{
		_viewer->removePolygonMesh("mesh");
		_viewer->removeAllPointClouds();
		_viewer->removeText3D("length_picked");
		//_viewer->addPointCloud<pcl::PointXYZRGB>(cloud_clusters[cluster_no], "clustering");
		//_viewer->addPointCloud<pcl::PointXYZRGB>(cloud_total, "clustering");
		_viewer->addPolygonMesh(*mesh_ptr, "mesh");
		_viewer->spinOnce(10);
	}
	_viewer->close();
}