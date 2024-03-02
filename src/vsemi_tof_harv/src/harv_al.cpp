#include "harv_al.h"

#include <thread>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "util.h"

//float truck_center_distance_estimated = 3.0;//2.5; // truck 1 - 3 meters away from camera, average 2.0 meter, and truck width 2.0 meter, so default distance would be 3.0 //2.5 meters
extern int sprout_y_moves;
extern float truck_center_distance_estimated;

Eigen::Affine3d compute_rotation_matrix(Pose pose)
{
	Eigen::Quaterniond q;
	q.x() = pose.x;
	q.y() = pose.y;
	q.z() = pose.z;
	q.w() = pose.w;

	Eigen::Affine3d t_imu = Eigen::Affine3d::Identity();
	t_imu.rotate (q);

	Eigen::Affine3d t_sensor_2_world = Eigen::Affine3d::Identity();
	Eigen::Matrix3d m;
	m <<
	1, 0, 0,
	0, 0, 1,
	0, -1, 0;
	t_sensor_2_world.rotate (m);

	Eigen::Affine3d t = t_imu * t_sensor_2_world;

	// to check and reverse the yaw transform
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_x(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_x_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);

	pcl::PointXYZRGBL p1;
	p1.x = 0;
	p1.y = 0;
	p1.z = 3;
	p1.r = 255;
	p1.g = 0;
	p1.b = 0;

	pcl::PointXYZRGBL p2;
	p2.x = -10;
	p2.y = 0;
	p2.z = 0;
	p2.r = 255;
	p2.g = 0;
	p2.b = 0;

	pcl::PointXYZRGBL p3;
	p3.x = 10;
	p3.y = 0;
	p3.z = 0;
	p3.r = 255;
	p3.g = 0;
	p3.b = 0;

	_cloud_x->points.push_back(p1);
	_cloud_x->points.push_back(p2);
	_cloud_x->points.push_back(p3);

	pcl::transformPointCloud (*_cloud_x, *_cloud_x, t);
	for (int i = 0; i < 3; i ++)
	{
		pcl::PointXYZRGBL &p = _cloud_x->points[i];
		p.z = 0;
	}

	pcl::PointXYZRGBL cloud_x_min_OBB, cloud_x_max_OBB, cloud_x_position_OBB;
	Eigen::Matrix3f cloud_x_rotational_matrix_OBB;
	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> feature;
	feature.setInputCloud(_cloud_x);
	feature.compute();
	feature.getOBB(cloud_x_min_OBB, cloud_x_max_OBB, cloud_x_position_OBB, cloud_x_rotational_matrix_OBB);

	Eigen::Affine3f t_cloud_x_OBB = Eigen::Affine3f::Identity();
	t_cloud_x_OBB.rotate (cloud_x_rotational_matrix_OBB);

	Eigen::Affine3f t_cloud_x_OBB_inverse = t_cloud_x_OBB.inverse();		
	pcl::transformPointCloud (*_cloud_x, *_cloud_x_OBB, t_cloud_x_OBB_inverse);

	Eigen::Affine3d t_cloud_x_OBB_inverseD = t_cloud_x_OBB_inverse.cast<double>();
	t = t_cloud_x_OBB_inverseD * t;

	pcl::PointXYZRGBL cp = _cloud_x_OBB->points[0];
	if (cp.y < 0)
	{

		Eigen::AngleAxisd r;
		r =
				Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
		Eigen::Affine3d t_ = Eigen::Affine3d::Identity();
		t_.rotate (r);
		pcl::transformPointCloud (*_cloud_x_OBB, *_cloud_x_OBB, t_);

		t = t_ * t;
	}

	return t;
}

void split_cloud_4(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4
)
{
	int n_points = cloud->points.size();
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud->points[i];
		uint32_t label = p.label;
		uint16_t x = static_cast<uint16_t>((label & 0x00FFF000) >> 12);
		uint16_t y = static_cast<uint16_t> (label & 0x00000FFF);
		
		if (y < 60)
		{
			cloud_1->points.push_back(p);
		} 
		else if (y >= 60 && y < 120)
		{
			cloud_2->points.push_back(p);
		}
		else if (y >= 120 && y < 180)
		{
			cloud_3->points.push_back(p);
		}
		else if (y >= 180 && y < 240)
		{
			cloud_4->points.push_back(p);
		}		
	}

	int n_1_points = cloud_1->points.size();	
	cloud_1->resize(n_1_points);
	cloud_1->width = n_1_points;
	cloud_1->height = 1;
	cloud_1->is_dense = false;

	int n_2_points = cloud_2->points.size();	
	cloud_2->resize(n_2_points);
	cloud_2->width = n_2_points;
	cloud_2->height = 1;
	cloud_2->is_dense = false;

	int n_3_points = cloud_3->points.size();	
	cloud_3->resize(n_3_points);
	cloud_3->width = n_3_points;
	cloud_3->height = 1;
	cloud_3->is_dense = false;

	int n_4_points = cloud_4->points.size();	
	cloud_4->resize(n_4_points);
	cloud_4->width = n_4_points;
	cloud_4->height = 1;
	cloud_4->is_dense = false;
}

void split_cloud_4_y(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4
)
{
	int n_points = cloud->points.size();
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud->points[i];
		
		if (p.y < 2.0)
		{
			cloud_1->points.push_back(p);
		} 
		else if (p.y >= 2.0 && p.y < 3.0)
		{
			cloud_2->points.push_back(p);
		}
		else if (p.y >= 3.0 && p.y < 4.0)
		{
			cloud_3->points.push_back(p);
		}
		else if (p.y >= 4.0)
		{
			cloud_4->points.push_back(p);
		}
	}

	int n_1_points = cloud_1->points.size();	
	cloud_1->resize(n_1_points);
	cloud_1->width = n_1_points;
	cloud_1->height = 1;
	cloud_1->is_dense = false;

	int n_2_points = cloud_2->points.size();	
	cloud_2->resize(n_2_points);
	cloud_2->width = n_2_points;
	cloud_2->height = 1;
	cloud_2->is_dense = false;

	int n_3_points = cloud_3->points.size();	
	cloud_3->resize(n_3_points);
	cloud_3->width = n_3_points;
	cloud_3->height = 1;
	cloud_3->is_dense = false;

	int n_4_points = cloud_4->points.size();	
	cloud_4->resize(n_4_points);
	cloud_4->width = n_4_points;
	cloud_4->height = 1;
	cloud_4->is_dense = false;
}
void split_cloud_4_z(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4
)
{
	int n_points = cloud->points.size();
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud->points[i];		
		if (std::isnan(p.z)) continue;

		if (p.z >= 2.0 && p.z < 3.0)
		{
			cloud_1->points.push_back(p);
		} 
		else if (p.z >= 3.0 && p.z < 4.0)
		{
			cloud_2->points.push_back(p);
		}
		else if (p.z >= 4.0 && p.z < 5.0)
		{
			cloud_3->points.push_back(p);
		}
		else if (p.z >= 5.0)
		{
			cloud_4->points.push_back(p);
		}
	}

	int n_1_points = cloud_1->points.size();	
	cloud_1->resize(n_1_points);
	cloud_1->width = n_1_points;
	cloud_1->height = 1;
	cloud_1->is_dense = false;

	int n_2_points = cloud_2->points.size();	
	cloud_2->resize(n_2_points);
	cloud_2->width = n_2_points;
	cloud_2->height = 1;
	cloud_2->is_dense = false;

	int n_3_points = cloud_3->points.size();	
	cloud_3->resize(n_3_points);
	cloud_3->width = n_3_points;
	cloud_3->height = 1;
	cloud_3->is_dense = false;

	int n_4_points = cloud_4->points.size();	
	cloud_4->resize(n_4_points);
	cloud_4->width = n_4_points;
	cloud_4->height = 1;
	cloud_4->is_dense = false;
}
void split_cloud_8_z(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4_2
)
{
	int n_points = cloud->points.size();
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud->points[i];		
		if (std::isnan(p.z)) continue;

		if (p.z >= 2.0 && p.z < 3.0)
		{
			if (p.x >= 0)
			{
				cloud_1_1->points.push_back(p);
			} else 
			{
				cloud_1_2->points.push_back(p);
			}
			
		} 
		else if (p.z >= 3.0 && p.z < 4.0)
		{
			if (p.x >= 0)
			{
				cloud_2_1->points.push_back(p);
			} else 
			{
				cloud_2_2->points.push_back(p);
			}
		}
		else if (p.z >= 4.0 && p.z < 5.0)
		{
			if (p.x >= 0)
			{
				cloud_3_1->points.push_back(p);
			} else 
			{
				cloud_3_2->points.push_back(p);
			}
		}
		else if (p.z >= 5.0)
		{
			if (p.x >= 0)
			{
				cloud_4_1->points.push_back(p);
			} else 
			{
				cloud_4_2->points.push_back(p);
			}
		}
	}

	n_points = cloud_1_1->points.size();	
	cloud_1_1->resize(n_points);
	cloud_1_1->width = n_points;
	cloud_1_1->height = 1;
	cloud_1_1->is_dense = false;

	n_points = cloud_1_2->points.size();	
	cloud_1_2->resize(n_points);
	cloud_1_2->width = n_points;
	cloud_1_2->height = 1;
	cloud_1_2->is_dense = false;

	n_points = cloud_2_1->points.size();	
	cloud_2_1->resize(n_points);
	cloud_2_1->width = n_points;
	cloud_2_1->height = 1;
	cloud_2_1->is_dense = false;

	n_points = cloud_2_2->points.size();	
	cloud_2_2->resize(n_points);
	cloud_2_2->width = n_points;
	cloud_2_2->height = 1;
	cloud_2_2->is_dense = false;

	n_points = cloud_3_1->points.size();	
	cloud_3_1->resize(n_points);
	cloud_3_1->width = n_points;
	cloud_3_1->height = 1;
	cloud_3_1->is_dense = false;

	n_points = cloud_3_2->points.size();	
	cloud_3_2->resize(n_points);
	cloud_3_2->width = n_points;
	cloud_3_2->height = 1;
	cloud_3_2->is_dense = false;

	n_points = cloud_4_1->points.size();	
	cloud_4_1->resize(n_points);
	cloud_4_1->width = n_points;
	cloud_4_1->height = 1;
	cloud_4_1->is_dense = false;

	n_points = cloud_4_2->points.size();	
	cloud_4_2->resize(n_points);
	cloud_4_2->width = n_points;
	cloud_4_2->height = 1;
	cloud_4_2->is_dense = false;
}

void split_cloud_16_z(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,
	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> clouds_out
)
{
	int n_points = cloud_in->points.size();
	/*
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud_in->points[i];		
		if (std::isnan(p.z)) continue;

		if (p.z >= 2.0 && p.z < 2.25)
		{
			clouds_out[0]->points.push_back(p);
		} else if (p.z >= 2.25 && p.z < 2.5) 
		{
			clouds_out[1]->points.push_back(p);
		} else if (p.z >= 2.5 && p.z < 2.75) 
		{
			clouds_out[2]->points.push_back(p);
		} else if (p.z >= 2.75 && p.z < 3.0) 
		{
			clouds_out[3]->points.push_back(p);
		} else if (p.z >= 3.0 && p.z < 3.25) 
		{
			clouds_out[4]->points.push_back(p);
		} else if (p.z >= 3.25 && p.z < 3.5) 
		{
			clouds_out[5]->points.push_back(p);
		} else if (p.z >= 3.5 && p.z < 3.75) 
		{
			clouds_out[6]->points.push_back(p);
		} else if (p.z >= 3.75 && p.z < 4.0) 
		{
			clouds_out[7]->points.push_back(p);
		} else if (p.z >= 4.0 && p.z < 4.25) 
		{
			clouds_out[8]->points.push_back(p);
		} else if (p.z >= 4.25 && p.z < 4.5) 
		{
			clouds_out[9]->points.push_back(p);
		} else if (p.z >= 4.5 && p.z < 4.75) 
		{
			clouds_out[10]->points.push_back(p);
		} else if (p.z >= 4.75 && p.z < 5.0) 
		{
			clouds_out[11]->points.push_back(p);
		} else if (p.z >= 5.0 && p.z < 5.25) 
		{
			clouds_out[12]->points.push_back(p);
		} else if (p.z >= 5.25 && p.z < 5.5) 
		{
			clouds_out[13]->points.push_back(p);
		} else if (p.z >= 5.5 && p.z < 6.0) 
		{
			clouds_out[14]->points.push_back(p);
		} else if (p.z >= 6.0) 
		{
			clouds_out[15]->points.push_back(p);
		}
	}
	*/

	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud_in->points[i];		
		if (std::isnan(p.z)) continue;

		//if (p.z >= 2.0 && p.z < 2.25)
		if (p.z >= 2.0 && p.z < 2.75)
		{
			clouds_out[0]->points.push_back(p);
		//} else if (p.z >= 2.25 && p.z < 2.5) 
		//{
		//	clouds_out[1]->points.push_back(p);
		//} else if (p.z >= 2.5 && p.z < 2.75) 
		//{
		//	clouds_out[2]->points.push_back(p);
		} else if (p.z >= 2.75 && p.z < 3.0) 
		{
			clouds_out[1]->points.push_back(p);
		} else if (p.z >= 3.0 && p.z < 3.25) 
		{
			clouds_out[2]->points.push_back(p);
		} else if (p.z >= 3.25 && p.z < 3.5) 
		{
			clouds_out[3]->points.push_back(p);
		} else if (p.z >= 3.5 && p.z < 3.75) 
		{
			clouds_out[4]->points.push_back(p);
		} else if (p.z >= 3.75 && p.z < 4.0) 
		{
			clouds_out[5]->points.push_back(p);
		} else if (p.z >= 4.0 && p.z < 4.25) 
		{
			clouds_out[6]->points.push_back(p);
		} else if (p.z >= 4.25 && p.z < 4.5) 
		{
			clouds_out[7]->points.push_back(p);
		} else if (p.z >= 4.5 && p.z < 4.75) 
		{
			clouds_out[8]->points.push_back(p);
		} else if (p.z >= 4.75 && p.z < 5.0) 
		{
			clouds_out[9]->points.push_back(p);
		} else if (p.z >= 5.0 && p.z < 5.25) 
		{
			clouds_out[10]->points.push_back(p);
		} else if (p.z >= 5.25 && p.z < 5.5) 
		{
			clouds_out[11]->points.push_back(p);
		} else if (p.z >= 5.5 && p.z < 6.0) 
		{
			clouds_out[12]->points.push_back(p);
		} else if (p.z >= 6.0 && p.z < 6.5) 
		{
			clouds_out[13]->points.push_back(p);
		} else if (p.z >= 6.5 && p.z < 7.0) 
		{
			clouds_out[14]->points.push_back(p);
		} else if (p.z >= 7.0 && p.z < 8.0) 
		{
			clouds_out[15]->points.push_back(p);
		}
	}
	
	for (int i = 0; i < 16; i ++)
	{
		n_points = clouds_out[i]->points.size();
		clouds_out[i]->resize(n_points);
		clouds_out[i]->width = n_points;
		clouds_out[i]->height = 1;
		clouds_out[i]->is_dense = false;
	}
}

bool find_ground_plane(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	float angle,
	float distanceThreshold,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane,
	Eigen::Affine3d &transform_floor
)
{
	pcl::SACSegmentation<pcl::PointXYZRGBL> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE );
	seg.setEpsAngle(  angle * (M_PI/180.0f) );
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(500);
	seg.setDistanceThreshold(distanceThreshold);
	Eigen::Vector3f axis_z = Eigen::Vector3f(0.0,0.0,1.0); //z axis
	seg.setAxis(axis_z);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	int n_plane_points = inliers->indices.size();
	//std::cout << "   find_ground => n_plane_points: " << n_plane_points << std::endl;
	if (n_plane_points < 3)
	{
		//std::cout << "Could not find plane with at least 3 points." << std::endl;
		return false;
	}
	pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);

	extract.filter(*cloud_plane);

	Eigen::Matrix<double, 1, 3> floor_plane_normal_vector, xz_plane_normal_vector;

	floor_plane_normal_vector[0] = coefficients->values[0];
	floor_plane_normal_vector[1] = coefficients->values[1];
	floor_plane_normal_vector[2] = coefficients->values[2];

	xz_plane_normal_vector[0] = 0.0;
	xz_plane_normal_vector[1] = 0.0;
	xz_plane_normal_vector[2] = 1.0;

	Eigen::Vector3d rotation_vector = xz_plane_normal_vector.cross(floor_plane_normal_vector);

	double theta = -atan2(rotation_vector.norm(), xz_plane_normal_vector.dot(floor_plane_normal_vector));
	//std::cout << "   find_ground => theta: " << theta << std::endl;
	if (fabs(theta) > 1.570796325)
	{
		if (theta < 0.0) theta = theta + M_PI;
		else theta = theta - M_PI;
		//std::cout << "   find_ground => adjusted theta: " << theta << std::endl;
	}

	rotation_vector = rotation_vector.normalized();
	Eigen::AngleAxisd rotation_angles = Eigen::AngleAxisd(theta, rotation_vector);

	transform_floor.rotate (rotation_angles);
	//std::cout << "   find_ground => done. " << std::endl;

	return true;
}

void find_likely_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_likely_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	float min_dx, float min_dz,
	pcl::PointXYZRGBL &cluster_min_XYZ, pcl::PointXYZRGBL &cluster_max_XYZ
)
{
	if (cloud->points.size() < 2) return;

	pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> cloud_clusters;
	std::vector<pcl::PointXYZRGBL> mins;
	std::vector<pcl::PointXYZRGBL> maxs;
	std::vector<float> areas;
	//std::cout << "================> find cluster ... " << std::endl;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}

		pcl::PointXYZRGBL min_XYZ, max_XYZ;
		pcl::getMinMax3D(*cloud_cluster, min_XYZ, max_XYZ);

		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
		pcl::PassThrough<pcl::PointXYZRGBL> pass;
		pass.setFilterLimitsNegative (false);
		pass.setInputCloud (cloud_cluster);		
		pass.setFilterFieldName ("z");
		//pass.setFilterLimits (min_XYZ.z, min_XYZ.z + 1.0);
		pass.setFilterLimits (min_XYZ.z, min_XYZ.z + 1.25);
		pass.filter (*cloud_tmp);

		pcl::getMinMax3D(*cloud_tmp, min_XYZ, max_XYZ);

		float dx = max_XYZ.x - min_XYZ.x;
		float dy = max_XYZ.y - min_XYZ.y;
		float dz = max_XYZ.z - min_XYZ.z;
		float area = dx * dy;
		//std::cout << " => cluster-> dx: " << dx << " dy: " << dy << " dz: " << dz  << " area: " << area << " min_XYZ.y: " << min_XYZ.y << " max_XYZ.y: " << max_XYZ.y << " max_XYZ.z: " << max_XYZ.z    << " with " << cloud_cluster->size() << " data points"<< std::endl;
		//std::cout << "   => min_XYZ.y: " << min_XYZ.y << "   max_XYZ.y: " << max_XYZ.y << std::endl;
		//std::cout << "   => min_dx: " <<min_dx << "   min_dz: " << min_dz << std::endl;

		if (dx >= min_dx && dz >= min_dz && min_XYZ.y < 3.5) 
		{
			pass.setFilterLimitsNegative (false);
			pass.setInputCloud (cloud_cluster);		
			pass.setFilterFieldName ("y");
			pass.setFilterLimits (min_XYZ.y, min_XYZ.y + 2.0);
			pass.filter (*cloud_tmp);

			pcl::PointXYZRGBL tmp_min_XYZ, tmp_max_XYZ;
			pcl::getMinMax3D(*cloud_tmp, tmp_min_XYZ, tmp_max_XYZ);

			float tmp_dx = tmp_max_XYZ.x - tmp_min_XYZ.x;
			float tmp_dz = tmp_max_XYZ.z - tmp_min_XYZ.z;

			//std::cout << "      => cluster-> tmp_dx: " << tmp_dx << "   tmp_dz: " << tmp_dz << std::endl;

			if (dx >= min_dx && dz >= min_dz) 
			{
				//std::cout << "      => valid cluster" << std::endl;
				cloud_clusters.push_back(cloud_cluster);
				mins.push_back(min_XYZ);
				maxs.push_back(max_XYZ);
				areas.push_back(area);
			}
		}
	}
	//std::cout << "=> cluster_found: " << cloud_clusters.size() << std::endl;
	if (cloud_clusters.size() == 1)
	{
		*cloud_likely_cluster = *cloud_clusters[0];
		cluster_min_XYZ = mins[0];
		cluster_max_XYZ = maxs[0];
		return;
	}
	
	if (cloud_clusters.size() == 0) return;

	int most_points = 0;
	float nearest_distance = 65.0;
	float max_area = 0.0;
	int selected_index = 0;
	
	//std::cout << "=> find by max area: " << std::endl;
	for (int i = 0; i < cloud_clusters.size(); i ++)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster = cloud_clusters[i];
		pcl::PointXYZRGBL min_XYZ = mins[i];
		
		float area = areas[i];
		
		if (area > max_area)
		{
			selected_index = i;
			max_area = area;
			*cloud_likely_cluster = *cloud_clusters[i];
			cluster_min_XYZ = mins[i];
			cluster_max_XYZ = maxs[i];
			//std::cout << "      => max_area: " << max_area << " cluster_min_XYZ.y: " << cluster_min_XYZ.y << " cluster_min_XYZ.z: " << cluster_min_XYZ.z << std::endl;
		}
	}
	int n_points_cluster = cloud_likely_cluster->points.size();
	float dx_cluster = cluster_max_XYZ.x - cluster_min_XYZ.x;
	//std::cout << "=> n_points_cluster: " << n_points_cluster << "   dx_cluster: " << dx_cluster << std::endl;

	//std::cout << "=> to confirm by max points: " << std::endl;
	for (int i = 0; i < cloud_clusters.size(); i ++)
	{
		if (i != selected_index)
		{
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster = cloud_clusters[i];
			pcl::PointXYZRGBL min_XYZ = mins[i];
			pcl::PointXYZRGBL max_XYZ = maxs[i];

			float dx = max_XYZ.x - min_XYZ.x;
			int n_points = cloud_cluster->points.size();
			//std::cout << "   => n_points: " << n_points << "   dx: " << dx << std::endl;

			if (dx > dx_cluster && n_points > n_points_cluster) // the selected by area, then may be not right selection
			{
				*cloud_likely_cluster = *cloud_clusters[i];
				cluster_min_XYZ = mins[i];
				cluster_max_XYZ = maxs[i];
				selected_index = i;
				//std::cout << "   => cluster_min_XYZ.y: " << cluster_min_XYZ.y << " cluster_min_XYZ.z: " << cluster_min_XYZ.z << std::endl;
			} 
		}
	}
	//std::cout << "=> to confirm by size and distance: " << std::endl;
	for (int i = 0; i < cloud_clusters.size(); i ++)
	{
		if (i != selected_index)
		{
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster = cloud_clusters[i];
			pcl::PointXYZRGBL min_XYZ = mins[i];
			pcl::PointXYZRGBL max_XYZ = maxs[i];

			// here may need to measure OBB length other than raw length, for more accurate, need a try?
			
			float dx = max_XYZ.x - min_XYZ.x;
			int n_points = cloud_cluster->points.size();
			//std::cout << "   => n_points: " << n_points << " dx: " << dx << " dx_cluster: " << dx_cluster << " n_points_cluster: " << n_points_cluster << std::endl;
			//std::cout << "   => min_XYZ.y: " << min_XYZ.y << " cluster_min_XYZ.y: " << cluster_min_XYZ.y << std::endl;
			
			pcl::PointXYZRGBL line_point_s, line_point_e;
			float x_s = 65.0, x_e = -65.0;
			for (int i = 0; i < n_points; i ++)
			{
				pcl::PointXYZRGBL p = cloud_cluster->points[i];
				if (p.x < x_s)
				{
					x_s = p.x;
					line_point_s = p;
				}
				if (p.x > x_e)
				{
					x_e = p.x;
					line_point_e = p;
				}
			}
			
			float slp = (line_point_e.y - line_point_s.y) / (line_point_e.x - line_point_s.x);
			float angle = 180.0 * std::atan(slp) / M_PI;
			//std::cout << "   => verifying near cluster, angle: " << angle << std::endl;

			if (
				//angle < 15.0 &&
				fabs(angle) < 15.0 &&
				min_XYZ.y < (cluster_min_XYZ.y + 0.25) && 				
				(
					dx > 0.5 * dx_cluster
					||
					dx > 1.6
				)
			)
			{
				*cloud_likely_cluster += *cloud_clusters[i];
				//std::cout << "   => cluster added 2 -> dx: " << dx << " n_points: " << n_points << " min_XYZ.y: " << min_XYZ.y << " max_XYZ.y: " << max_XYZ.y << std::endl;
			}
		}
	}
}

void find_likely_clusters(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	float min_dx, 
	float min_dy, 
	float min_dz,
	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> &likely_clusters,
	std::vector<pcl::PointXYZRGBL> &mins,
	std::vector<pcl::PointXYZRGBL> &maxs
)
{
	if (cloud->points.size() < 2) return;

	pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}

		pcl::PointXYZRGBL min_XYZ, max_XYZ;
		pcl::getMinMax3D(*cloud_cluster, min_XYZ, max_XYZ);

		float dx = max_XYZ.x - min_XYZ.x;
		float dy = max_XYZ.y - min_XYZ.y;
		float dz = max_XYZ.z - min_XYZ.z;
		float area = dx * dy;
		//std::cout << "cluster-> dx: " << dx << " dy: " << dy << " dz: " << dz  << " area: " << area << " min_XYZ.y: " << min_XYZ.y << " max_XYZ.y: " << max_XYZ.y << " max_XYZ.z: " << max_XYZ.z    << " with " << cloud_cluster->size() << " data points"<< std::endl;
		//std::cout << "   min_XYZ.y: " << min_XYZ.y << "   max_XYZ.y: " << max_XYZ.y << std::endl;
		//std::cout << "   dx:     " << dx << "       dy:     " << dy << "       dz:     " << dz << std::endl;
		//std::cout << "   min_dx: " << min_dx << "   min_dy: " << min_dy << "   min_dz: " << min_dz << std::endl;
		//std::cout << "   max_XYZ.z: " << max_XYZ.z << std::endl;

		if (dx >= min_dx && dy > min_dy && dz >= min_dz) 
		{	
			likely_clusters.push_back(cloud_cluster);
			mins.push_back(min_XYZ);
			maxs.push_back(max_XYZ);			
		}
	}
}

void find_largest_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_largest_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize
)
{
	int n_cloud_points = cloud->points.size();
	if (cloud->points.size() < 2) return;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGBL>);

	pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	//std::cout << "============> find_largest_cluster -> 2 " << std::endl;

	int max_points = 0;
	float max_dx = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}
		int n_pts = cloud_cluster->points.size();

		pcl::PointXYZRGBL min_XYZ, max_XYZ;
		pcl::getMinMax3D(*cloud_cluster, min_XYZ, max_XYZ);

		float dx = max_XYZ.x - min_XYZ.x;
		//std::cout << "   n_pts: " << n_pts << " max_points: " << max_points << " dx: " << dx << " max_dx: " << max_dx << " max_XYZ.z: " << max_XYZ.z << std::endl;
		if ((n_pts > max_points && dx > max_dx) || dx > 1.5 * max_dx) 
		{
			//std::cout << "      ... " << std::endl;
			max_dx = dx;
			max_points = n_pts;
			*cloud_output = *cloud_cluster;
		}
	}
	*cloud_largest_cluster = *cloud_output;
}

void find_nearest_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_nearest_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	float min_dx, float min_dy
)
{
	int n_cloud_points = cloud->points.size();
	if (cloud->points.size() < 2) return;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGBL>);

	pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	//std::cout << "============> find_largest_cluster -> 2 " << std::endl;

	float min_y = 100.0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}
		int n_pts = cloud_cluster->points.size();

		pcl::PointXYZRGBL min_XYZ, max_XYZ;
		pcl::getMinMax3D(*cloud_cluster, min_XYZ, max_XYZ);
		float dx = max_XYZ.x - min_XYZ.x;
		float dy = max_XYZ.y - min_XYZ.y;

		if (min_XYZ.y < min_y && dx > min_dx && dy > min_dy) 
		{
			//std::cout << "      ... " << std::endl;
			min_y = min_XYZ.y;
			*cloud_output = *cloud_cluster;
		}
	}
	*cloud_nearest_cluster = *cloud_output;
}

bool find_edge_points(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
	float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
	float offset_z, float thick,
	bool near_edge,
	bool far_edge
)
{
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

	*_cloud = *cloud;

	cloud_edge->points.clear();
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_projected_downsampled(new pcl::PointCloud<pcl::PointXYZRGBL>);
	int n_points = _cloud->points.size();

	float cuf_off_min_z = min_z + offset_z; 
	float cuf_off_max_z = cuf_off_min_z + thick;
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = _cloud->points[i];
		if (p.z > cuf_off_min_z && p.z < cuf_off_max_z)
		{
			p.z = max_z;
			cloud_projected->points.push_back(p);
		}
	}

	//std::cout << "   cloud_projected (before downsample): " << cloud_projected->points.size() << std::endl;
	pcl::UniformSampling<pcl::PointXYZRGBL> uniform_sampling;
	uniform_sampling.setInputCloud(cloud_projected);
	uniform_sampling.setRadiusSearch(0.125);
	uniform_sampling.filter (*cloud_projected_downsampled);
	//std::cout << "   cloud_projected_downsampled (after downsample): " << cloud_projected_downsampled->points.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_step(new pcl::PointCloud<pcl::PointXYZRGBL>);

	float step = 0.125;
	int n = ceil((max_x - min_x) / step);
	pcl::PassThrough<pcl::PointXYZRGBL> pass;
	pass.setFilterLimitsNegative (false);
	pass.setInputCloud (cloud_projected_downsampled);

	pcl::PointXYZRGBL min_step, max_step;

	pass.setFilterFieldName ("x");
	for (int i = 0; i < n; i ++)
	{
		pass.setFilterLimits (min_x + i * step, min_x + (i + 1) * step);
		pass.filter (*cloud_step);

		if (cloud_step->points.size() > 1)
		{
			pcl::getMinMax3D(*cloud_step, min_step, max_step);

			min_step.r = 255;
			max_step.r = 255;
			min_step.z = max_z;
			max_step.z = max_z;
			if (near_edge)
			{
				cloud_edge->points.push_back(min_step);
			}
			if (far_edge)
			{
				cloud_edge->points.push_back(max_step);
			}			
		}
	}

	return true;
}

bool find_edge_points_y(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_left_edge,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_right_edge,
	float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
	float offset_z, float thick
)
{
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
	*_cloud = *cloud;

	cloud_left_edge->points.clear();
	cloud_right_edge->points.clear();
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_projected_downsampled(new pcl::PointCloud<pcl::PointXYZRGBL>);
	int n_points = _cloud->points.size();

	float cuf_off_min_z = min_z + offset_z; 
	float cuf_off_max_z = cuf_off_min_z + thick;
	//std::cout << "==> cuf_off_min_z: " << cuf_off_min_z << " cuf_off_max_z: " << cuf_off_max_z << std::endl;
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = _cloud->points[i];
		if (p.z > cuf_off_min_z && p.z < cuf_off_max_z)
		{
			p.z = max_z;
			cloud_projected->points.push_back(p);
		}
	}

	//std::cout << "   cloud_projected (before downsample): " << cloud_projected->points.size() << std::endl;
	pcl::UniformSampling<pcl::PointXYZRGBL> uniform_sampling;
	uniform_sampling.setInputCloud(cloud_projected);
	uniform_sampling.setRadiusSearch(0.125);
	uniform_sampling.filter (*cloud_projected_downsampled);
	//std::cout << "   cloud_projected_downsampled (after downsample): " << cloud_projected_downsampled->points.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_step(new pcl::PointCloud<pcl::PointXYZRGBL>);

	float step = 0.125;
	int n = ceil((max_y - min_y) / step);
	pcl::PassThrough<pcl::PointXYZRGBL> pass;
	pass.setFilterLimitsNegative (false);
	pass.setInputCloud (cloud_projected_downsampled);

	pcl::PointXYZRGBL min_step, max_step;

	pass.setFilterFieldName ("y");
	for (int i = 0; i < n; i ++)
	{
		pass.setFilterLimits (min_y + i * step, min_y + (i + 1) * step);
		pass.filter (*cloud_step);

		if (cloud_step->points.size() > 1)
		{
			pcl::getMinMax3D(*cloud_step, min_step, max_step);

			min_step.r = 255;
			max_step.r = 255;
			min_step.z = max_z;
			max_step.z = max_z;
			
			cloud_left_edge->points.push_back(min_step);			
			cloud_right_edge->points.push_back(max_step);
		}
	}

	return true;
}

void extract_line_points(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
	float distanceThreshold,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge_remains
)
{
	//std::cout << "cloud_edge->points.size(): " << cloud_edge->points.size() << std::endl;
	if (cloud_edge->points.size() < 5) return;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_edge(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_edge_remains(new pcl::PointCloud<pcl::PointXYZRGBL>);
	*_cloud_edge = *cloud_edge;
	pcl::SACSegmentation<pcl::PointXYZRGBL> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(500);
	seg.setDistanceThreshold(distanceThreshold);
	seg.setInputCloud(_cloud_edge);
	seg.segment(*inliers, *coefficients);
	//std::cout << "inliers->indices.size(): " << inliers->indices.size() << std::endl;
	if (inliers->indices.size() < 3)
	{
		//std::cout << "Warning: could not find line." << std::endl;
		return;
	}

	pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
	extract.setInputCloud(_cloud_edge);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_line);

	extract.setNegative(true);

	extract.filter(*_cloud_edge_remains);
	*cloud_edge_remains = *_cloud_edge_remains;
	//std::cout << "cloud_edge_remains->points.size(): " << cloud_edge_remains->points.size() << std::endl;
}
bool validate_cluster_edge_line(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
	float cluster_min_y,
	float min_length,
	float max_length,
	float max_slope,
	float min_y_shift,
	float max_y_shift,
	float &line_length,
	float &line_slope
)
{
	bool line_valid = false;
	pcl::PointXYZRGBL line_min_XYZ, line_max_XYZ;

	pcl::getMinMax3D(*cloud_line, line_min_XYZ, line_max_XYZ);
	float line_dx = line_max_XYZ.x - line_min_XYZ.x;
	float line_dy = line_max_XYZ.y - line_min_XYZ.y;

	line_slope = fabs(line_dy / line_dx);
	float distance_2_near_edge = fabs(line_min_XYZ.y - cluster_min_y);

	pcl::PointXYZRGBL line_min_OBB, line_max_OBB, position;
	Eigen::Matrix3f rotational_matrix; 
	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> f;
	f.setInputCloud(cloud_line);
	f.compute();
	f.getOBB(line_min_OBB, line_max_OBB, position, rotational_matrix);

	line_length = line_max_OBB.x - line_min_OBB.x;

	//std::cout << "         ==> line_length: " << line_length << " line_length: " << line_length << " line_slope: " << line_slope << " max_slope: " << max_slope << " d: " << distance_2_near_edge << " min_y_shift: " << min_y_shift << " max_y_shift: " << max_y_shift << std::endl;
	//std::cout << "         ==> line_min_XYZ.y: "<< line_min_XYZ.y << " line_min_XYZ.z: " << line_min_XYZ.z << std::endl;
	//std::cout << "         ==> min_length: "<< min_length << " max_length: " << max_length << std::endl;
	//std::cout << "         ==> distance_2_near_edge: "<< distance_2_near_edge << " min_y_shift: " << min_y_shift << " max_y_shift: " << max_y_shift << std::endl;
	if (line_length < min_length || line_length > max_length || line_slope > max_slope || distance_2_near_edge < min_y_shift || distance_2_near_edge > max_y_shift)
	{
		//std::cout << "         ==> invalid ... "<< std::endl;
		line_valid = false;
	} else 
	{
		//std::cout << "         ==> valid ... "<< std::endl;
		line_valid = true;
	}

	return line_valid;
}

float find_longest_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_longest_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize
)
{
	if (cloud->points.size() < 2) return 0;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _longest_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);

	pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	float max_dx = 0.0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}

		//pcl::PointXYZRGBL min_XYZ, max_XYZ;
		//pcl::getMinMax3D(*cloud_cluster, min_XYZ, max_XYZ);
		pcl::PointXYZRGBL min_OBB, max_OBB, position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB; 
	
		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> f;
		f.setInputCloud(cloud_cluster);
		f.compute();
		f.getOBB(min_OBB, max_OBB, position_OBB, rotational_matrix_OBB);

		//float dx = max_XYZ.x - min_XYZ.x;
		float dx = max_OBB.x - min_OBB.x;
		//std::cout << "------ cluster-> dx: " << dx << " with " << cloud_cluster->size() << " data points"<< std::endl;

		if (dx > max_dx)
		{
			max_dx = dx;
			*_longest_cluster = *cloud_cluster;
			//std::cout << "------ longest cluster-> dx: " << max_dx << " with " << cloud_cluster->size() << " data points"<< std::endl;
		}
	}
	*cloud_longest_cluster = *_longest_cluster;

	return max_dx;
}
bool find_top_line_OBB(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_OBB,
	float wall_length, float wall_height, float wall_min_z,
	float step, float min_length_tolerance, 
	bool ignore_distance, 
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_OBB,
	pcl::PointXYZRGBL wall_min_XYZ, pcl::PointXYZRGBL wall_max_XYZ
)
{
	bool top_line_found = false;

	int n = ceil(wall_height / step); 
	//std::cout << "===> find_top_line_OBB ... " << std::endl;

	pcl::PassThrough<pcl::PointXYZRGBL> pass;
	pass.setFilterLimitsNegative (false);
	pass.setInputCloud (cloud_wall_OBB);
	pass.setFilterFieldName ("z");

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_step(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointXYZRGBL minXYZ, maxXYZ;
	float max_dx = 0.0;
	float min_slope = 100.0;
	for (int i = 0; i < n; i ++)
	{
		pass.setFilterLimits (wall_min_z + (i - 0.5) * step, wall_min_z + (i + 1.5) * step);
		pass.filter (*cloud_step);
		//std::cout << "   => cloud_step: " << cloud_step->points.size() << std::endl;

		if (cloud_step->points.size() > 1)
		{
			int n_step_points = cloud_step->points.size();
			//std::cout << "         => raw n_step_points: " << n_step_points << std::endl;
			if (n_step_points < 3) 
			{
				break;
			}

			double mcx = 0.0, mcy = 0.0, mcz = 0.0;
			for (int i = 0; i < n_step_points; i++ )
			{
				pcl::PointXYZRGBL p = cloud_step->points[i];
				mcx += p.x;
				mcy += p.y;
				mcz += p.z;
			}
			float cy = mcy / n_step_points;
			for (int i = 0; i < n_step_points; i++ )
			{
				pcl::PointXYZRGBL &p = cloud_step->points[i];
				p.y = cy;
			}

			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_remains(new pcl::PointCloud<pcl::PointXYZRGBL>);
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
			extract_line_points(cloud_step, cloud_tmp_line, 0.125, cloud_tmp_remains);
			int n_tmp_line_points = cloud_tmp_line->points.size();
			//std::cout << "         => raw n_tmp_line_points: " << n_tmp_line_points << std::endl;

			if (n_tmp_line_points < 2) continue;

			float length_of_line = find_longest_cluster(cloud_tmp_line, cloud_tmp_line, 0.5, 10, 1000); 
			n_tmp_line_points = cloud_tmp_line->points.size();
			//std::cout << "         => clustered n_tmp_line_points: " << n_tmp_line_points << std::endl;
			
			*cloud_step = *cloud_tmp_line;

			//std::cout << "   => clean cloud_step: " << cloud_step->points.size() << std::endl;

			if (n_tmp_line_points > 0)
			{
				pcl::getMinMax3D(*cloud_step, minXYZ, maxXYZ);
				float dx = maxXYZ.x - minXYZ.x;
				//std::cout << "      => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << " wall_length: " << wall_length << " max_dx: " << max_dx << " points: " << cloud_step->points.size() << std::endl;
				//std::cout << "      => min_length_tolerance: " << min_length_tolerance << " wall_length: " << wall_length << " dx: " << dx << std::endl;

				if (dx > min_length_tolerance * wall_length || (dx > 1.5 && dx > 0.5 * wall_length)) // adding absolute length of 2.0 meters for valid length
				{
					float dz = maxXYZ.z - minXYZ.z;
					float slope = dz / dx;
					//std::cout << "               => slope: " << slope << " min_slope: " << min_slope << std::endl;
					if (slope < 1.25 * min_slope || slope < 0.25)
					{
						min_slope = slope;
						if (dx > max_dx) max_dx = dx;
						*cloud_top_line_OBB = *cloud_step;
						top_line_found = true;
						//std::cout << "                  => valid top found => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << " slope: " << slope << std::endl;
					}
				}
			}
		}
	}
	
	// optimize top line
	pcl::PointXYZRGBL top_min_XYZ, top_max_XYZ;
	pcl::getMinMax3D(*cloud_top_line_OBB, top_min_XYZ, top_max_XYZ);

	pcl::CropBox<pcl::PointXYZRGBL> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(top_min_XYZ.x, wall_min_XYZ.y,   top_max_XYZ.z, 1.0));
	boxFilter.setMax(Eigen::Vector4f(top_max_XYZ.x, wall_max_XYZ.y,   wall_max_XYZ.z, 1.0)); 
	boxFilter.setInputCloud(cloud_wall_OBB);

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_opt_wall(new pcl::PointCloud<pcl::PointXYZRGBL>);

	boxFilter.filter(*cloud_opt_wall);

	pcl::PointXYZRGBL wall_opt_min_XYZ, wall_opt_max_XYZ;
	pcl::getMinMax3D(*cloud_opt_wall, wall_opt_min_XYZ, wall_opt_max_XYZ);
	float wall_opt_length = top_max_XYZ.x - top_min_XYZ.x; // should use the detected top line length
	float wall_opt_height = wall_opt_max_XYZ.z - wall_opt_min_XYZ.z;

	n = ceil(wall_opt_height / step); 
	//std::cout << "=> opt n: " << n << " step: " << step << std::endl;

	pass.setFilterLimitsNegative (false);
	pass.setInputCloud (cloud_opt_wall);
	pass.setFilterFieldName ("z");
	//std::cout << "   ===> verify top line ... " << std::endl;
	max_dx = 0.0;
	for (int i = 0; i < n; i ++)
	{
		pass.setFilterLimits (wall_opt_min_XYZ.z + (i - 0.5) * step, wall_opt_min_XYZ.z + (i + 1.5) * step);
		pass.filter (*cloud_step);
		//std::cout << "   => cloud_step: " << cloud_step->points.size() << std::endl;

		if (cloud_step->points.size() > 1)
		{
			int n_step_points = cloud_step->points.size();
			//std::cout << "         => raw n_step_points: " << n_step_points << std::endl;
			if (n_step_points < 3) 
			{
				break;
			}

			double mcx = 0.0, mcy = 0.0, mcz = 0.0;
			for (int i = 0; i < n_step_points; i++ )
			{
				pcl::PointXYZRGBL p = cloud_step->points[i];
				mcx += p.x;
				mcy += p.y;
				mcz += p.z;
			}
			float cy = mcy / n_step_points;
			for (int i = 0; i < n_step_points; i++ )
			{
				pcl::PointXYZRGBL &p = cloud_step->points[i];
				p.y = cy;
			}

			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_remains(new pcl::PointCloud<pcl::PointXYZRGBL>);
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
			extract_line_points(cloud_step, cloud_tmp_line, 0.125, cloud_tmp_remains);
			int n_tmp_line_points = cloud_tmp_line->points.size();
			//std::cout << "         => raw n_tmp_line_points: " << n_tmp_line_points << std::endl;

			if (n_tmp_line_points < 2) continue;

			//float length_of_line = find_longest_cluster(cloud_tmp_line, cloud_tmp_line, 0.5, 10, 1000); 
			float length_of_line = find_longest_cluster(cloud_tmp_line, cloud_tmp_line, 1.0, 5, 1000); 
			n_tmp_line_points = cloud_tmp_line->points.size();
			//std::cout << "         => clustered n_tmp_line_points: " << n_tmp_line_points << std::endl;
			
			*cloud_step = *cloud_tmp_line;

			//std::cout << "   => clean cloud_step: " << cloud_step->points.size() << std::endl;

			if (n_tmp_line_points > 0)
			{
				pcl::getMinMax3D(*cloud_step, minXYZ, maxXYZ);
				float dx = maxXYZ.x - minXYZ.x;
				//std::cout << "      opt  => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << " wall_opt_length: " << wall_opt_length << " max_dx: " << max_dx << " points: " << cloud_step->points.size() << std::endl;
				//std::cout << "      opt  => min_length_tolerance: " << min_length_tolerance << " wall_opt_length: " << wall_opt_length << " dx: " << dx << std::endl;
				//std::cout << "      opt  => 0.5 * wall_opt_length: " << 0.5 * wall_opt_length << std::endl;
				if (dx > 1.5 && dx > 0.8 * wall_opt_length)
				{
						if (dx > max_dx) max_dx = dx;
						*cloud_top_line_OBB += *cloud_step;
						//std::cout << "               opt   => valid top found => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << std::endl;
				} else if (dx > 1.5) 
				{
					*cloud_top_line_OBB += *cloud_step;
					//std::cout << "               opt   => optimization of top height => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << std::endl;
				} else 
				{
					break;
				}
			}
		}
	}
	
	return top_line_found;
}
bool find_top_lines_OBB(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster_OBB,
	pcl::PointXYZRGBL cluster_OBB_min_XYZ, pcl::PointXYZRGBL cluster_OBB_max_XYZ,
	float thick, float min_length_tolerance, float default_width, 
	pcl::PointXYZRGBL line_OBB_min_XYZ, pcl::PointXYZRGBL line_OBB_max_XYZ,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_1_OBB,
	bool optimize_top_line,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_1_OBB,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_2_OBB
)
{
	bool top_line_found = false;

	//std::cout << "   => thick: " << thick << " min_length_tolerance: " << min_length_tolerance << " optimize_top_line: " << optimize_top_line << std::endl;	
	pcl::PassThrough<pcl::PointXYZRGBL> pass;
	pass.setFilterLimitsNegative (false);
	//std::cout << "==> cluster_OBB_min_XYZ.y: " << cluster_OBB_min_XYZ.y << std::endl;

	pcl::CropBox<pcl::PointXYZRGBL> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-65.0, cluster_OBB_min_XYZ.y,         -65.0, 1.0));
	boxFilter.setMax(Eigen::Vector4f( 65.0, cluster_OBB_min_XYZ.y + thick,   cluster_OBB_max_XYZ.z, 1.0)); 
	boxFilter.setInputCloud(cloud_closest_cluster_OBB);
	boxFilter.filter(*cloud_wall_1_OBB);

	//std::cout << "   => cloud_wall_1_OBB: " << cloud_wall_1_OBB->points.size() << std::endl;
	if (cloud_wall_1_OBB->points.size() == 0) return false;

	//std::cout << "   ===> before cloud_wall_1_OBB->points: " << cloud_wall_1_OBB->points.size() << std::endl;

	find_largest_cluster(cloud_wall_1_OBB, cloud_wall_1_OBB, 0.175, 15, 10000); // 0.5 to prevent break the wall which should be connected

	//std::cout << "   ===> after cloud_wall_1_OBB->points: " << cloud_wall_1_OBB->points.size() << std::endl;

	pcl::PointXYZRGBL wall_1_min_XYZ, wall_1_max_XYZ;
	pcl::getMinMax3D(*cloud_wall_1_OBB, wall_1_min_XYZ, wall_1_max_XYZ);
	float wall_1_length = wall_1_max_XYZ.x - wall_1_min_XYZ.x;
	float wall_1_height = wall_1_max_XYZ.x - wall_1_min_XYZ.x;
	//std::cout << "   ===> wall_1_length: " << wall_1_length << std::endl;
	float cluster_length = cluster_OBB_max_XYZ.x - cluster_OBB_min_XYZ.x;
	//std::cout << "   ===> cluster_length: " << cluster_length << std::endl;
	if (wall_1_length < min_length_tolerance * cluster_length)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);

		boxFilter.setMin(Eigen::Vector4f(-65.0, cluster_OBB_min_XYZ.y + thick,                       -65.0, 1.0));
		boxFilter.setMax(Eigen::Vector4f( 65.0, cluster_OBB_min_XYZ.y + thick * 2,   cluster_OBB_max_XYZ.z, 1.0)); 
		boxFilter.filter(*cloud_tmp);
		if (cloud_tmp->points.size() > 2) 
		{
			find_largest_cluster(cloud_tmp, cloud_tmp, 0.5, 15, 10000);
			pcl::PointXYZRGBL tmp_min_XYZ, tmp_max_XYZ;
			pcl::getMinMax3D(*cloud_tmp, tmp_min_XYZ, tmp_max_XYZ);
			float tmp_length = tmp_max_XYZ.x - tmp_min_XYZ.x;

			if (tmp_length > wall_1_length)
			{
				wall_1_min_XYZ = tmp_min_XYZ;
				wall_1_max_XYZ = tmp_max_XYZ;
				*cloud_wall_1_OBB = *cloud_tmp;

				wall_1_length = wall_1_max_XYZ.x - wall_1_min_XYZ.x;
				wall_1_height = wall_1_max_XYZ.x - wall_1_min_XYZ.x;
				//std::cout << "      ===> wall_1_length: " << wall_1_length << std::endl;
				//std::cout << "      ===> cluster_length: " << cluster_length << std::endl;
				//if (wall_1_length < min_length_tolerance * cluster_length) return false; // in case just seen part of near edge, it is still ok
			}
		}
	}

	// optimize the top line
	if (optimize_top_line)
	{
		top_line_found = find_top_line_OBB(cloud_wall_1_OBB, wall_1_length, wall_1_height, wall_1_min_XYZ.z, 0.125, min_length_tolerance, false, cloud_top_line_1_OBB, wall_1_min_XYZ, wall_1_max_XYZ); // 0.25 step -> line detected not good
		if (! top_line_found) return false;
	} else 
	{
		pass.setFilterLimitsNegative (false);
		pass.setInputCloud (cloud_wall_1_OBB);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (wall_1_min_XYZ.z, line_OBB_max_XYZ.z);
		
		pass.filter (*cloud_top_line_1_OBB);

		top_line_found = true;
	}
	//std::cout << "1    => top_line_found: " << top_line_found << std::endl;
	//std::cout << "   => cloud_top_line_1_OBB: " << cloud_top_line_1_OBB->points.size() << std::endl;

	pcl::PointXYZRGBL top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ;
	pcl::getMinMax3D(*cloud_top_line_1_OBB, top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ);

	pcl::PointXYZRGBL wall_2_point_1;

	wall_2_point_1.x = 0.5 * (top_line_1_OBB_min_XYZ.x + top_line_1_OBB_max_XYZ.x);
	wall_2_point_1.y = top_line_1_OBB_min_XYZ.y + default_width; // 2.6 m is the standard width of truck
	wall_2_point_1.z = top_line_1_OBB_max_XYZ.z;
	wall_2_point_1.r = 255;
	wall_2_point_1.g = 255;
	wall_2_point_1.b = 255;
	//std::cout << "      initial wall_2_point_1.y: " << wall_2_point_1.y << std::endl;

	// to find wall far side
	int attempt = 0;
	bool wall_2_found = false;
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_2(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
	//std::cout << "   thick: " << thick << std::endl;
	pass.setInputCloud (cloud_closest_cluster_OBB);
	pass.setFilterFieldName ("y");
	while(attempt < 4) // start from 2.6m, to 3.7m, needs 4 times
	{
		float y1 = cluster_OBB_min_XYZ.y + 1.5 + thick * (attempt - 0.5);
		float y2 = cluster_OBB_min_XYZ.y + 1.5 + thick * (attempt + 1);
		//std::cout << "   y1: " << y1 << "   y2: " << y2 << std::endl;
		pass.setFilterLimits (y1, y2);		
		pass.filter (*cloud_tmp);
		//std::cout << "    far side => cloud_tmp: " << cloud_tmp->points.size() << std::endl;

		if (cloud_tmp->points.size() == 0)
		{
			//std::cout << "   no wall_2_found, use guess value. " << std::endl;
			break;			
		}

		pcl::PointXYZRGBL wall_min_XYZ, wall_max_XYZ;
		pcl::getMinMax3D(*cloud_tmp, wall_min_XYZ, wall_max_XYZ);
		float wall_2_length = wall_max_XYZ.x - wall_min_XYZ.x;
		float width_to_line_1 = wall_min_XYZ.y - top_line_1_OBB_min_XYZ.y;
		//std::cout << "      attempt: " << attempt << "  far side => wall_2_length: " << wall_2_length << " wall_1_length: " << wall_1_length << " width_to_line_1: " << width_to_line_1 << std::endl;
		if (width_to_line_1 <= default_width && (attempt == 0 || wall_2_length >= min_length_tolerance * wall_1_length)) // for now, do not support oversized truck, which is 3.7m, restricked to 2.6m
		{
			//std::cout << "         valid => attempt: " << attempt << std::endl;
			*cloud_wall_2 = *cloud_tmp;
			wall_2_found = true;
		} else if (attempt > 0 && wall_2_length < min_length_tolerance * wall_1_length)
		{
			*cloud_wall_2 = *cloud_tmp; // shold take this last one for more accuracy
			wall_2_found = true;
			//std::cout << "      end of looking for wall 2. " << std::endl;
			break;
		}

		attempt ++;
	}
	//std::cout << "      wall_2_found: " << wall_2_found << std::endl;
	if (wall_2_found)
	{
		pcl::PointXYZRGBL wall_min_XYZ, wall_max_XYZ;
		pcl::getMinMax3D(*cloud_wall_2, wall_min_XYZ, wall_max_XYZ);

		wall_2_point_1.y = wall_min_XYZ.y;

		pass.setInputCloud (cloud_wall_2);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (top_line_1_OBB_max_XYZ.z - 0.25, top_line_1_OBB_max_XYZ.z + 0.25);		
		pass.filter (*cloud_tmp);
		
		if (cloud_tmp->points.size() > 0)
		{
			pcl::PointXYZRGBL wall_min_XYZ, wall_max_XYZ;
			pcl::getMinMax3D(*cloud_tmp, wall_min_XYZ, wall_max_XYZ);
		}

		if (
			((wall_2_point_1.y - top_line_1_OBB_min_XYZ.y) < default_width - 0.2) 
			||
			((wall_2_point_1.y - top_line_1_OBB_min_XYZ.y) > default_width + 0.2)
		)
		{
			//std::cout << "      detected width too narraw, use default: " << default_width - 0.2 << std::endl;
			wall_2_point_1.y = top_line_1_OBB_min_XYZ.y + default_width;
		}
	}

	//std::cout << "   final wall_2_point_1.y: " << wall_2_point_1.y << std::endl;
	cloud_top_line_2_OBB->points.push_back(wall_2_point_1);

	return top_line_found;
}
bool compute_vertex(
	pcl::PointXYZRGBL min_OBB, pcl::PointXYZRGBL max_OBB,
	float z,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex_OBB
)
{
	//std::cout << "      compute_vertex ... 1 " << std::endl;
	cloud_vertex_OBB->points.clear();

	float x1 = min_OBB.x;
	float y1 = min_OBB.y;
	float x2 = max_OBB.x;
	float y2 = max_OBB.y;

	pcl::PointXYZRGBL p1, p2, p3, p4;
	p1.x = x1;
	p1.y = y1;
	p1.z = z;
	p1.r = 255;
	p1.g = 255;
	p1.b = 255;

	p2.x = x1;
	p2.y = y2;
	p2.z = z;
	p2.r = 255;
	p2.g = 255;
	p2.b = 255;

	p3.x = x2;
	p3.y = y1;
	p3.z = z;
	p3.r = 255;
	p3.g = 255;
	p3.b = 255;

	p4.x = x2;
	p4.y = y2;
	p4.z = z;
	p4.r = 255;
	p4.g = 255;
	p4.b = 255;

	cloud_vertex_OBB->points.push_back(p1);
	cloud_vertex_OBB->points.push_back(p2);
	cloud_vertex_OBB->points.push_back(p3);
	cloud_vertex_OBB->points.push_back(p4);
	
	return true;
}
void compute_falling_position_OPP(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_OBB,
	float max_z,
	pcl::PointXYZRGBL min_volume_OBB, pcl::PointXYZRGBL max_volume_OBB,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_OBB_falling
)
{
	float fall_min_z = std::max(max_z + 0.5, max_volume_OBB.z - 0.5);

	pcl::PassThrough<pcl::PointXYZRGBL> pass;
	pass.setFilterLimitsNegative (false);
	pass.setInputCloud (cloud_OBB);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (fall_min_z, max_volume_OBB.z);
	pass.filter (*cloud_OBB_falling);
}
bool compute_volume_voxel_OPP(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_OBB,
	float z,
	pcl::PointXYZRGBL min_volume_OBB, pcl::PointXYZRGBL max_volume_OBB,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_voxel_OBB_avaiable
)
{
	float dx = max_volume_OBB.x - min_volume_OBB.x;
	float dy = max_volume_OBB.y - min_volume_OBB.y;
	float volume_raw = dx * dy * 1.0;

	float step_x = 0.125;
	float step_y = 0.125;

	int n_grid_x = (int) ceil(dx / step_x);
	int n_grid_y = (int) ceil(dy / step_y);

	step_x = dx / n_grid_x;
	step_y = dy / n_grid_y;

	pcl::PointXYZRGBL min_XYZ, max_XYZ;
	pcl::CropBox<pcl::PointXYZRGBL> boxFilter;
	for (int nx = 1; nx < n_grid_x - 1; nx ++)
	{
		float x1 = min_volume_OBB.x + nx * step_x - 0.25;
		float x2 = x1 + step_x + 0.25;
		//std::cout << "   ===> volume x1: " << x1 << " x2: " << x2 << std::endl;

		for (int ny = 1; ny < n_grid_y - 2; ny ++)
		{
			float y1 = min_volume_OBB.y + ny * step_y - 0.25;
			float y2 = y1 + step_y + 0.25;

			boxFilter.setMin(Eigen::Vector4f(x1, y1, -16.0, 1.0));
			boxFilter.setMax(Eigen::Vector4f(x2, y2,  16.0, 1.0));
			boxFilter.setInputCloud(cloud_OBB);
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGBL>);
			boxFilter.filter(*cloud_voxel);

			pcl::PointXYZRGBL p;
			p.x = 0.5 * (x1 + x2);
			p.y = 0.5 * (y1 + y2);
			p.r = 0;
			p.g = 255;
			p.b = 0;
			p.z = z;

			//std::cout << "   ===> cloud_voxel->points.size(): " << cloud_voxel->points.size() << std::endl;

			if (cloud_voxel->points.size() > 0)
			{
			} else {
				cloud_voxel_OBB_avaiable->points.push_back(p);
			}
		}
	}

	return true;
}
bool find_best_dest(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_best_dest,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	pcl::PointXYZRGBL vertex_OBB_min_XYZ, 
	pcl::PointXYZRGBL vertex_OBB_max_XYZ,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster_OBB
)
{
	pcl::PointXYZRGBL min_XYZ, max_XYZ;
	pcl::getMinMax3D(*cloud, min_XYZ, max_XYZ);

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
	int n_points = cloud->points.size();
	float z_cut = 0.5 * (min_XYZ.z + max_XYZ.z);
	if (z_cut < (min_XYZ.z + 0.25)) z_cut = min_XYZ.z + 0.25;
	if (z_cut > max_XYZ.z) z_cut = max_XYZ.z;
	for (int i = 0; i < n_points; i ++)
	{
		pcl::PointXYZRGBL p = cloud->points[i];
		if (p.z < z_cut)
		{
			p.z = min_XYZ.z;
			cloud_tmp->points.push_back(p);
		}
	}
	n_points = cloud_tmp->points.size();
	//std::cout << "   ===============> n_points: " << n_points << std::endl;
	
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_empty(new pcl::PointCloud<pcl::PointXYZRGBL>);
	compute_volume_voxel_OPP(cloud_cluster_OBB, min_XYZ.z, vertex_OBB_min_XYZ, vertex_OBB_max_XYZ, cloud_available_empty);
	//std::cout << "   ===============> cloud_available_empty: " << cloud_available_empty->points.size() << std::endl;
	*cloud_tmp += *cloud_available_empty;

	n_points = cloud_tmp->points.size();
	cloud_tmp->resize(n_points);
	cloud_tmp->width = n_points;
	cloud_tmp->height = 1;
	cloud_tmp->is_dense = false;

	bool found = false;

	if (n_points > 2) 
	{
		pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
		tree->setInputCloud(cloud_tmp);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
		ec.setClusterTolerance(clusterTolerance);
		ec.setMinClusterSize(minClusterSize);
		ec.setMaxClusterSize(maxClusterSize);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_tmp);
		ec.extract(cluster_indices);

		int max_n_points = 0;
		float max_dx = 0.0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cloud_cluster->points.push_back(cloud_tmp->points[*pit]);
			}

			int n_points = cloud_cluster->points.size();
			//std::cout << "------ distance: " << distance << " dx: " << dx << " dy: " << dy << " points: " << n_points << " area: " << area << std::endl;
			//std::cout << " points: " << n_points << std::endl;

			if (n_points > max_n_points)
			{
				max_n_points = n_points;
				*cloud_best_dest = *cloud_cluster;
				found = true;
				//std::cout << "------ selected" << std::endl;
			}
		}
	} else {
		*cloud_best_dest = *cloud_tmp;
	}

	return found;
}

void add_center_ref(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, float distance)
{
	pcl::PointXYZRGBL p;
	p.x = 0;
	p.y = 0;
	p.z = distance;
	
	p.r = 255;
	p.g = 0;
	p.b = 0;
	cloud->points.push_back(p);
}

void filter_by_stat(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_input,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_output,
	int meanK,
	int stddevMulThresh
)
{
	if (cloud_input->points.size() == 0) return;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(stddevMulThresh);
	sor.setInputCloud(cloud_input);
	sor.filter(*cloud_output);
}

void filter_by_clustering(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_input,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_output,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize
)
{
	//std::cout << "===========>filter_by_clustering-- cloud_input->points.size(): " << cloud_input->points.size() << std::endl;
	//if (cloud_input->points.size() == 0) return;
	if (cloud_input->points.size() > 1) 
	{	
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_result(new pcl::PointCloud<pcl::PointXYZRGBL>);

		pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>);
		tree->setInputCloud(cloud_input);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
		ec.setClusterTolerance(clusterTolerance);
		ec.setMinClusterSize(minClusterSize);
		ec.setMaxClusterSize(maxClusterSize);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_input);
		ec.extract(cluster_indices);

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cloud_cluster->points.push_back(cloud_input->points[*pit]);
			}
			*cloud_tmp_result += *cloud_cluster;
		}
		*cloud_output = *cloud_tmp_result;
	} else if (cloud_input->points.size() == 1) 
	{
		*cloud_output = *cloud_input;
	} else 
	{
		cloud_output->points.clear();
	}
	//std::cout << "===========>filter_by_clustering-- done. " << std::endl;
}

bool find_likely_line(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cluster,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr center,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr edge,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr line,
	float dz,
	bool near_edge,
	bool far_edge,
	float distanceTolerance,
	float min_dx,
	float max_dx,
	float max_slope,
	float min_y_shift,
	float max_y_shift,
	float min_head_length,
	float &line_length
)
{
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

	pcl::PointXYZRGBL c_min_XYZ, c_max_XYZ;
	pcl::getMinMax3D(*cluster, c_min_XYZ, c_max_XYZ);
	float c_dy = c_max_XYZ.y - c_min_XYZ.y;
	//std::cout << "      ==> c_dy: " << c_dy << std::endl;
/*
	pcl::CropBox<pcl::PointXYZRGBL> box_filter;
	box_filter.setInputCloud(cluster);
	//box_filter.setMin(Eigen::Vector4f(c_min_XYZ.x, c_min_XYZ.y,       c_max_XYZ.z - dz, 1.0));
	box_filter.setMin(Eigen::Vector4f(c_min_XYZ.x, c_min_XYZ.y,       c_max_XYZ.z - 1.0, 1.0));
	box_filter.setMax(Eigen::Vector4f(c_max_XYZ.x, c_min_XYZ.y + 2.8, c_max_XYZ.z,       1.0));
	box_filter.filter(*cloud);
*/
	bool valid_line_found = false;

	*cloud = *cluster;
	int n_points = cloud->points.size();
	//std::cout << "      ==> n_points: " << n_points << std::endl;

	if (n_points > 0)
	{
		pcl::PointXYZRGBL min_XYZ, max_XYZ;
		pcl::getMinMax3D(*cloud, min_XYZ, max_XYZ);
		float dz = max_XYZ.z - min_XYZ.z;

		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmp_edge(new pcl::PointCloud<pcl::PointXYZRGBL>);
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_largest_edge(new pcl::PointCloud<pcl::PointXYZRGBL>);

		find_edge_points(cloud, tmp_edge, min_XYZ.x, min_XYZ.y, min_XYZ.z, max_XYZ.x, max_XYZ.y, max_XYZ.z, 0.0, dz, near_edge, far_edge);
		int n_tmp_edge_points = tmp_edge->points.size();
		//std::cout << "      ==> n_tmp_edge_points: " << n_tmp_edge_points << std::endl;
		int n_tmp_line_points = 0;
		if (n_tmp_edge_points > 2)
		{
			*edge = *tmp_edge;

			// filtering by cluster needed before evaluating line					
			find_largest_cluster(tmp_edge, cloud_largest_edge, distanceTolerance, 9, 10000);
			int n_tmp_edge_cluster_points = cloud_largest_edge->points.size();
			//std::cout << "      ==> n_tmp_edge_cluster_points: " << n_tmp_edge_cluster_points << std::endl;

			if (n_tmp_edge_cluster_points > 0)
			{
				std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> cloud_tmp_lines;
				std::vector<float> tmp_line_lengths;
				std::vector<float> tmp_line_slopes;

				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _remains(new pcl::PointCloud<pcl::PointXYZRGBL>);
				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _c(new pcl::PointCloud<pcl::PointXYZRGBL>);
				*_c = *cloud_largest_edge;
				for (int i = 0; i < 3; i ++)
				{
					if (_c->points.size() < 3) break;
					
					//std::cout << "1      ==> _c points: " << _c->points.size() << std::endl;
					pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
					extract_line_points(_c, cloud_tmp_line, 0.125, _remains);
					
					n_tmp_line_points = cloud_tmp_line->points.size();
					//std::cout << "1      ==> n_tmp_line_points: " << n_tmp_line_points << std::endl;
					if (n_tmp_line_points > 2)
					{
						find_longest_cluster(cloud_tmp_line, cloud_tmp_line, distanceTolerance, 7, 1000);
						n_tmp_line_points = cloud_tmp_line->points.size();
						//std::cout << "2         ==> n_tmp_line_points: " << n_tmp_line_points << std::endl;
						
						float line_slope;
						float line_length;
						if (n_tmp_line_points > 3)
						{
							bool line_valid = validate_cluster_edge_line(cloud_tmp_line, min_XYZ.y, min_dx, max_dx, max_slope, min_y_shift, max_y_shift, line_length, line_slope); 
							//std::cout << "      ==> line_valid: " << line_valid << std::endl;
							if (line_valid)
							{
								cloud_tmp_lines.push_back(cloud_tmp_line);
								tmp_line_lengths.push_back(line_length);
								tmp_line_slopes.push_back(line_slope);
							} else 
							{
								break;
							}
						} else 
						{
							break;
						}
					} else 
					{
						break;
					}
					*_c = *_remains;
				}
				//std::cout << "   ==> cloud_tmp_lines: " << cloud_tmp_lines.size() << std::endl;
				if (cloud_tmp_lines.size() > 0)
				{
					for (int i = 0; i < cloud_tmp_lines.size(); i ++)
					{
						// verify if the line valid
						pcl::PointXYZRGBL min_OBB, max_OBB, position_OBB;
						Eigen::Matrix3f rotational_matrix_OBB;
						pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> f;
						f.setInputCloud(cloud_tmp_lines[i]);
						f.compute();
						f.getOBB(min_OBB, max_OBB, position_OBB, rotational_matrix_OBB);
						//std::cout << "   position_OBB  x:: " << position_OBB.x << " y: " << position_OBB.y << " z: " << position_OBB.z << std::endl;

						Eigen::Affine3f t_OBB = Eigen::Affine3f::Identity();
						t_OBB.rotate (rotational_matrix_OBB);
						t_OBB.translation() << position_OBB.x, position_OBB.y, position_OBB.z;

						Eigen::Affine3f t_OBB_inverse = t_OBB.inverse();

						pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _tmp_center(new pcl::PointCloud<pcl::PointXYZRGBL>);
						pcl::transformPointCloud (*center, *_tmp_center, t_OBB_inverse);
						pcl::PointXYZRGBL &cp = _tmp_center->points[0];
						if (cp.y < 0)
						{
							//std::cout << "==> rotate OBB to positive side ... " << std::endl;
							Eigen::AngleAxisf r;
							r =
								Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
								* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
								* Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
							Eigen::Affine3f t_ = Eigen::Affine3f::Identity();
							t_.rotate (r);

							pcl::transformPointCloud (*_tmp_center, *_tmp_center, t_);
							//std::cout << "==> after rotate, x: " << cp.x << " y: " << cp.y << " z: " << cp.z << std::endl;

							t_OBB_inverse = t_ * t_OBB_inverse;

							Eigen::Affine3f t_inverse = t_.inverse();
							t_OBB = t_OBB * t_inverse;
						}
						pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _tmp_line_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
						pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _tmp_cloud_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
						pcl::transformPointCloud (*cloud_tmp_lines[i], *_tmp_line_OBB, t_OBB_inverse);
						pcl::transformPointCloud (*cluster, *_tmp_cloud_OBB, t_OBB_inverse);

						pcl::PointXYZRGBL tmp_line_min_OBB, tmp_line_max_OBB;
						pcl::getMinMax3D(*_tmp_line_OBB, tmp_line_min_OBB, tmp_line_max_OBB);

						pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _tmp_c(new pcl::PointCloud<pcl::PointXYZRGBL>);
						pcl::PassThrough<pcl::PointXYZRGBL> pass;
						pass.setFilterLimitsNegative (false);
						pass.setInputCloud (_tmp_cloud_OBB);
						pass.setFilterFieldName ("y");
						pass.setFilterLimits (-65.0, tmp_line_min_OBB.y); 
						pass.filter (*_tmp_c);
						
						//std::cout << "1      ==> _tmp_c points: " << _tmp_c->points.size() << std::endl;

						if (_tmp_c->points.size() > 0)
						{
							pcl::PointXYZRGBL head_min_OBB, head_max_OBB;
							pcl::getMinMax3D(*_tmp_c, head_min_OBB, head_max_OBB);
							float head_dy = head_max_OBB.y - head_min_OBB.y;
							//std::cout << "      ==> head_dy: " << head_dy << " min_head_length: " << min_head_length << std::endl;
							if (head_dy >= min_head_length)
							{
								line_length = tmp_line_lengths[i];
								*line = *cloud_tmp_lines[i];
								valid_line_found = true;
							} else if (near_edge)
							{
								line_length = tmp_line_lengths[i];
								*line = *cloud_tmp_lines[i];
								valid_line_found = true;
								// shift (c_dy - head_dy)
								for (int _i = 0; _i < line->points.size(); _i ++)
								{
									pcl::PointXYZRGBL &p = line->points[_i];
									p.y += (c_dy - head_dy);
								}
							}
						}
					}
				}
			}
		}
	}

	return valid_line_found;
}

void downsample_cloud(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_in,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_out,
	float radius
)
{
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);

	pcl::UniformSampling<pcl::PointXYZRGBL> uniform_sampling;
	uniform_sampling.setInputCloud(_cloud_in);
	uniform_sampling.setRadiusSearch(radius);
	uniform_sampling.filter (*_cloud_tmp);
	*_cloud_out = *_cloud_tmp;
}
void detect_truck_atside(
    Eigen::Affine3d t_sensor,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_raw,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_center,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_downsampled,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_downsampled,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_ground_plane,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cropped,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_content_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_1_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_1_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_2_OBB,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_points_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point,
    bool &vertex_computed,
    bool &falling_found,
    bool &dest_found,
	bool &x_in_boundary, 
	bool &y_in_boundary, 
	bool balanced_fill,
	bool distance_ctrl,
	float &sprout_angle_Z,
	float sprout_angle_available
)
{
	//std::cout << "==> detect_truck_atside ------------------------------------------------------------ " << std::endl;

    int n_cropped_points = cloud_cropped->points.size();

	if (n_cropped_points == 0) return;

	pcl::transformPointCloud (*cloud_cropped, *cloud_cropped, t_sensor.inverse());

	// --- 16 threads filtering --- //
	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> clouds_filtered;
	for (int i = 0; i < 16; i ++)
	{
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_seg(new pcl::PointCloud<pcl::PointXYZRGBL>);
		clouds_filtered.push_back(_cloud_seg);
	}
	std::vector<std::thread> threads;
	split_cloud_16_z(cloud_cropped, clouds_filtered);

	for (int i = 0; i < 16; i ++)
	{
		//std::cout << "===>before filter clouds_filtered " << std::to_string(i) << ": " << clouds_filtered[i]->points.size() << std::endl;
		std::thread th_filter(&filter_by_clustering, clouds_filtered[i], clouds_filtered[i], 0.1 + 0.005 * i, 60 - i * 2.5, 100000);
		threads.push_back(std::move(th_filter));
	}
	
	for (int i = 0; i < 16; i ++)
	{
		threads[i].join();
	}
	cloud_cropped->points.clear();
	for (int i = 0; i < 16; i ++)
	{
		//std::cout << "===>after filter clouds_filtered " << std::to_string(i) << ": " << clouds_filtered[i]->points.size() << std::endl;
		*cloud_cropped += *clouds_filtered[i];
	}
	pcl::transformPointCloud (*cloud_cropped, *cloud_cropped, t_sensor);

	n_cropped_points = cloud_cropped->points.size();

	int n_downsampled_points = 0;
	if (n_cropped_points > 0) 
	{
		downsample_cloud(cloud_cropped, cloud_downsampled, 0.125);
		n_downsampled_points = cloud_downsampled->points.size();

	}
	//std::cout << "==> n_downsampled_points: " << n_downsampled_points << std::endl;

	int n_filtered_points = 0;
	if (n_downsampled_points > 0)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
		sor.setMeanK(5);
		sor.setStddevMulThresh(1.0);
		sor.setInputCloud(cloud_downsampled);
		sor.filter(*cloud_filtered);
		
		n_filtered_points = cloud_filtered->points.size();
	}
	//std::cout << "==> n_filtered_points: " << n_filtered_points << std::endl;

	int n_cluster_points = 0;
	pcl::PointXYZRGBL cluster_min_XYZ, cluster_max_XYZ;
	if (n_filtered_points > 0) 
	{
		find_likely_cluster(cloud_filtered, cloud_closest_cluster, 0.175, 100, 50000, 1.6, 0.25, cluster_min_XYZ, cluster_max_XYZ); // max 0.175 from 0.1 + 0.005 * i
		pcl::getMinMax3D(*cloud_closest_cluster, cluster_min_XYZ, cluster_max_XYZ);
		//std::cout << "==> cluster_min_XYZ.y: " << cluster_min_XYZ.y << "  cluster_max_XYZ.z: " << cluster_max_XYZ.z << std::endl;

		n_cluster_points = cloud_closest_cluster->points.size();
	}
	//std::cout << "final ==> n_cluster_points: " << n_cluster_points << std::endl;

	int n_edge_points = 0;
	if (n_cluster_points > 0) 
	{
		find_edge_points(cloud_closest_cluster, cloud_edge, cluster_min_XYZ.x, cluster_min_XYZ.y, cluster_min_XYZ.z, cluster_max_XYZ.x, cluster_max_XYZ.y, cluster_max_XYZ.z, 0.0, 1.0, true, false);
		n_edge_points = cloud_edge->points.size();
	}
	//std::cout << "==> n_edge_points: " << n_edge_points << std::endl;

	int n_line_points = 0;
	bool line_valid = false;	
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge_remains(new pcl::PointCloud<pcl::PointXYZRGBL>);
	float length_of_line = 0;
	float line_slope;
	if (n_edge_points > 0)
	{
		extract_line_points(cloud_edge, cloud_line, 0.175, cloud_edge_remains);
		n_line_points = cloud_line->points.size();
		//std::cout << "   ==> after clustering n_line_points: " << n_line_points << std::endl;

		if (n_line_points > 0)
		{
			line_valid = validate_cluster_edge_line(cloud_line, cluster_min_XYZ.y, 1.5, 6.0, 2.0, 0.0, 3.0, length_of_line, line_slope);
		}
		//std::cout << "==> attempt 1 => n_line_points: " << n_line_points << " line_valid: " << line_valid << std::endl;
	}	

	if ((! line_valid) && n_cluster_points > 0)
	{
		find_edge_points(cloud_closest_cluster, cloud_edge, cluster_min_XYZ.x, cluster_min_XYZ.y, cluster_min_XYZ.z, cluster_max_XYZ.x, cluster_max_XYZ.y, cluster_max_XYZ.z, 0.5, 0.5, true, false);
		n_edge_points = cloud_edge->points.size();
		//std::cout << "      ==> n_edge_points: " << n_edge_points << std::endl;
		if (n_edge_points > 0)
		{
			extract_line_points(cloud_edge, cloud_line, 0.125, cloud_edge_remains);
			n_line_points = cloud_line->points.size();
			//std::cout << "   ==> after clustering n_line_points: " << n_line_points << std::endl;
			if (n_line_points > 0)
			{
				line_valid = validate_cluster_edge_line(cloud_line, cluster_min_XYZ.y, 1.5, 6.0, 2.0, 0.0, 3.0, length_of_line, line_slope);
			}
		}
		//std::cout << "==> attempt 2 => n_line_points: " << n_line_points << " line_valid: " << line_valid << std::endl;
	}

	if ((! line_valid) && n_cluster_points > 0)
	{
		find_edge_points(cloud_closest_cluster, cloud_edge, cluster_min_XYZ.x, cluster_min_XYZ.y, cluster_min_XYZ.z, cluster_max_XYZ.x, cluster_max_XYZ.y, cluster_max_XYZ.z, 0.75, 0.5, true, false);
		n_edge_points = cloud_edge->points.size();
		//std::cout << "      ==> n_edge_points: " << n_edge_points << std::endl;
		if (n_edge_points > 0)
		{
			extract_line_points(cloud_edge, cloud_line, 0.125, cloud_edge_remains);
			n_line_points = cloud_line->points.size();
			//std::cout << "   ==> before clustering n_line_points: " << n_line_points << std::endl;
			if (n_line_points > 0)
			{
				line_valid = validate_cluster_edge_line(cloud_line, cluster_min_XYZ.y, 1.5, 6.0, 2.0, 0.0, 3.0, length_of_line, line_slope);
			}
		}
		//std::cout << "==> attempt 3 => n_line_points: " << n_line_points << " line_valid: " << line_valid << std::endl;
	}

	if (! line_valid)
	{
		n_line_points = 0;
	}
	n_line_points = cloud_line->points.size();
	//std::cout << "==> line_valid: " << line_valid << " n_line_points: " << n_line_points << std::endl;

	pcl::PointXYZRGBL min_OBB, max_OBB, position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;		
	Eigen::Affine3f t_OBB = Eigen::Affine3f::Identity();
	pcl::PointXYZRGBL cluster_OBB_min_XYZ, cluster_OBB_max_XYZ;

	pcl::PointXYZRGBL line_OBB_min_XYZ, line_OBB_max_XYZ;
	
	bool cluster_valid = false;
	Eigen::Affine3f t_OBB_inverse;
	
	int truck_location = 0;
	float angle_d = 0.0;
	if (n_line_points > 0)
	{	
		add_center_ref(_cloud_center, 25.0);
		pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_sensor);

		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> f;
		f.setInputCloud(cloud_line);
		f.compute();
		f.getOBB(min_OBB, max_OBB, position_OBB, rotational_matrix_OBB);
		//std::cout << "   position_OBB  x:: " << position_OBB.x << " y: " << position_OBB.y << " z: " << position_OBB.z << std::endl;

		t_OBB = Eigen::Affine3f::Identity();
		t_OBB.rotate (rotational_matrix_OBB);

		t_OBB_inverse = t_OBB.inverse();

		pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_OBB_inverse);
		pcl::PointXYZRGBL &cp = _cloud_center->points[0];
		if (cp.y < 0)
		{
			//std::cout << "==> rotate OBB to positive side ... " << std::endl;
			Eigen::AngleAxisf r;
			r =
				  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
			Eigen::Affine3f t_ = Eigen::Affine3f::Identity();
			t_.rotate (r);

			pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_);
			//std::cout << "==> after rotate, x: " << cp.x << " y: " << cp.y << " z: " << cp.z << std::endl;

			t_OBB_inverse = t_ * t_OBB_inverse;

			Eigen::Affine3f t_inverse = t_.inverse();
			t_OBB = t_OBB * t_inverse;
		}
		
		pcl::transformPointCloud (*cloud_line, *cloud_line_OBB, t_OBB_inverse);
		pcl::transformPointCloud (*cloud_closest_cluster, *cloud_closest_cluster_OBB, t_OBB_inverse);
		
		pcl::getMinMax3D(*cloud_line_OBB, line_OBB_min_XYZ, line_OBB_max_XYZ);
		//std::cout << "==> line_OBB_min_XYZ.y: " << line_OBB_min_XYZ.y << " line_OBB_max_XYZ.y: " << line_OBB_max_XYZ.y << std::endl;

		float arm_length = 3.0;
		pcl::PointXYZRGBL line_point_s, line_point_e;
		float x_s = 65.0, x_e = -65.0;
		for (int i = 0; i < n_line_points; i ++)
		{
			pcl::PointXYZRGBL p = cloud_line->points[i];
			if (p.x < x_s)
			{
				x_s = p.x;
				line_point_s = p;
			}
			if (p.x > x_e)
			{
				x_e = p.x;
				line_point_e = p;
			}
		}
		float dx = line_point_e.x - line_point_s.x;
		float dy = line_point_e.y - line_point_s.y;
		
		float slp = dy / dx;
		float angle = std::atan(slp);
		float offset_y = 0.0;
		angle_d = 180.0 * angle / M_PI;
		
		//std::cout << "===> dx: " << dx << " dy: " << dy << " slp: " << slp << " angle_d: " << angle_d << std::endl;
		if (angle_d > 15.0)
		{
			float cos_a = std::cos(angle);
			float d_l = arm_length / cos_a;
			float d_d = d_l - arm_length;
			offset_y = d_d * cos_a;
			
			//std::cout << "===> dx: " << dx << " dy: " << dy << " slp: " << slp << " angle: " << 180.0 * angle / M_PI << " d_l: " << d_l << " d_d: " << d_d << " offset_y: " << offset_y << std::endl;
			offset_y += 0.6125;
		} else 
		{
			offset_y = line_OBB_min_XYZ.y;// - 0.125;
		}
		if (angle_d > 10.0)
		{
			truck_location = 4;
		} else if (angle_d < -10.0)
		{
			truck_location = 2;
		}
		if (offset_y < line_OBB_min_XYZ.y)
		{
			offset_y = line_OBB_min_XYZ.y;
		}
		//std::cout << "==> offset_y: " << offset_y  << std::endl;

		pcl::getMinMax3D(*cloud_closest_cluster_OBB, cluster_OBB_min_XYZ, cluster_OBB_max_XYZ);
		float dy_estimated_cluster = cluster_OBB_max_XYZ.y - offset_y;
		//std::cout << "==> dy_estimated_cluster: " << dy_estimated_cluster  << std::endl;
		if (dy_estimated_cluster < 2.5)
		{
			dy_estimated_cluster = cluster_OBB_max_XYZ.y - line_OBB_min_XYZ.y;
			//std::cout << "==> dy_estimated_cluster too small, re-estimate: " << dy_estimated_cluster  << std::endl;
			if (dy_estimated_cluster < 3.0)
			{
				//reset offset_y as looks like offset_y calculated by angle not right
				offset_y = line_OBB_min_XYZ.y;
			}
		}

		sprout_angle_Z = angle_d;
		
		//std::cout << "==> cloud_closest_cluster_OBB: " << cloud_closest_cluster_OBB->points.size() << std::endl;
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_n(new pcl::PointCloud<pcl::PointXYZRGBL>);
		pcl::PassThrough<pcl::PointXYZRGBL> pass;
		pass.setFilterLimitsNegative (false);
		pass.setInputCloud (cloud_closest_cluster_OBB);		
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (offset_y, 65.0);
		pass.filter (*cloud_tmp);
		//std::cout << "==> cloud_tmp: " << cloud_tmp->points.size() << std::endl;
		
		pass.setFilterLimits(-65.0, offset_y);
		pass.filter (*cloud_tmp_n);
		
		*cloud_closest_cluster_OBB = *cloud_tmp;
		
		pcl::getMinMax3D(*cloud_closest_cluster_OBB, cluster_OBB_min_XYZ, cluster_OBB_max_XYZ);
		float cluster_dx = cluster_OBB_max_XYZ.x - cluster_OBB_min_XYZ.x;
		float cluster_dy = cluster_OBB_max_XYZ.y - cluster_OBB_min_XYZ.y;

		//std::cout << "==> cluster_dx: " << cluster_dx << " cluster_dy: " << cluster_dy << std::endl;
		//std::cout << "==> cluster_OBB_min_XYZ.x: " << cluster_OBB_min_XYZ.x << std::endl;
		//std::cout << "==> cluster_OBB_min_XYZ.y: " << cluster_OBB_min_XYZ.y << std::endl;
		//std::cout << "==> cluster_OBB_max_XYZ.x: " << cluster_OBB_max_XYZ.x << std::endl;

		if (cluster_dx >= 1.6)
		{
			pcl::PointXYZRGBL tmp_min_XYZ, tmp_max_XYZ;
			pcl::getMinMax3D(*cloud_tmp_n, tmp_min_XYZ, tmp_max_XYZ);
			float tmp_dy = tmp_max_XYZ.y - tmp_min_XYZ.y;
			//std::cout << "==> tmp_dy: " << tmp_dy  << std::endl;
			//std::cout << "==> tmp_min_XYZ.y: " << tmp_min_XYZ.y  << std::endl;

			if (tmp_dy > 2.5) // the line detected, may be far side, but in this case, the dy should be less than truck width
			{
				//std::cout << "==> might be far side detected with tmp_dy: " << tmp_dy << std::endl;
			} else if (tmp_dy > 1.6)
			{
				// no near side edge found
				//std::cout << "==> looks like not valid with tmp_dy: " << tmp_dy << std::endl;
			} else 
			{
				cluster_valid = true;
			}
			
		}
	}
	//std::cout << "==> cluster_valid: " << cluster_valid << std::endl;

	bool top_line_found = false;
	if (cluster_valid)
	{
		top_line_found = find_top_lines_OBB(
			cloud_closest_cluster_OBB, 
			cluster_OBB_min_XYZ, cluster_OBB_max_XYZ, 
			0.5, 0.6, 2.0, 
			line_OBB_min_XYZ, line_OBB_max_XYZ, 
			cloud_wall_1_OBB,
			true,
			cloud_top_line_1_OBB, cloud_top_line_2_OBB
		);
	}
	//std::cout << "==> top_line_found: " << top_line_found << std::endl;

	pcl::PointXYZRGBL top_OBB_min_XYZ, top_OBB_max_XYZ;	
	pcl::PointXYZRGBL vertex_OBB_min_XYZ, vertex_OBB_max_XYZ;
	vertex_computed = false;
	pcl::PointXYZRGBL top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ;
	if (top_line_found)
	{
		pcl::getMinMax3D(*cloud_top_line_1_OBB, top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ);

		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane_top_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
		*cloud_plane_top_OBB = *cloud_top_line_1_OBB + *cloud_top_line_2_OBB;

		pcl::getMinMax3D(*cloud_plane_top_OBB, top_OBB_min_XYZ, top_OBB_max_XYZ);

		vertex_computed = compute_vertex(top_OBB_min_XYZ, top_OBB_max_XYZ, top_line_1_OBB_max_XYZ.z, cloud_vertex_OBB);
		pcl::getMinMax3D(*cloud_vertex_OBB, vertex_OBB_min_XYZ, vertex_OBB_max_XYZ);
	}
	//std::cout << "   ==> vertex_computed: " << vertex_computed << std::endl;

	//std::cout << "==> vertex: " << vertex_computed << ":" << std::endl;
	//std::cout << "   ===> vertex_OBB_min_XYZ.x: " << vertex_OBB_min_XYZ.x << " vertex_OBB_min_XYZ.y: " << vertex_OBB_min_XYZ.y << " vertex_OBB_min_XYZ.z: " << vertex_OBB_min_XYZ.z << std::endl;
	//std::cout << "   ===> vertex_OBB_max_XYZ.x: " << vertex_OBB_max_XYZ.x << " vertex_OBB_max_XYZ.y: " << vertex_OBB_max_XYZ.y << " vertex_OBB_max_XYZ.z: " << vertex_OBB_max_XYZ.z << std::endl;

	pcl::PointXYZRGBL point_OBB_falling;
	pcl::PointXYZRGBL point_OBB_dest;
	falling_found = false;
	dest_found = false;
	int n_content_points = 0;
	y_in_boundary = true;
	bool head_detected = false;

	pcl::CropBox<pcl::PointXYZRGBL> box_filter;
	if (vertex_computed)
	{
		//////////////////////////////////////////////// ---compensate the invisible part--- ///////////////////////////////////////////////////
		
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_c_obb(new pcl::PointCloud<pcl::PointXYZRGBL>);

		pcl::CropBox<pcl::PointXYZRGBL> boxFilter;
		boxFilter.setMin(Eigen::Vector4f(cluster_OBB_min_XYZ.x, cluster_OBB_min_XYZ.y + 0.25,   cluster_OBB_min_XYZ.z + 0.25, 1.0));
		boxFilter.setMax(Eigen::Vector4f(cluster_OBB_max_XYZ.x, cluster_OBB_max_XYZ.y,          cluster_OBB_max_XYZ.z,        1.0)); 
		boxFilter.setInputCloud(cloud_closest_cluster_OBB);
		boxFilter.filter(*cloud_tmp_c_obb);

		int n_points_cluster_obb = cloud_tmp_c_obb->points.size();

		pcl::PointXYZRGBL cluster_OBB_point_l, cluster_OBB_point_r;
		float x_s = 65.0, x_e = -65.0;
		for (int i = 0; i < n_points_cluster_obb; i ++)
		{
			pcl::PointXYZRGBL p = cloud_tmp_c_obb->points[i];
			if (p.x < x_s)
			{
				x_s = p.x;
				cluster_OBB_point_l = p;
			}
			if (p.x > x_e)
			{
				x_e = p.x;
				cluster_OBB_point_r = p;
			}
		}
		float dx_cluster = cluster_OBB_point_r.x - cluster_OBB_point_l.x;
		float dy_cluster = cluster_OBB_point_r.y - cluster_OBB_point_l.y;

		uint32_t label_l = cluster_OBB_point_l.label;
		int x_l = static_cast<uint16_t> ((label_l & 0x00FFF000) >> 12);
		int y_l = static_cast<uint16_t> (label_l & 0x00000FFF);

		uint32_t label_r = cluster_OBB_point_r.label;
		int x_r = static_cast<uint16_t> ((label_r & 0x00FFF000) >> 12);
		int y_r = static_cast<uint16_t> (label_r & 0x00000FFF);

		float vertx_length = vertex_OBB_max_XYZ.x - vertex_OBB_min_XYZ.x;
		//std::cout << "===============> dx_cluster: " << dx_cluster << " vertx_length: " << vertx_length << " x_l: " << x_l << " y_l: " << y_l << " x_r: " << x_r << " y_r: " << y_r << std::endl;
		
		bool border_l = false;
		bool border_r = false;
		
		bool compensate_l = false;
		bool compensate_r = false;
		bool head_left = false;
		bool head_right = false;
		
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster_near(new pcl::PointCloud<pcl::PointXYZRGBL>);
		pcl::PointXYZRGBL cluster_near_min_XYZ, cluster_near_max_XYZ;	

		pcl::PassThrough<pcl::PointXYZRGBL> pass;
		pass.setFilterLimitsNegative (false);
		pass.setInputCloud (cloud_closest_cluster_OBB);		
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (cluster_OBB_min_XYZ.y + 0.25, cluster_OBB_min_XYZ.y + 0.5);
		pass.filter (*cloud_cluster_near);

		pcl::getMinMax3D(*cloud_cluster_near, cluster_near_min_XYZ, cluster_near_max_XYZ);

		float d_left  = vertex_OBB_min_XYZ.x   - cluster_near_min_XYZ.x;
		float d_right = cluster_near_max_XYZ.x - vertex_OBB_max_XYZ.x;
		//std::cout << "===============> d_left: " << d_left << " d_right: " << d_right << std::endl;

		if (d_left > 0.75 && d_left < 2.0)
		{
			head_left = true;
		}

		if (d_right > 0.75 && d_right < 2.0)
		{
			head_right = true;
		}

		if (head_left || head_right)
		{
			head_detected = true;
		}
		//std::cout << "===============> head_detected: " << head_detected << " head_left: " << head_left << " head_right: " << head_right << std::endl;
		
		// compensate
		if (
			(head_detected && vertx_length < 3.0) 
			|| 
			((! head_detected) && vertx_length < 4.0)
		)
		{
			if (x_l > 290)
			{
				border_l = true;
			} else if (x_l > 270 && x_r > 80)
			{
				border_l = true;
			}

			if (x_r < 30)
			{
				border_r = true;
			} else if (x_r < 50 && x_l < 240)
			{
				border_r = true;
			}
			//std::cout << "1===============> border_l: " << border_l << " border_r: " << border_r << std::endl;
			
			if (border_r && (! head_right))
			{
				//std::cout << "===============> compensate right ---------------------> " << std::endl;				
				if (head_detected)
				{
					if (vertex_OBB_max_XYZ.x < (vertex_OBB_min_XYZ.x + 3.0)) 
					{
						vertex_OBB_max_XYZ.x = vertex_OBB_min_XYZ.x + 3.0;
						compensate_r = true;
					}
				} else 
				{
					if (vertex_OBB_max_XYZ.x < (vertex_OBB_min_XYZ.x + 4.0)) 
					{
						vertex_OBB_max_XYZ.x = vertex_OBB_min_XYZ.x + 4.0;
						compensate_r = true;
					}
				}
				if (compensate_r) compute_vertex(vertex_OBB_min_XYZ, vertex_OBB_max_XYZ, vertex_OBB_max_XYZ.z, cloud_vertex_OBB);				
			} 
			if (border_l && (! head_left))
			{
				//std::cout << "===============> compensate left ---------------------> " << std::endl;
				if (head_detected)
				{
					if (vertex_OBB_min_XYZ.x > (vertex_OBB_max_XYZ.x - 3.0)) 
					{
						vertex_OBB_min_XYZ.x = vertex_OBB_max_XYZ.x - 3.0;
						compensate_l = true;
					}
				} else 
				{
					if (vertex_OBB_min_XYZ.x > (vertex_OBB_max_XYZ.x - 4.0)) 
					{
						vertex_OBB_min_XYZ.x = vertex_OBB_max_XYZ.x - 4.0; 
						compensate_l = true;
					}
				}
				if (compensate_l) compute_vertex(vertex_OBB_min_XYZ, vertex_OBB_max_XYZ, vertex_OBB_max_XYZ.z, cloud_vertex_OBB);
			}
			
			//std::cout << "===============> compensate_l: " << compensate_l << " compensate_r: " << compensate_r << std::endl;
			
			if (compensate_l || compensate_r)
			{
				//std::cout << "===============> optimize the height of vertex ------------->" << std::endl;
				
				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
				pcl::CropBox<pcl::PointXYZRGBL> boxFilter;
				boxFilter.setMin(Eigen::Vector4f(vertex_OBB_min_XYZ.x + 2.0, -65.0,   vertex_OBB_max_XYZ.z, 1.0));
				boxFilter.setMax(Eigen::Vector4f(vertex_OBB_max_XYZ.x - 2.0,  65.0,                   65.0, 1.0)); 
				boxFilter.setInputCloud(cloud_wall_1_OBB);
				boxFilter.filter(*cloud_tmp);

				pcl::PointXYZRGBL wall_opt_min_XYZ, wall_opt_max_XYZ;
				pcl::getMinMax3D(*cloud_tmp, wall_opt_min_XYZ, wall_opt_max_XYZ);
				float wall_opt_height = wall_opt_max_XYZ.z - wall_opt_min_XYZ.z;
				float wall_opt_length = wall_opt_max_XYZ.x - wall_opt_min_XYZ.x;

				float step = 0.125;
				int n = ceil(wall_opt_height / step); 
				//std::cout << "=> opt n: " << n << " step: " << step << std::endl;

				pcl::PassThrough<pcl::PointXYZRGBL> pass;
				pass.setFilterLimitsNegative (false);				
				pass.setInputCloud (cloud_tmp);
				pass.setFilterFieldName ("z");

				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_step(new pcl::PointCloud<pcl::PointXYZRGBL>);
				for (int i = 0; i < n; i ++)
				{
					pass.setFilterLimits (wall_opt_min_XYZ.z + (i - 0.5) * step, wall_opt_min_XYZ.z + (i + 1.5) * step);
					pass.filter (*cloud_step);
					//std::cout << "   => cloud_step: " << cloud_step->points.size() << std::endl;

					if (cloud_step->points.size() > 1)
					{
						int n_step_points = cloud_step->points.size();
						//std::cout << "         => raw n_step_points: " << n_step_points << std::endl;
						if (n_step_points < 3) 
						{
							break;
						}

						double mcx = 0.0, mcy = 0.0, mcz = 0.0;
						for (int i = 0; i < n_step_points; i++ )
						{
							pcl::PointXYZRGBL p = cloud_step->points[i];
							mcx += p.x;
							mcy += p.y;
							mcz += p.z;
						}
						float cy = mcy / n_step_points;
						for (int i = 0; i < n_step_points; i++ )
						{
							pcl::PointXYZRGBL &p = cloud_step->points[i];
							p.y = cy;
						}

						pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_remains(new pcl::PointCloud<pcl::PointXYZRGBL>);
						pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
						extract_line_points(cloud_step, cloud_tmp_line, 0.125, cloud_tmp_remains);
						int n_tmp_line_points = cloud_tmp_line->points.size();
						//std::cout << "         => raw n_tmp_line_points: " << n_tmp_line_points << std::endl;

						if (n_tmp_line_points < 2) continue;

						float length_of_line = find_longest_cluster(cloud_tmp_line, cloud_tmp_line, 0.5, 10, 1000); 
						n_tmp_line_points = cloud_tmp_line->points.size();
						//std::cout << "         => clustered n_tmp_line_points: " << n_tmp_line_points << std::endl;
						
						*cloud_step = *cloud_tmp_line;

						//std::cout << "   => clean cloud_step: " << cloud_step->points.size() << std::endl;

						if (n_tmp_line_points > 0)
						{
							pcl::PointXYZRGBL minXYZ, maxXYZ;
							pcl::getMinMax3D(*cloud_step, minXYZ, maxXYZ);
							float dx = maxXYZ.x - minXYZ.x;
							//std::cout << "    opt  => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << " wall_opt_length: " << wall_opt_length  << " points: " << cloud_step->points.size() << std::endl;
							//std::cout << "    opt  => wall_opt_length: " << wall_opt_length << " dx: " << dx << std::endl;
							//std::cout << "    opt  => 0.5 * wall_opt_length: " << 0.5 * wall_opt_length << std::endl;
							if (dx > 0.8 * wall_opt_length || dx > 2.0)
							{
									*cloud_top_line_1_OBB = *cloud_step;
									//std::cout << "               opt   => valid top found => dx: " << dx << " maxXYZ.z: " << maxXYZ.z << std::endl;
							} else 
							{
								break;
							}
						}
					}
				}

				pcl::PointXYZRGBL minXYZ, maxXYZ;
				pcl::getMinMax3D(*cloud_top_line_1_OBB, minXYZ, maxXYZ);
				vertex_OBB_min_XYZ.z = maxXYZ.z;
				vertex_OBB_max_XYZ.z = maxXYZ.z;
				compute_vertex(vertex_OBB_min_XYZ, vertex_OBB_max_XYZ, vertex_OBB_max_XYZ.z, cloud_vertex_OBB);
			}
		}

		pcl::PointXYZRGBL &center_p = _cloud_center->points[0];
		center_p.z = vertex_OBB_max_XYZ.z;
		
		box_filter.setInputCloud(cloud_closest_cluster_OBB);
		box_filter.setMin(Eigen::Vector4f(vertex_OBB_min_XYZ.x + 0.25, vertex_OBB_min_XYZ.y + 0.25,                        -65.0, 1.0));
		box_filter.setMax(Eigen::Vector4f(vertex_OBB_max_XYZ.x - 0.25, vertex_OBB_max_XYZ.y,         vertex_OBB_max_XYZ.z - 0.25, 1.0));
		box_filter.filter(*cloud_content_OBB);
		n_content_points = cloud_content_OBB->points.size();

		*cloud_available_voxel_OBB = *cloud_content_OBB;
		//std::cout << "   00===============> n_content_points: " << n_content_points << std::endl;
		
		point_OBB_dest.r = 255;
		point_OBB_dest.g = 255;
		point_OBB_dest.b = 255;

		point_OBB_dest.x = 0.5 * (vertex_OBB_min_XYZ.x + vertex_OBB_max_XYZ.x);
		point_OBB_dest.y = 0.5 * (vertex_OBB_min_XYZ.y + vertex_OBB_max_XYZ.y);
		point_OBB_dest.z = vertex_OBB_max_XYZ.z; 
		dest_found = true;

		if (balanced_fill) 
		{
			float d_vertex = vertex_OBB_max_XYZ.x - vertex_OBB_min_XYZ.x;
			if (cloud_available_voxel_OBB->points.size() > 0 && d_vertex > 1.5)
			{
				pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
				sor.setMeanK(15);
				sor.setStddevMulThresh(1.0);
				sor.setInputCloud(cloud_available_voxel_OBB);
				sor.filter(*cloud_available_voxel_OBB);
				int n_available_points = cloud_available_voxel_OBB->points.size();
				//std::cout << "1   ===============> n_available_points: " << n_available_points << std::endl;

				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
				find_best_dest(cloud_available_voxel_OBB, cloud_available_cluster, 0.25, 1, 10000, vertex_OBB_min_XYZ, vertex_OBB_max_XYZ, cloud_closest_cluster_OBB);
				*cloud_available_voxel_OBB = *cloud_available_cluster;
				n_available_points = cloud_available_voxel_OBB->points.size();
				//std::cout << "2   ==> n_available_points: " << n_available_points << std::endl;

				// reliable dest may be at least something 5 points?
				if (n_available_points >= 5)
				{
					double mcx = 0.0, mcy = 0.0;
					for (int i = 0; i < n_available_points; i++ )
					{
						pcl::PointXYZRGBL p = cloud_available_voxel_OBB->points[i];
						mcx += p.x;
						mcy += p.y;
					}
					point_OBB_dest.x = mcx / n_available_points;
					point_OBB_dest.y = mcy / n_available_points;
					point_OBB_dest.z = vertex_OBB_max_XYZ.z;

					if (point_OBB_dest.y < (vertex_OBB_min_XYZ.y + 0.25)) 
					{
						point_OBB_dest.y = vertex_OBB_min_XYZ.y + 0.25;
					}
					if (point_OBB_dest.y > (vertex_OBB_max_XYZ.y - 0.75)) 
					{
						point_OBB_dest.y = vertex_OBB_max_XYZ.y - 0.75;
					}

					if (point_OBB_dest.x < (vertex_OBB_min_XYZ.x + 0.75)) point_OBB_dest.x = vertex_OBB_min_XYZ.x + 0.75;
					if (point_OBB_dest.x > (vertex_OBB_max_XYZ.x - 0.75)) point_OBB_dest.x = vertex_OBB_max_XYZ.x - 0.75;
				}
			}
		}

		point_OBB_falling.x = 0;//center_p.x;
		point_OBB_falling.y = 0.5 * (vertex_OBB_min_XYZ.y + vertex_OBB_max_XYZ.y);
		point_OBB_falling.z = vertex_OBB_max_XYZ.z + 0.5;
		point_OBB_falling.r = 255;
		point_OBB_falling.g = 0;
		point_OBB_falling.b = 0;
		
		cloud_fall_point_OBB->points.clear();
		cloud_fall_point_OBB->points.push_back(point_OBB_falling);

		//std::cout << "   ==> falling_found:    " << falling_found << std::endl;
		// falling outside y boundary, set dest to the center y
		if (falling_found)
		{
			//std::cout << "   ==> point_OBB_falling.y:    " << point_OBB_falling.y << " vertex_OBB_min_XYZ.y:    " << vertex_OBB_min_XYZ.y << " vertex_OBB_max_XYZ.y:    " << vertex_OBB_max_XYZ.y << std::endl;
			if (point_OBB_falling.y < vertex_OBB_min_XYZ.y)
			{
				point_OBB_dest.y = 0.5 * (vertex_OBB_min_XYZ.y + vertex_OBB_max_XYZ.y);
			} else
			if (point_OBB_falling.y > (vertex_OBB_max_XYZ.y - 0.5))
			{
				point_OBB_dest.y = vertex_OBB_min_XYZ.y + 1.0;
			}
			//std::cout << "   ==> point_OBB_dest.y:    " << point_OBB_dest.y << std::endl;
			//if (
			//	point_OBB_falling.y > (vertex_OBB_min_XYZ.y + 0.25) && 
			//	point_OBB_falling.y < (vertex_OBB_max_XYZ.y - 0.25)
			//)
			//{
			//	y_in_boundary = true;
			//}	
			if (
				point_OBB_falling.y < (vertex_OBB_min_XYZ.y - 0.25) 
				||
				point_OBB_falling.y > (vertex_OBB_max_XYZ.y + 0.25)
			)
			{
				y_in_boundary = false;
			} else 
			{
				//
			}
			//std::cout << "   ==> y_in_boundary:    " << y_in_boundary << std::endl;
		}

		cloud_dest_point_OBB->points.push_back(point_OBB_dest);
	}
	//std::cout << "   ==> final process ... " << std::endl;

	pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_OBB);
	pcl::transformPointCloud (*cloud_vertex_OBB, *cloud_vertex, t_OBB);
	pcl::transformPointCloud (*cloud_available_voxel_OBB, *cloud_available_voxel, t_OBB);
	pcl::transformPointCloud (*cloud_occupied_voxel_OBB, *cloud_occupied_voxel, t_OBB);
	pcl::transformPointCloud (*cloud_fall_point_OBB, *cloud_fall_point, t_OBB);
	pcl::transformPointCloud (*cloud_dest_point_OBB, *cloud_dest_point, t_OBB);
	pcl::transformPointCloud (*cloud_fall_points_OBB, *cloud_fall_points_OBB, t_OBB); //for debug only
	
	x_in_boundary = false;
	if (vertex_computed)
	{
		pcl::PointXYZRGBL &point_falling = cloud_fall_point->points[0];
		// c_p should not be used for center
		//pcl::PointXYZRGBL c_p = _cloud_center->points[0];

		pcl::PointXYZRGBL vertex_min_XYZ, vertex_max_XYZ;
		pcl::getMinMax3D(*cloud_vertex, vertex_min_XYZ, vertex_max_XYZ);
		//std::cout << "   ===> pre-adjustment ==> c_p.x: " << c_p.x << " point_falling.x: " << point_falling.x << ", falling_found: " << falling_found << std::endl;
		//std::cout << "   ===> pre-adjustment ==> point_falling.x: " << point_falling.x << ", falling_found: " << falling_found << std::endl;
		// correct falling x if found in error
		if (falling_found) 
		{
			//float d_center_2_fall = point_falling.x - c_p.x;
			//std::cout << "   ==> c_p.x: " << c_p.x << " point_falling.x: " << point_falling.x << " d_center_2_fall: " << d_center_2_fall << std::endl;
			float d_center_2_fall = std::fabs(point_falling.x);
			//std::cout << "      ==> point_falling.x: " << point_falling.x << " d_center_2_fall: " << d_center_2_fall << std::endl;
			//if (fabs(d_center_2_fall) > 1.25) must be error detection of falling point, get rid of it
			if (fabs(d_center_2_fall) > 0.75)
			{
				//std::cout << "      ==> reset back the falling point to center as the detected falling point looks not right: " << std::endl;
				point_falling.x = 0; // c_p.x; // set falling point back to the center
				point_falling.y = 0.5 * (vertex_min_XYZ.y + vertex_max_XYZ.y);
			}
		}
		// set falling x if falling not found at all
		else 
		{
			point_falling.x = 0.0;
			point_falling.y = 0.5 * (vertex_min_XYZ.y + vertex_max_XYZ.y);
		}
		//std::cout << "   ===> post-adjustment ==> c_p.x: " << c_p.x << " point_falling.x: " << point_falling.x << std::endl;
		pcl::transformPointCloud (*cloud_fall_point, *cloud_fall_point_OBB, t_OBB_inverse);
		point_OBB_falling = cloud_fall_point_OBB->points[0];

		//std::cout << "point_OBB_falling.x: " << point_OBB_falling.x << " vertex_OBB_min_XYZ.x: " << vertex_OBB_min_XYZ.x << " vertex_OBB_max_XYZ.x: " << vertex_OBB_max_XYZ.x << std::endl;
		if (
			point_OBB_falling.x > (vertex_OBB_min_XYZ.x + 0.5) && 
			point_OBB_falling.x < (vertex_OBB_max_XYZ.x - 0.5)
		)
		{
			x_in_boundary = true;
		}	

		//std::cout << "   ===> falling_found: " << falling_found << std::endl;
		float vertex_center_y = 0.5 * (vertex_OBB_min_XYZ.y + vertex_OBB_max_XYZ.y);
		//std::cout << "   ===> vertex_center_y: " << vertex_center_y << std::endl;
		if (! falling_found) 
		{
			// needs adjust distance
			//std::cout << "   ===> before adjustment, point_falling.y: " << point_falling.y << std::endl;
			//std::cout << "   ===> truck_center_distance_estimated + 0.5: " << truck_center_distance_estimated + 0.5 << std::endl;
			if (vertex_center_y > (truck_center_distance_estimated + 0.5))
			{
				if (sprout_y_moves < 5)
				{
					y_in_boundary = false;
					point_falling.y = truck_center_distance_estimated;
					//std::cout << "   ===> set point_falling.y: " << truck_center_distance_estimated << std::endl;
				} else if (sprout_y_moves >= 5)
				{
				}
			} else if (vertex_center_y < (truck_center_distance_estimated - 0.5))
			{
				if (sprout_y_moves > -5)
				{
					y_in_boundary = false;
					point_falling.y = truck_center_distance_estimated;
					//std::cout << "   ===> set point_falling.y: " << truck_center_distance_estimated << std::endl;
				} else if (sprout_y_moves <= -5)
				{
				}
			}
			//std::cout << "   ===> after adjustment, point_falling.y: " << point_falling.y << std::endl;
		}

		pcl::PointXYZRGBL &point_dest = cloud_dest_point_OBB->points[0];
	}
	
	//std::cout << "   ==> process completed. " << std::endl;
}

void detect_truck_behind(
    Eigen::Affine3d t_sensor,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_raw,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_center,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_downsampled,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_downsampled,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_ground_plane,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cropped,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_content_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_1_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_1_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_2_OBB,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_points_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point,
    bool &vertex_computed,
    bool &falling_found,
    bool &dest_found,
	bool &x_in_boundary, 
	bool &y_in_boundary, 
	bool balanced_fill,
	bool distance_ctrl,
	float &sprout_angle_Z,
	float sprout_angle_available
)
{
	add_center_ref(_cloud_center, 25.0);
	pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_sensor);

    int n_cropped_points = cloud_cropped->points.size();

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cropped_pre_filtered(new pcl::PointCloud<pcl::PointXYZRGBL>);
	*cloud_cropped_pre_filtered = *cloud_cropped;

	/////////////////////////////////////////////////////////
	pcl::PointXYZRGBL _tmp_min_XYZ, _tmp_max_XYZ;
	pcl::getMinMax3D(*cloud_cropped, _tmp_min_XYZ, _tmp_max_XYZ);
	//std::cout << "cloud_cropped => _tmp_min_XYZ.z: " << _tmp_min_XYZ.z << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_cut(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::CropBox<pcl::PointXYZRGBL> box_filter;
	box_filter.setInputCloud(cloud_cropped);
	box_filter.setMin(Eigen::Vector4f(-100.0,  4.0, -100.0, 1.0));
	//box_filter.setMax(Eigen::Vector4f( 100.0,  6.0,  3.0, 1.0));
	box_filter.setMax(Eigen::Vector4f( 100.0,  7.0,  _tmp_min_XYZ.z + 2.0, 1.0));
	box_filter.filter(*cloud_tmp_cut);
	*cloud_cropped = *cloud_tmp_cut;
	///////////////////////////////////////////////////////////

	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_1(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_2(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_3(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_4(new pcl::PointCloud<pcl::PointXYZRGBL>);

	split_cloud_4(
		cloud_cropped,
		_cloud_1,
		_cloud_2,
		_cloud_3,
		_cloud_4
	);
		
	//std::thread th_filter_1(&filter_by_clustering, _cloud_1, _cloud_1, 0.125, 30, 100000);
	//std::thread th_filter_2(&filter_by_clustering, _cloud_2, _cloud_2, 0.125, 30, 100000);
	//std::thread th_filter_3(&filter_by_clustering, _cloud_3, _cloud_3, 0.125, 30, 100000);
	//std::thread th_filter_4(&filter_by_clustering, _cloud_4, _cloud_4, 0.125, 30, 100000);
	std::thread th_filter_1(&filter_by_clustering, _cloud_1, _cloud_1, 0.125, 60, 100000);
	std::thread th_filter_2(&filter_by_clustering, _cloud_2, _cloud_2, 0.125, 60, 100000);
	std::thread th_filter_3(&filter_by_clustering, _cloud_3, _cloud_3, 0.125, 60, 100000);
	std::thread th_filter_4(&filter_by_clustering, _cloud_4, _cloud_4, 0.125, 60, 100000);
	th_filter_1.join();
	th_filter_2.join();
	th_filter_3.join();
	th_filter_4.join();	
	
	*cloud_cropped = *_cloud_1;
	*cloud_cropped += *_cloud_2;
	*cloud_cropped += *_cloud_3;
	*cloud_cropped += *_cloud_4;
		
	//filter_by_clustering(cloud_cropped, cloud_cropped, 0.0625, 15, 100000);
	n_cropped_points = cloud_cropped->points.size();
	//std::cout << "   ==> after filter_by_clustering => n_cropped_points: " << n_cropped_points << std::endl;

	int n_downsampled_points = 0;
	if (n_cropped_points > 0) 
	{
		pcl::UniformSampling<pcl::PointXYZRGBL> uniform_sampling;
		uniform_sampling.setInputCloud(cloud_cropped);
		uniform_sampling.setRadiusSearch(0.125);
		uniform_sampling.filter (*cloud_downsampled);
		n_downsampled_points = cloud_downsampled->points.size();
	}
	//std::cout << "==> n_downsampled_points: " << n_downsampled_points << std::endl;

	int n_filtered_points = 0;
	if (n_downsampled_points > 0)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
		//sor.setMeanK(30);
		sor.setMeanK(15);
		sor.setStddevMulThresh(0.5);

		sor.setInputCloud(cloud_downsampled);
		sor.filter(*cloud_filtered);

		n_filtered_points = cloud_filtered->points.size();
	}
	//std::cout << "==> n_filtered_points: " << n_filtered_points << std::endl;

	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> likely_clusters;
	std::vector<pcl::PointXYZRGBL> cluster_mins;
	std::vector<pcl::PointXYZRGBL> cluster_maxs;

	int n_cluster_points = 0;
	int n_edge_points = 0;
	int n_line_points = 0;
	
	bool valid_cluster_found = false;
	if (n_filtered_points > 0) 
	{
		find_likely_clusters(cloud_filtered, 0.25, 200, 50000, 1.8, 1.5, 0.4, likely_clusters, cluster_mins, cluster_maxs);
		int n_clusters = likely_clusters.size();
		//std::cout << "===> n_clusters: " << n_clusters << std::endl;

		if (n_clusters > 0)
		{
			std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> valid_clusters;
			std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> valid_edges;
			std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> valid_lines;
			for (int i = 0; i < n_clusters; i ++)
			{
				//std::cout << "   ==> evaluate cluster: " << i << std::endl;
				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cluster = likely_clusters[i];

				float line_length = 0.0;
				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
				pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
				
				//bool cluster_valid = find_likely_line(cluster, _cloud_center, cloud_edge_tmp, cloud_line_tmp, 0.5, false, true, 0.75, 1.6, 2.5, 1.0, 0.0, 4.0, 1.04, line_length);
				bool cluster_valid = find_likely_line(cluster, _cloud_center, cloud_edge_tmp, cloud_line_tmp, 0.5, false, true, 0.75, 1.6, 2.5, 1.0, 0.0, 4.0, 0.6, line_length);
				//std::cout << "1      ===> cluster " << i << " -> line_length: " << line_length << " valid: " << cluster_valid << std::endl;
				if (! cluster_valid)
				{
					//cluster_valid = find_likely_line(cluster, _cloud_center, cloud_edge_tmp, cloud_line_tmp, 0.5, true, false, 0.75, 1.6, 2.5, 1.0, 0.0, 4.0, 1.0, line_length);
					cluster_valid = find_likely_line(cluster, _cloud_center, cloud_edge_tmp, cloud_line_tmp, 0.5, true, false, 0.75, 1.6, 2.5, 1.0, 0.0, 4.0, 0.6, line_length);
					//std::cout << "2      ===> cluster " << i << " -> line_length: " << line_length << " valid: " << cluster_valid << std::endl;
				}

				//std::cout << "      ===> cluster " << i << " -> line_length: " << line_length << " valid: " << cluster_valid << std::endl;
				if (cluster_valid)
				{
					valid_clusters.push_back(cluster);
					valid_edges.push_back(cloud_edge_tmp);
					valid_lines.push_back(cloud_line_tmp);
					valid_cluster_found = true;
				} else {// for debug
					*cloud_closest_cluster = *cluster;
					*cloud_edge = *cloud_edge_tmp;
				}
			}
			if (valid_clusters.size() == 1)
			{
				*cloud_closest_cluster = *valid_clusters[0];
				*cloud_edge = *valid_edges[0];
				*cloud_line = *valid_lines[0];
			} else if (valid_clusters.size() > 1)
			{
				int max_line_points = 0;
				for (int i = 0; i < valid_clusters.size(); i ++)
				{
					pcl::PointCloud<pcl::PointXYZRGBL>::Ptr line = valid_lines[i];
					int ps = line->points.size();
					if (ps > max_line_points)
					{
						max_line_points = ps;
						*cloud_closest_cluster = *valid_clusters[i];
						*cloud_edge = *valid_edges[i];
						*cloud_line = *valid_lines[i];
					}
				}
			}
		}
		if (valid_cluster_found)
		{
			n_cluster_points = cloud_closest_cluster->points.size();
			//std::cout << "==> n_cluster_points: " << n_cluster_points << std::endl;
			n_edge_points = cloud_edge->points.size();
			//std::cout << "==> n_edge_points: " << n_edge_points << std::endl;
			n_line_points = cloud_line->points.size();
			//std::cout << "==> n_line_points: " << n_line_points << std::endl;
		}		
	}
	//std::cout << "==> valid_cluster_found: " << valid_cluster_found << " n_cluster_points: " << n_cluster_points << std::endl;
	Eigen::Affine3f t_OBB = Eigen::Affine3f::Identity();
	pcl::PointXYZRGBL cluster_OBB_min_XYZ, cluster_OBB_max_XYZ;

	pcl::PointXYZRGBL line_OBB_min_XYZ, line_OBB_max_XYZ;
	
	if (valid_cluster_found)
	{
		pcl::PointXYZRGBL min_OBB, max_OBB, position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBL> f;
		f.setInputCloud(cloud_line);
		f.compute();
		f.getOBB(min_OBB, max_OBB, position_OBB, rotational_matrix_OBB);
		//std::cout << "   position_OBB  x:: " << position_OBB.x << " y: " << position_OBB.y << " z: " << position_OBB.z << std::endl;

		t_OBB = Eigen::Affine3f::Identity();
		t_OBB.rotate (rotational_matrix_OBB);
		t_OBB.translation() << position_OBB.x, position_OBB.y, position_OBB.z;

		Eigen::Affine3f t_OBB_inverse = t_OBB.inverse();

		pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_OBB_inverse);
		pcl::PointXYZRGBL &cp = _cloud_center->points[0];
		if (cp.y < 0)
		{
			//std::cout << "==> rotate OBB to positive side ... " << std::endl;
			Eigen::AngleAxisf r;
			r =
				  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
			Eigen::Affine3f t_ = Eigen::Affine3f::Identity();
			t_.rotate (r);

			pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_);
			//std::cout << "==> after rotate, x: " << cp.x << " y: " << cp.y << " z: " << cp.z << std::endl;

			t_OBB_inverse = t_ * t_OBB_inverse;

			Eigen::Affine3f t_inverse = t_.inverse();
			t_OBB = t_OBB * t_inverse;
		}
		
		pcl::transformPointCloud (*cloud_line, *cloud_line_OBB, t_OBB_inverse);
		pcl::transformPointCloud (*cloud_closest_cluster, *cloud_closest_cluster_OBB, t_OBB_inverse);
		// to use cloud_cropped_pre_filtered, as the falling content been filtered out
		pcl::transformPointCloud (*cloud_cropped_pre_filtered, *cloud_content_OBB, t_OBB_inverse);

		// further validate cluster
		pcl::getMinMax3D(*cloud_closest_cluster_OBB, cluster_OBB_min_XYZ, cluster_OBB_max_XYZ);
		float cluster_OBB_dx = cluster_OBB_max_XYZ.x - cluster_OBB_min_XYZ.x;
		//std::cout << "==> cluster_OBB_dx: " << cluster_OBB_dx << std::endl;
		//std::cout << "==> line_OBB_min_XYZ.y: " << line_OBB_min_XYZ.y << std::endl;
		//std::cout << "==> line_OBB_min_XYZ.z: " << line_OBB_min_XYZ.z << std::endl;
		if (cluster_OBB_dx > 3.0) 
		{
			//std::cout << "   ==> cluster invalid cluster_OBB_dx: " << cluster_OBB_dx << std::endl;
			valid_cluster_found = false;
		} else 
		{
			pcl::getMinMax3D(*cloud_line_OBB, line_OBB_min_XYZ, line_OBB_max_XYZ);
			
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
			pcl::PassThrough<pcl::PointXYZRGBL> pass;
			pass.setFilterLimitsNegative (false);
			pass.setInputCloud (cloud_closest_cluster_OBB);
			pass.setFilterFieldName ("y");
			//pass.setFilterLimits (-65.0, line_OBB_min_XYZ.z - 1.0); 
			//pass.setFilterLimits (-65.0, line_OBB_min_XYZ.y - 1.0); 
			pass.setFilterLimits (-65.0, line_OBB_min_XYZ.y - 0.6); 
			pass.filter (*cloud_tmp);

			//std::cout << "   ==> validate OBB cluster => cloud_tmp: " << cloud_tmp->points.size() << std::endl;

			if (cloud_tmp->points.size() > 0)
			{
				*cloud_closest_cluster_OBB = *cloud_line_OBB;
			} else 
			{
				valid_cluster_found = false;
			}
		}
	}
	//std::cout << "==> OBB valid_cluster_found: " << valid_cluster_found << std::endl;

	bool top_line_found = false;
	if (valid_cluster_found)
	{
		*cloud_top_line_1_OBB = *cloud_closest_cluster_OBB;
		*cloud_wall_1_OBB = *cloud_closest_cluster_OBB;

		pcl::PointXYZRGBL top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ;
		pcl::getMinMax3D(*cloud_top_line_1_OBB, top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ);

		pcl::PointXYZRGBL wall_2_point;

		wall_2_point.x = 0.5 * (top_line_1_OBB_min_XYZ.x + top_line_1_OBB_max_XYZ.x);
		wall_2_point.y = top_line_1_OBB_min_XYZ.y + 4.0; // 4.0 m is the standard length of truck
		wall_2_point.z = top_line_1_OBB_max_XYZ.z;
		cloud_top_line_2_OBB->points.push_back(wall_2_point);

		top_line_found = true;
	}
	//std::cout << "==> top_line_found: " << top_line_found << std::endl;

	pcl::PointXYZRGBL top_OBB_min_XYZ, top_OBB_max_XYZ;	
	pcl::PointXYZRGBL vertex_OBB_min_XYZ, vertex_OBB_max_XYZ;
	pcl::PointXYZRGBL content_OBB_min, content_OBB_max;
	pcl::PointXYZRGBL point_OBB_falling;
	pcl::PointXYZRGBL point_OBB_dest;
	vertex_computed = false;
	falling_found = false;
	dest_found = false;
	int n_content_points = 0;
	if (top_line_found)
	{

		pcl::PointXYZRGBL top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ;
		pcl::getMinMax3D(*cloud_top_line_1_OBB, top_line_1_OBB_min_XYZ, top_line_1_OBB_max_XYZ);

		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane_top_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
		*cloud_plane_top_OBB = *cloud_top_line_1_OBB + *cloud_top_line_2_OBB;

		pcl::getMinMax3D(*cloud_plane_top_OBB, top_OBB_min_XYZ, top_OBB_max_XYZ);

		vertex_computed = compute_vertex(top_OBB_min_XYZ, top_OBB_max_XYZ, top_line_1_OBB_max_XYZ.z, cloud_vertex_OBB);
		pcl::getMinMax3D(*cloud_vertex_OBB, vertex_OBB_min_XYZ, vertex_OBB_max_XYZ);

		pcl::CropBox<pcl::PointXYZRGBL> box_filter;
		box_filter.setInputCloud(cloud_content_OBB);
		box_filter.setMin(Eigen::Vector4f(vertex_OBB_min_XYZ.x, vertex_OBB_min_XYZ.y, -10.0, 1.0));
		box_filter.setMax(Eigen::Vector4f(vertex_OBB_max_XYZ.x, vertex_OBB_max_XYZ.y,  vertex_OBB_max_XYZ.z + 1.0, 1.0));
		box_filter.filter(*cloud_content_OBB);

		n_content_points = cloud_content_OBB->points.size();
	}
	pcl::PointXYZRGBL falling_OBB_min, falling_OBB_max;
	pcl::PointXYZRGBL available_OBB_min, available_OBB_max;
	float volumn_total,volumn_available;
	int n_fall_points = 0;
	float fall_min_x = content_OBB_min.x;
	float fall_max_x = content_OBB_max.x;
	float fall_min_y = content_OBB_min.y;
	float fall_max_y = content_OBB_max.y;
	x_in_boundary = false;
	if (vertex_computed)
	{
		if (n_content_points > 0)
		{
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
			sor.setMeanK(15);
			sor.setStddevMulThresh(1.0);
			sor.setInputCloud(cloud_content_OBB);
			sor.filter(*cloud_content_OBB);

			pcl::getMinMax3D(*cloud_content_OBB, content_OBB_min, content_OBB_max);
			
			pcl::PassThrough<pcl::PointXYZRGBL> pass;
			pass.setFilterLimitsNegative (false);
			pass.setInputCloud (cloud_content_OBB);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (vertex_OBB_max_XYZ.z + 0.5, vertex_OBB_max_XYZ.z + 1.0); 
			pass.filter (*cloud_fall_points_OBB);
			n_fall_points = cloud_fall_points_OBB->points.size();

			if (n_fall_points > 0)
			{
				pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
				sor.setMeanK(5);
				sor.setStddevMulThresh(1.0);
				sor.setInputCloud(cloud_fall_points_OBB);
				sor.filter(*cloud_fall_points_OBB);
				sor.setInputCloud(cloud_fall_points_OBB);
				sor.filter(*cloud_fall_points_OBB);
				n_fall_points = cloud_fall_points_OBB->points.size();
			}
			//std::cout << "===============> cloud_fall_points_OBB: " << n_fall_points << std::endl;
		}

		pcl::PointXYZRGBL center_p = _cloud_center->points[0];
		point_OBB_falling.x = center_p.x;
		point_OBB_falling.y = 0.5 * (vertex_OBB_min_XYZ.y + vertex_OBB_max_XYZ.y);//center_p.y;
		point_OBB_falling.z = vertex_OBB_max_XYZ.z;
		point_OBB_falling.r = 255;
		point_OBB_falling.g = 0;
		point_OBB_falling.b = 0;		

		//if (n_fall_points >= 15)
		if (n_fall_points >= 6)
		{
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_largest_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
			//find_largest_cluster(cloud_fall_points_OBB, cloud_largest_cluster, 0.25, 15, 10000);
			find_largest_cluster(cloud_fall_points_OBB, cloud_largest_cluster, 0.25, 5, 10000);
			*cloud_fall_points_OBB = *cloud_largest_cluster;
			n_fall_points = cloud_fall_points_OBB->points.size();

			if (n_fall_points > 0)
			{				
				pcl::getMinMax3D(*cloud_fall_points_OBB, falling_OBB_min, falling_OBB_max);
				fall_min_x = falling_OBB_min.x;
				fall_max_x = falling_OBB_max.x;
				fall_min_y = falling_OBB_min.y;
				fall_max_y = falling_OBB_max.y;
				//std::cout << "   ==> fall_min_x: " << fall_min_x << std::endl;
				//std::cout << "   ==> fall_max_x: " << fall_max_x << std::endl;

				double mcx = 0.0, mcy = 0.0, mcz = 0.0;
				for (int i = 0; i < n_fall_points; i++ )
				{
					pcl::PointXYZRGBL p = cloud_fall_points_OBB->points[i];
					mcx += p.x;
					mcy += p.y;
					mcz += p.z;
				}
				point_OBB_falling.x = mcx / n_fall_points;
				point_OBB_falling.y = mcy / n_fall_points + 1.0;
				point_OBB_falling.z = mcz / n_fall_points;

				falling_found = true;

				if (
					point_OBB_falling.x > vertex_OBB_min_XYZ.x && 
					point_OBB_falling.x < vertex_OBB_max_XYZ.x
				)
				{
					x_in_boundary = true;
				}				

				//std::cout << "   ==> point_OBB_falling.y: " << point_OBB_falling.y << std::endl;
			}
		}

		cloud_fall_point_OBB->points.push_back(point_OBB_falling);

		// for behind, probably do not need to find dest point, but just point to the center of vertex
		// and even for side, we could have equally-fill and fill-in-center-only option,
		// for fill-in-center-only, we could just use vertex center.

		point_OBB_dest.r = 255;
		point_OBB_dest.g = 255;
		point_OBB_dest.b = 255;

		point_OBB_dest.x = 0.5 * (vertex_OBB_min_XYZ.x + vertex_OBB_max_XYZ.x);
		point_OBB_dest.y = 0.5 * (vertex_OBB_min_XYZ.y + vertex_OBB_max_XYZ.y);
		point_OBB_dest.z = vertex_OBB_max_XYZ.z; 
		dest_found = true;

		cloud_dest_point_OBB->points.push_back(point_OBB_dest);
	}
	
	pcl::transformPointCloud (*_cloud_center, *_cloud_center, t_OBB);
	pcl::transformPointCloud (*cloud_vertex_OBB, *cloud_vertex, t_OBB);
	pcl::transformPointCloud (*cloud_available_voxel_OBB, *cloud_available_voxel, t_OBB);
	pcl::transformPointCloud (*cloud_occupied_voxel_OBB, *cloud_occupied_voxel, t_OBB);
	pcl::transformPointCloud (*cloud_fall_point_OBB, *cloud_fall_point, t_OBB);
	pcl::transformPointCloud (*cloud_dest_point_OBB, *cloud_dest_point, t_OBB);
	
}

void detect_truck(
    Eigen::Affine3d t_sensor,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_raw,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_center,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_downsampled,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_downsampled,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_ground_plane,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cropped,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_content_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_1_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_1_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_2_OBB,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel,

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_points_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point_OBB,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point,
    bool &vertex_computed,
    bool &falling_found,
    bool &dest_found,
	bool &x_in_boundary, 
	bool &y_in_boundary, 
	bool balanced_fill,
	bool distance_ctrl,
	bool truck_behind,
	float &sprout_angle_Z,
	float sprout_angle_available
)
{
	if (truck_behind)
	{
		detect_truck_behind(
			t_sensor,
			cloud_raw,
			_cloud_center,
			_cloud_downsampled,
			cloud_downsampled,
			cloud_ground_plane,
			cloud_cropped,
			cloud_filtered,
			cloud_closest_cluster,
			cloud_edge,
			cloud_line,
			cloud_line_OBB,
			cloud_closest_cluster_OBB,
			cloud_content_OBB,
			cloud_wall_1_OBB,
			cloud_top_line_1_OBB,
			cloud_top_line_2_OBB,

			cloud_vertex_OBB,
			cloud_vertex,

			cloud_available_voxel_OBB,
			cloud_available_voxel,

			cloud_occupied_voxel_OBB,
			cloud_occupied_voxel,

			cloud_fall_points_OBB,
			cloud_fall_point_OBB,
			cloud_fall_point,
			cloud_dest_point_OBB,
			cloud_dest_point,

			vertex_computed,
			falling_found,
			dest_found,
			x_in_boundary,
			y_in_boundary,
			balanced_fill,
			distance_ctrl,

			sprout_angle_Z,
			sprout_angle_available
		);
	} else 
	{
		detect_truck_atside(
			t_sensor,
			cloud_raw,
			_cloud_center,
			_cloud_downsampled,
			cloud_downsampled,
			cloud_ground_plane,
			cloud_cropped,
			cloud_filtered,
			cloud_closest_cluster,
			cloud_edge,
			cloud_line,
			cloud_line_OBB,
			cloud_closest_cluster_OBB,
			cloud_content_OBB,
			cloud_wall_1_OBB,
			cloud_top_line_1_OBB,
			cloud_top_line_2_OBB,

			cloud_vertex_OBB,
			cloud_vertex,

			cloud_available_voxel_OBB,
			cloud_available_voxel,

			cloud_occupied_voxel_OBB,
			cloud_occupied_voxel,

			cloud_fall_points_OBB,
			cloud_fall_point_OBB,
			cloud_fall_point,
			cloud_dest_point_OBB,
			cloud_dest_point,

			vertex_computed,
			falling_found,
			dest_found,
			x_in_boundary,
			y_in_boundary,
			balanced_fill,
			distance_ctrl,

			sprout_angle_Z,
			sprout_angle_available
		);
	}
}
