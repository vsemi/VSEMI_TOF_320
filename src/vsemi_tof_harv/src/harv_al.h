#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

struct Pose
{
	double x;
	double y;
	double z;
	double w;
};

Eigen::Affine3d compute_rotation_matrix(Pose pose);

void split_cloud_4_z(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4
);
void split_cloud_4(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_1,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_2,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_3,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_4
);
void filter_by_stat(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_input,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_output,
	int meanK,
	int stddevMulThresh
);
bool find_ground_plane(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	float angle,
	float distanceThreshold,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane,
	Eigen::Affine3d &transform_floor
);
void find_likely_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_likely_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	float min_dx, float min_dz,
	pcl::PointXYZRGBL &cluster_min_XYZ, pcl::PointXYZRGBL &cluster_max_XYZ
);
void find_likely_clusters(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	float min_dx, float min_dy, float min_dz,
	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> &likely_clusters,
	std::vector<pcl::PointXYZRGBL> &mins,
	std::vector<pcl::PointXYZRGBL> &maxs
);
void find_largest_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_largest_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize
);
bool find_edge_points(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
	float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
	float offset_z, float thick,
	bool near_edge,
	bool far_edge
);
void extract_line_points(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
	float distanceThreshold,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge_remains
);
bool validate_cluster_edge_line(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line,
	float cluster_min_y,
	float min_dx,
	float max_slope,
	float min_y_shift,
	float max_y_shift
);
float find_longest_cluster(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_longest_cluster,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize
);
bool find_top_line_OBB(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_OBB,
	float wall_length, float wall_height, float wall_min_z,
	float step,
	bool ignore_distance,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_OBB
);
bool find_top_lines_OBB(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster_OBB,
	pcl::PointXYZRGBL cluster_OBB_min_XYZ, pcl::PointXYZRGBL cluster_OBB_max_XYZ,
	float thick, float min_length_tolerance, float default_width, 
	pcl::PointXYZRGBL line_OBB_min_XYZ, pcl::PointXYZRGBL line_OBB_max_XYZ,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_1_OBB,
	bool optimize_top_line,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_1_OBB,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_2_OBB
);
bool compute_vertex(
	pcl::PointXYZRGBL min_OBB, pcl::PointXYZRGBL max_OBB,
	float z,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex_OBB
);
void compute_falling_position_OPP(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_OBB,
	float max_z,
	pcl::PointXYZRGBL min_volume_OBB, pcl::PointXYZRGBL max_volume_OBB,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_OBB_falling
);
bool compute_volume_voxel_OPP(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_OBB,
	float max_z,
	pcl::PointXYZRGBL min_volume_OBB, pcl::PointXYZRGBL max_volume_OBB,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_volume_voxel_OBB_avaiable
);
bool find_best_dest(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_best_dest,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize,
	pcl::PointXYZRGBL vertex_OBB_min_XYZ, 
	pcl::PointXYZRGBL vertex_OBB_max_XYZ,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cluster_OBB
);

void add_center_ref(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, float distance);

void filter_by_clustering(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_input,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_output,
	float clusterTolerance,
	int minClusterSize,
	int maxClusterSize
);
bool find_likely_line(
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr edge,
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr line,
	float &line_length
);
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
);
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
);
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
);
