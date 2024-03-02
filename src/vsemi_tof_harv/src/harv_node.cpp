#include <stdio.h>
#include <iostream>
#include <time.h>
#include <string>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "cartesian_transform.hpp"
#include "interface.hpp"

#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

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

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "color.hpp"

#include "imu.h"

#include "util.h"
#include "harv_al.h"

#include "socketcan.h"
#include "ctrl.h"
#include "serial.hpp"

//#include <vsemi_tof_harv/vsemi_tof_harvConfig.h>

using namespace espros;

int imageType = 1; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int lensType = 0;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
bool medianFilter = false;
bool averageFilter = false;
double temporalFilterFactor = 0;
int temporalFilterThreshold = 0;
int edgeThreshold = 0;
int temporalEdgeThresholdLow = 0;
int temporalEdgeThresholdHigh = 0;
int interferenceDetectionLimit = 0;
bool startStream = true;
bool useLastValue = false;
bool publishPointCloud = true;

int channel = 0;
int frequencyModulation = 2;
int int0 = 3400, int1 = 200, int2 = 200, intGr = 25000;
int hdr_mode = 0;
int offset = 0, minAmplitude = 60;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;
int old_lensCenterOffsetX = 0;
int old_lensCenterOffsetY = 0;

int min_integration_time  = 2600;
int max_integration_time  = 4000;
int step_integration_time = 200;

int max_tolerance_overflow = 60;
int max_tolerance_low_amplitude = 400;

bool data_replay = false;

int roi_leftX = 0;
int roi_topY = 0;
int roi_rightX = 319;
int roi_bottomY = 239;

const int width   = 320;
const int width2  = 160;
const int height  = 240;
const int height2 = 120;
const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

float truck_center_distance_estimated = 3.0;

struct Point2D {
	int x;
	int y;
};

std::vector<Point2D> points_valid;
std::vector<Point2D> points_low_amplitude;
std::vector<Point2D> points_saturation_overflow;

bool mode_manual = true;
bool tof_started = false;

int testFrame;

std::string video_camera = "";

uint32_t frameSeq;
boost::signals2::connection connectionFrames;
boost::signals2::connection connectionCameraInfo;

sensor_msgs::CameraInfo cameraInfo;

bool video_ok = false;

ImageColorizer *imageColorizer;

static std::string strFrameID = "sensor_frame";

CartesianTransform cartesianTransform;

Interface* interface;
cv::VideoCapture* videoCapture;

IMU* imu_0;
IMU* imu_sensor;
bool imu_enabled = false;

Eigen::Affine3f t_sensor_2_world = Eigen::Affine3f::Identity();

std::chrono::steady_clock::time_point st_time;
std::chrono::steady_clock::time_point en_time;
double interval, frame_rate;
int n_frames = 0;

bool ros_initialized = false;
bool startup_complited = false;

espros::CartesianTransform cartesianTransform_wf;
espros::CartesianTransform cartesianTransform_sf;

// - ros pub
ros::Publisher video_publisher;
ros::Publisher cloud_publisher;
ros::Publisher pose_camera_publisher;
ros::Publisher pose_spout_publisher;

std::string error_msg = "";

// - fs utilities ----------------------------------------------------------------------------------------
template<class Matrix>
void write_camera_pose(const char* filename, const Matrix& matrix){
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
};
template<class Matrix>
void read_camera_pose(const char* filename, Matrix& matrix){
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
};

inline bool file_exists (const std::string& name) {
	if (name != "") {
		struct stat buffer;
		return (stat (name.c_str(), &buffer) == 0);
	}

	return false;
}
// - end - fs utilities --------------------------------------------------------------------------------------------

void publish_cloud(ros::Publisher publisher, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, ros::Time time) {

	cloud->resize(cloud->points.size());
	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;

	pcl::PCLPointCloud2 cloud2;
	pcl::toPCLPointCloud2(*cloud, cloud2);

	sensor_msgs::PointCloud2 ros_msg_pointcloud2;
	pcl_conversions::fromPCL(cloud2, ros_msg_pointcloud2);

	ros_msg_pointcloud2.header.frame_id = strFrameID;
	ros_msg_pointcloud2.header.stamp      = time;

	publisher.publish(ros_msg_pointcloud2);
}
void publish_image(ros::Publisher publisher, cv::Mat image, ros::Time time) {

	sensor_msgs::Image ros_msg;
	ros_msg.header.frame_id = strFrameID;
	ros_msg.height = image.rows;
	ros_msg.width = image.cols;
	ros_msg.encoding = sensor_msgs::image_encodings::BGR8;
	ros_msg.step = image.cols * image.elemSize();
	size_t size = ros_msg.step * image.rows;
	ros_msg.data.resize(size);

	if (image.isContinuous())
	{
		memcpy((char*)(&ros_msg.data[0]), image.data, size);
	}
	else
	{
		uchar* ros_data_ptr = (uchar*)(&ros_msg.data[0]);
		uchar* cv_data_ptr = image.data;
		for (int i = 0; i < image.rows; ++i)
		{
			memcpy(ros_data_ptr, cv_data_ptr, ros_msg.step);
			ros_data_ptr += ros_msg.step;
			cv_data_ptr += image.step;
		}
	}

	ros_msg.header.stamp   = time;

	publisher.publish(ros_msg);
}

// - ctrl
int ctrl_signal_life_count = -1;
int ctrl_signal_life_assigned = -1;

int ctrl_current_ctrl_type = -1; // 0: x, 1: y

int comm_ctrl_type = 1; // 0: not available, 1: serial, 2: CAN
bool comm_ctrl_enabled = false;
// - SocketCan -------------------------------------------------------------------------------------------------------------
scpp::SocketCan socket_can;
std::string socket_can_name;

SerialConnection serial;
std::string serial_port_name;

std::string imu_port;

//////////////////////////////////////////////
//===================================================
void updateFrame(std::shared_ptr<Frame> frame);
void _pause_camera(bool p);

void updateCameraInfo(std::shared_ptr<CameraInfo> ci)
{
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
}

void initialise()
{
    frameSeq = 0;
    ros::NodeHandle nh("~");

    nh.getParam("fov_type", lensType);
    nh.getParam("int0", int0);
    nh.getParam("min_amplitude", minAmplitude);

    nh.getParam("frequency_modulation", frequencyModulation);
    nh.getParam("channel", channel);

    nh.getParam("min_integration_time", min_integration_time);
    nh.getParam("max_integration_time", max_integration_time);
    nh.getParam("step_integration_time", step_integration_time);

    nh.getParam("max_tolerance_overflow", max_tolerance_overflow);
    nh.getParam("max_tolerance_low_amplitude", max_tolerance_low_amplitude);

    nh.getParam("default_truck_center_distance", truck_center_distance_estimated);
	std::cout << "default_truck_center_distance: " << truck_center_distance_estimated << std::endl;

    nh.getParam("video_camera", video_camera);

	nh.getParam("imu_port", imu_port);
	//imu_port = "/dev/" + imu_port;

	nh.getParam("data_replay", data_replay);


	if (! data_replay)
	{
		video_publisher = nh.advertise<sensor_msgs::Image>("video", 1);
		cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBL> > ("cloud", 1);
		pose_camera_publisher     = nh.advertise<geometry_msgs::PoseStamped>("camera", 1);
		pose_spout_publisher     = nh.advertise<geometry_msgs::PoseStamped>("spout", 1);

	}

	if (! data_replay)
	{
		interface = new Interface();
		connectionCameraInfo = interface->subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
		connectionFrames = interface->subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });
	}

    cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType); //0.02 mm - sensor pixel size
    old_lensCenterOffsetX = lensCenterOffsetX;
    old_lensCenterOffsetY = lensCenterOffsetY;
    old_lensType = lensType;
}

void setParameters()
{
    std::cerr << "Set parameters: " << std::endl;

    interface->stopStream();
    interface->setMinAmplitude(minAmplitude);
    interface->setIntegrationTime(int0, int1, int2, intGr);
    interface->setHDRMode((uint8_t)hdr_mode);
    interface->setFilter(medianFilter, averageFilter, static_cast<uint16_t>(temporalFilterFactor * 1000), temporalFilterThreshold, edgeThreshold,
                        temporalEdgeThresholdLow, temporalEdgeThresholdHigh, interferenceDetectionLimit, useLastValue);

    uint8_t modIndex;
    if(frequencyModulation == 0) modIndex = 1;
    else if(frequencyModulation == 1)  modIndex = 0;
    else    modIndex = frequencyModulation;

    interface->setModulation(modIndex, channel);

    interface->setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY);

    if(startStream){

        if(imageType == Frame::GRAYSCALE) interface->streamGrayscale();
        else if(imageType == Frame::DISTANCE) interface->streamDistance();
        else if(imageType == Frame::AMPLITUDE) interface->streamDistanceAmplitude();
        else interface->streamDCS();

    }else{
        interface->stopStream();
    }

    if(old_lensCenterOffsetX != lensCenterOffsetX || old_lensCenterOffsetY != lensCenterOffsetY || old_lensType != lensType){
        cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType);
        old_lensCenterOffsetX = lensCenterOffsetX;
        old_lensCenterOffsetY = lensCenterOffsetY;
        old_lensType = lensType;
    }

    std::cerr << "lensType: " << lensType << std::endl;
    std::cerr << "int0: " << int0 << std::endl;
    std::cerr << "minAmplitude: " << minAmplitude << std::endl;
    std::cerr << "frequencyModulation: " << frequencyModulation << std::endl;
    std::cerr << "channel: " << channel << std::endl;

    std::cerr << "min_integration_time: " << min_integration_time << std::endl;
    std::cerr << "max_integration_time: " << max_integration_time << std::endl;
    std::cerr << "step_integration_time: " << step_integration_time << std::endl;

    std::cerr << "max_tolerance_overflow: " << max_tolerance_overflow << std::endl;
    std::cerr << "max_tolerance_low_amplitude: " << max_tolerance_low_amplitude << std::endl;

    std::cerr << std::endl;
}

void startStreaming()
{
    std::cerr << "Start Streaming, imageType: " << imageType << std::endl;
    switch(imageType) {
    case Frame::GRAYSCALE:
        interface->streamGrayscale();
        ROS_INFO("streaming grayscale");
        break;
    case Frame::DISTANCE:
        interface->streamDistance();
        ROS_INFO("streaming distance");
        break;
    case Frame::AMPLITUDE:
        interface->streamDistanceAmplitude();
        ROS_INFO("streaming distance-amplitude");
        break;
    case Frame::DCS:
        interface->streamDCS();
        ROS_INFO("streaming dcs");
        break;
    default:
        break;
    }
    tof_started = true;
    std::cerr << "Streaming started." << std::endl;
}

void stopStreaming()
{
    interface->stopStream();
    tof_started = false;
	std::cerr << "Streaming stopped." << std::endl;
}

// - display cv
bool vertex_computed_disp = false;
void display_video();

// - inception
static std::string package_path = "";
static std::string pose_file_path = "";
Eigen::Affine3d t_calibration = Eigen::Affine3d::Identity();
float camera_height = 4.5;

float sensor_height = 3.25;
float truck_height  = 2.5;
//float truck_height  = 3.0;

bool truck_behind  = false;
bool balanced_fill = true;
bool distance_ctrl = true;

float sprout_angle_Z;
float sprout_angle_available;

int sprout_y_moves = 0;

Eigen::Affine3d t_sensor = Eigen::Affine3d::Identity();

pcl::PointXYZRGBL display_ctr_p1;
pcl::PointXYZRGBL display_ctr_p2;
bool display_ctrl_available = false;

float ground_max_z = -3.5;//-4.0;
float angle_cali_x = 80.0;
float angle_cali_y = angle_cali_x * 240.0 / 320.0;

float TRUCK_HEIGHT = 2.0;
//float TRUCK_HEIGHT = 3.0;
float crop_min = ground_max_z + 1.0;
float crop_max = crop_min + TRUCK_HEIGHT;

bool vertex_computed = false;
bool falling_found = false;
bool dest_found = false;
bool falling_point_not_full = false;
bool x_in_boundary = false;
bool y_in_boundary = false;
// --end - inception

// - data
cv::Mat _image;

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_raw(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_center(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGBL>); // original with no unbending process, to reserve actual data
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGBL>);  // may be done with unbending process, for seg ground easier
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_ground_plane(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGBL>);

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_closest_cluster_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_content_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_wall_1_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_1_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_top_line_2_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_vertex(new pcl::PointCloud<pcl::PointXYZRGBL>);

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_voxel(new pcl::PointCloud<pcl::PointXYZRGBL>);

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_fall_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_available_fall(new pcl::PointCloud<pcl::PointXYZRGBL>);

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_occupied_voxel(new pcl::PointCloud<pcl::PointXYZRGBL>);

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_points_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point_OBB(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point(new pcl::PointCloud<pcl::PointXYZRGBL>);

Pose _pose;
Pose _pose_;
Eigen::Quaternionf q_camera;
Eigen::Quaternionf q_spout;

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_test_frame_(new pcl::PointCloud<pcl::PointXYZRGBL>);
cv::Mat _image_test_frame_;
// - end - data

void _clean()
{
	// to clean data
	_cloud_transformed->points.clear();
	_cloud_downsampled->points.clear();
	_cloud_center->points.clear();
	cloud_downsampled->points.clear();
	cloud_ground_plane->points.clear();
	cloud_cropped->points.clear();
	cloud_filtered->points.clear();
	cloud_closest_cluster->points.clear();
	cloud_edge->points.clear();
	cloud_line->points.clear();

	cloud_line_OBB->points.clear();
	cloud_closest_cluster_OBB->points.clear();
	cloud_content_OBB->points.clear();
	cloud_wall_1_OBB->points.clear();
	cloud_top_line_1_OBB->points.clear();
	cloud_top_line_2_OBB->points.clear();

	cloud_vertex_OBB->points.clear();
	cloud_vertex->points.clear();

	cloud_available_voxel_OBB->points.clear();
	cloud_available_voxel->points.clear();
	cloud_available_fall_OBB->points.clear();
	cloud_available_fall->points.clear();

	cloud_occupied_voxel_OBB->points.clear();
	cloud_occupied_voxel->points.clear();

	cloud_fall_points_OBB->points.clear();
	cloud_fall_point_OBB->points.clear();
	cloud_fall_point->points.clear();
	cloud_dest_point_OBB->points.clear();
	cloud_dest_point->points.clear();

}

// - data flow control
bool image_received = false;
bool cloud_received = false;
bool pose_received = false;
bool is_process_busy = false;
// - data flow control

void data_correction()
{
	int n_points = _cloud_raw->points.size();
	for (int y = 0; y < 240; y ++)
	{
		for (int x = 0; x < 320; x ++)
		{
			int i = y * 320 + x;
			pcl::PointXYZRGBL &_p = _cloud_raw->points[i];

			double distance;
			cartesianTransform_wf.inverse_transformPixel(x, y, _p.z, distance);
			//std::cout << " p.x: " << p.x << " p.y: " << p.y << " p.z: " << p.z << " distance: " << distance << std::endl;

			double _px, _py, _pz;
			cartesianTransform_sf.transformPixel(x, y, distance, _px, _py, _pz);
			//std::cout << " _px: " << _px << " _py: " << _py << " _pz: " << _pz << std::endl;
			_p.x = -_px;
			_p.y = _py;
			_p.z = _pz;
		}
	}
}
void _process()
{
	//std::cout << "==> _process ... \n" << std::endl;
    if (is_process_busy) return;

	is_process_busy = true;
	if (n_frames == 0)
	{
		st_time = std::chrono::steady_clock::now();
	}

	_clean();
	//std::cout << "2 ===>_pose.x: " << _pose.x << " _pose.y: " << _pose.y << " _pose.z: " << _pose.z << " _pose.w: " << _pose.w << std::endl;
	t_sensor = compute_rotation_matrix(_pose);

	t_sensor = t_calibration * t_sensor;
	//sensor_height = (-camera_height) - 1.5; // offset 1.5m is too much?
	sensor_height = (-camera_height) - 1.25;
	//sensor_height = (-camera_height) - 1.0;

	//std::cout << "==> t_sensor: \n" << t_sensor.matrix() << std::endl;
	//std::cout << "   ==> sensor_height: " << sensor_height << " truck_height:" << truck_height << std::endl;

	//std::vector<int> cloud_clean_indices;
	//pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZRGBL>);
	//pcl::removeNaNFromPointCloud(*_cloud_raw, *cloud_clean, cloud_clean_indices);
	//*_cloud_raw = *cloud_clean;

	/////////////////////////
	int n_raw_points_pre_clean = _cloud_raw->points.size();
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZRGBL>);
	for (int i = 0; i < n_raw_points_pre_clean; i ++)
	{
		pcl::PointXYZRGBL p = _cloud_raw->points[i];
		//if (std::isfinite(p.x) || std::isfinite(p.y) || std::isfinite(p.z))
		if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
		{
			cloud_clean->points.push_back(p);
			//continue;
		} //else {
		//}
	}
	int n_clean_points = cloud_clean->points.size();
	cloud_clean->resize(n_clean_points);
	cloud_clean->width = n_clean_points;
	cloud_clean->height = 1;
	cloud_clean->is_dense = false;
	*_cloud_raw = *cloud_clean;
	//////////////////////////

	int n_raw_points = _cloud_raw->points.size();
	//std::cout << "   ==> n_raw_points: " << n_raw_points << std::endl;

	if (n_raw_points > 0)
	{
		//data_correction();

		pcl::transformPointCloud (*_cloud_raw, *_cloud_transformed, t_sensor);

		pcl::PassThrough<pcl::PointXYZRGBL> pass;
		pass.setFilterLimitsNegative (false);
		pass.setInputCloud (_cloud_transformed);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (-sensor_height, -sensor_height + truck_height);
		pass.filter (*cloud_cropped);
	}

	int n_cropped_points = cloud_cropped->points.size();
	//std::cout << "   ==> pre filtering => n_cropped_points: " << n_cropped_points << std::endl;

	vertex_computed = false;
	dest_found = false;
	vertex_computed = false;
	falling_found = false;
	dest_found = false;
	falling_point_not_full = false;

	if (n_cropped_points > 0)
	{
		detect_truck(
			t_sensor,
			_cloud_transformed, //_cloud_raw,
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
			truck_behind,

			sprout_angle_Z,
			sprout_angle_available
		);
	}

	vertex_computed_disp = vertex_computed;

	update_ctrl_signal();

	//std::cout << "=========> data_replay: " << data_replay << std::endl;

	if (! data_replay)
	{
		// pub data for recording
		ros::Time curTime = ros::Time::now();
		cv::Mat img = _image.clone();
		//std::cout << "=========> Publish RGB and point cloud ... " << std::endl;
		publish_image(video_publisher, img, curTime);
		publish_cloud(cloud_publisher, _cloud_raw, curTime);

		geometry_msgs::PoseStamped pose_camera;
		pose_camera.header.frame_id = strFrameID;
		pose_camera.header.stamp = curTime;
		pose_camera.pose.position.x = 0;
		pose_camera.pose.position.y = 0;
		pose_camera.pose.position.z = 0;
		pose_camera.pose.orientation.w = q_camera.w();
		pose_camera.pose.orientation.x = q_camera.x();
		pose_camera.pose.orientation.y = q_camera.y();
		pose_camera.pose.orientation.z = q_camera.z();
		pose_camera_publisher.publish(pose_camera);

		geometry_msgs::PoseStamped pose_spout;
		pose_spout.header.frame_id = strFrameID;
		pose_spout.header.stamp = curTime;
		pose_spout.pose.position.x = 0;
		pose_spout.pose.position.y = 0;
		pose_spout.pose.position.z = 0;
		pose_spout.pose.orientation.w = q_spout.w();
		pose_spout.pose.orientation.x = q_spout.x();
		pose_spout.pose.orientation.y = q_spout.y();
		pose_spout.pose.orientation.z = q_spout.z();
		pose_spout_publisher.publish(pose_spout);
		// - end pub
	}

	n_frames ++;

	image_received = false;
	cloud_received = false;
	pose_received = false;

	is_process_busy = false;

	//error_msg = "CAN Signal Failed";
	//error_msg = "";

	//std::cout << "===> sprout_angle_Z: " << sprout_angle_Z << std::endl;

	//if (vertex_computed && fabs(sprout_angle_Z) < 15.0) //
	if (vertex_computed && (sprout_angle_Z < 15.0 && sprout_angle_Z > -25.0)) //
	{
		// check and adjust integration time if needed

		//if (n_sat_over_flow > 1900)
		//{
			//std::cout << "===> sprout_angle_Z: " << sprout_angle_Z << std::endl;
			int n_cluster_points = cloud_closest_cluster->points.size();

			pcl::PointXYZRGBL point_s, point_e;
			float x_s = 65.0, x_e = -65.0;
			for (int i = 0; i < n_cluster_points; i ++)
			{
				pcl::PointXYZRGBL p = cloud_closest_cluster->points[i];
				if (p.x < x_s)
				{
					x_s = p.x;
					point_s = p;
				}
				if (p.x > x_e)
				{
					x_e = p.x;
					point_e = p;
				}
			}

			uint32_t label_l = point_s.label;
			int x_l = static_cast<uint16_t> ((label_l & 0x00FFF000) >> 12);
			int y_l = static_cast<uint16_t> (label_l & 0x00000FFF);

			uint32_t label_r = point_e.label;
			int x_r = static_cast<uint16_t> ((label_r & 0x00FFF000) >> 12);
			int y_r = static_cast<uint16_t> (label_r & 0x00000FFF);

			int min_x = std::min(x_l, x_r);
			int max_x = std::max(x_l, x_r);
			int min_y = std::min(y_l, y_r);
			int max_y = std::max(y_l, y_r);
			//std::cout << "===> min_x: " << min_x << " max_x: " << max_x << " min_y: " << min_y << " max_y: " << max_y << std::endl;

			int n_sat_over_flow = points_saturation_overflow.size();
			//std::cout << "===> points_valid: " << points_valid.size() << ", points_low_amplitude: " << points_low_amplitude.size() << ", points_saturation_overflow: " << points_saturation_overflow.size() << std::endl;

			int n_sat_overflow_in_cluster = 0;
			for (int i = 0; i < n_sat_over_flow; i ++)
			{
				Point2D p = points_saturation_overflow[i];
				if (p.x > min_x && p.y > min_y && p.x < max_x && p.y < max_y)
				{
					// invlid in cluster
					n_sat_overflow_in_cluster ++;
				}
			}
			//std::cout << "   ===> n_sat_overflow_in_cluster: " << n_sat_overflow_in_cluster << std::endl;
			bool to_adjust_integration_time = false;
			if (n_sat_overflow_in_cluster > max_tolerance_overflow)
			{
				if (int0 > min_integration_time) // min integratoin time, should be 2600
				{
					int0 -= step_integration_time; // step, should be 500
					to_adjust_integration_time = true;
				}
			} else if (n_sat_overflow_in_cluster == 0)
			{
				int n_low_amplitude = points_low_amplitude.size();
				int n_low_amplitude_points_in_cluster = 0;
				for (int i = 0; i < n_low_amplitude; i ++)
				{
					Point2D p = points_low_amplitude[i];
					if (p.x > min_x && p.y > min_y && p.x < max_x && p.y < max_y)
					{
						// invlid in cluster
						n_low_amplitude_points_in_cluster ++;
					}
				}
				//std::cout << "   ===> n_low_amplitude_points_in_cluster: " << n_low_amplitude_points_in_cluster << std::endl;
				if (n_low_amplitude_points_in_cluster > max_tolerance_low_amplitude)
				{
					if (int0 < max_integration_time) // max integratoin time, should be 4000
					{
						int0 += step_integration_time; // step, should be 500
						to_adjust_integration_time = true;
					}
				}
			}

			if (to_adjust_integration_time)
			{
				if (int0 < min_integration_time) int0 = min_integration_time;
				if (int0 > max_integration_time) int0 = max_integration_time;
				setParameters();
			}

			//std::cout << "   ===> current integration time: " << int0 << std::endl;

		//}
	}
}

cv::Mat read_image(std::string img_file_name)
{
	std::string img_file_path = package_path + "/resources/" + img_file_name;
	std::cout << "   ===> img_file_path: " << img_file_path << std::endl;
	cv::Mat image = cv::imread(img_file_path);
	std::cout << "   ===> img_file_path: " << img_file_path << " loaded." << std::endl;
	return image;
}

void start_RGB()
{
	cv::namedWindow("Video", cv::WND_PROP_FULLSCREEN);
	cv::setWindowProperty("Video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	cv::Mat imgDisplay(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));

	//cv::putText(
	//	imgDisplay, "Argriculture Automation", cv::Point(420, 320),
	//	cv::FONT_HERSHEY_COMPLEX_SMALL, 3.5, cv::Scalar(255,255,255), 2, cv::LINE_AA
	//);
	cv::Mat main_logo = read_image("main_logo.png");
	main_logo.copyTo(imgDisplay(cv::Rect(cv::Point(420, 320), main_logo.size())));

	//cv::putText(
	//	imgDisplay, "Powered by:", cv::Point(880, 500),
	//	cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(255,255,255), 1, cv::LINE_AA
	//);
	//cv::line(imgDisplay, cv::Point(880, 520), cv::Point(1100, 520), cv::Scalar(0,255,0), 2, cv::LINE_AA);
	//cv::putText(
	//	imgDisplay, "Suojian", cv::Point(880, 560),
	//	cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(255,255,255), 1, cv::LINE_AA
	//);
	//cv::putText(
	//	imgDisplay, "Visionary Semiconductor Inc.", cv::Point(880, 600),
	//	cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(255,255,255), 1, cv::LINE_AA
	//);
	cv::Mat power_by = read_image("power_by.png");
	power_by.copyTo(imgDisplay(cv::Rect(cv::Point(880, 500), power_by.size())));

	cv::imshow("Video", imgDisplay);
	cv::waitKey(3000);

	cv::Mat video_frame;
    if (video_ok)
    {
        while(ros::ok())
        {
			try
			{
				if (ros_initialized)
				{
					videoCapture->read(video_frame);
					if (! video_frame.empty())
					{
						if (testFrame != 0)
						{
							_image = _image_test_frame_.clone();
						} else
						{
							_image = video_frame.clone();
						}
						//std::cout << "===> _image w: " << _image.cols << " h: " << _image.rows << std::endl;
						try
						{
							display_video();
						} catch (...) {}
						cv::waitKey(1);
					}
				}
			} catch (...) {
				std::cerr << "Something wrong with RGB camera!" << std::endl;
			}
        }
    }
}
int test_count = 10;
bool tof_frame_ready = false;
//pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud_raw_(new pcl::PointCloud<pcl::PointXYZRGBL>);  // this used for process in another thread
void updateFrame(std::shared_ptr<Frame> frame)
{
	//std::cout << "==> updateFrame ... \n" << std::endl;
	//if (is_process_busy || (!startup_complited || data_replay)) return;
	//if ((!startup_complited) || data_replay) return;
	if ((!startup_complited) || is_process_busy || data_replay) return;

    float angle_x = 0;
    float angle_y = 0;
    float angle_z_sensor = 0;
    if (imu_enabled)
    {
        angle_x        = M_PI * imu_sensor->Angle[0] / 180.0;
        angle_y        = M_PI * imu_sensor->Angle[1] / 180.0;
        angle_z_sensor = M_PI * imu_sensor->Angle[2] / 180.0;
    }

    float d_angle_z = 0.0f;

    Eigen::Matrix3f r_imu;
    Eigen::Matrix3f r_imu_d;
    Eigen::Matrix3f r_spout;

	//if (! sprout_angle_available)
	//{
		//sprout_angle_Z = -90.;
		//std::cout << "==> sprout_angle_available: " << sprout_angle_available << " sprout_angle_Z: " << sprout_angle_Z << std::endl;
        r_spout =
              Eigen::AngleAxisf(M_PI * sprout_angle_Z / 180.0, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(0,        Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(0,        Eigen::Vector3f::UnitX());
        Eigen::Affine3f t_spout = Eigen::Affine3f::Identity();
        t_spout.rotate (r_spout);

		q_spout = Eigen::Quaternionf(r_spout);
		auto euler = q_spout.toRotationMatrix().eulerAngles(0, 1, 2);
		//std::cout << "roll: " << 180.0 * euler[0] / 3.14 << " pitch: " << 180.0 * euler[1] / 3.14 << " yaw: " << 180.0 * euler[2] / 3.14 << std::endl;
	//}

    if (imu_enabled)
    {
        r_imu =
              Eigen::AngleAxisf(angle_z_sensor, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(angle_y,        Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(angle_x,        Eigen::Vector3f::UnitX());
        Eigen::Affine3f t_imu = Eigen::Affine3f::Identity();
        t_imu.rotate (r_imu);

        r_imu_d =
                Eigen::AngleAxisf(d_angle_z, Eigen::Vector3f::UnitZ());
        Eigen::Affine3f t_imu_d = Eigen::Affine3f::Identity();
        t_imu_d.rotate (r_imu_d);

        Eigen::Quaternionf q_camera_imu(r_imu);
        Eigen::Quaternionf q_camera_d(r_imu_d);
        q_camera = q_camera_d * q_camera_imu;

		if (testFrame == 1 || testFrame == 2)
		{
			//-0.24112982,0.05933512,-0.28764188,0.92498535
			_pose.x = -0.28209540;
			_pose.y = -0.19534004;
			_pose.z = 0.44940034;
			_pose.w = 0.82480526;
		} else if (testFrame == 3)
		{
			//-0.22069642,0.07216609,-0.38957700,0.89124346
			_pose.x = -0.22069642;
			_pose.y =  0.07216609;
			_pose.z = -0.38957700;
			_pose.w =  0.89124346;
		} else if (testFrame == 4)
		{
			//0.04974532,-0.21211636,0.97530389,0.03625363
			_pose.x =  0.04974532;
			_pose.y = -0.21211636;
			_pose.z =  0.97530389;
			_pose.w =  0.03625363;
		} else if (testFrame == 5)
		{
			//-0.04810283,-0.20761789,0.95686638,0.19745262
			_pose.x = -0.04810283;
			_pose.y = -0.20761789;
			_pose.z =  0.95686638;
			_pose.w =  0.19745262;
		} else if (testFrame == 6)
		{
			//-0.17646305,-0.31822380,0.82703859,0.42848751
			_pose.x = -0.17646305;
			_pose.y = -0.31822380;
			_pose.z =  0.82703859;
			_pose.w =  0.42848751;
		} else if (testFrame == 7)
		{
			//0.06280476,-0.16392930,0.97907549,-0.10292746
			_pose.x =  0.06280476;
			_pose.y = -0.16392930;
			_pose.z =  0.97907549;
			_pose.w = -0.10292746;
		} else if (testFrame == 8)
		{
			//-0.17155331,-0.30843276,0.81098884,0.46662179
			_pose.x = -0.17155331;
			_pose.y = -0.30843276;
			_pose.z =  0.81098884;
			_pose.w =  0.46662179;
		}else if (testFrame == 9)
		{
			//-0.19918735,-0.32114208,0.78965402,0.48336184
			_pose.x = -0.19918735;
			_pose.y = -0.32114208;
			_pose.z =  0.78965402;
			_pose.w =  0.48336184;
		} else
		{
			_pose.x = q_camera.x();
			_pose.y = q_camera.y();
			_pose.z = q_camera.z();
			_pose.w = q_camera.w();
		}
    }

    int x, y, k, l;

    const size_t nPixel = frame->width * frame->height;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
    cloud->points.resize(nPixel);

    uint16_t distance;//, amplitude;
    double px, py, pz;
    //double max_X = 0.0, z_max_X = 0.0;
    //double min_X = 1000.0, z_min_X = 0.0;

    Color c;
    int rgb;
    uint8_t r;
    uint8_t g;
    uint8_t b;

	points_valid.clear();
	points_low_amplitude.clear();
	points_saturation_overflow.clear();

    for(k=0, l=0, y=0; y< frame->height; y++){
        for(x=0; x< frame->width; x++, k++, l+=2){
            pcl::PointXYZRGBL &p = cloud->points[k];
            distance = (frame->distData[l+1] << 8) + frame->distData[l];

            c = imageColorizer->getColor(distance);

            r = c.r;
            g = c.g;
            b = c.b;

			Point2D p2d;
			p2d.x = x;
			p2d.y = y;

			if (distance > 0 && distance < 64000)
			{
				cartesianTransform.transformPixel(x, y, distance, px, py, pz);
				p.x = -static_cast<float>(px / 1000.0); //mm -> m
				p.y = static_cast<float>(py / 1000.0);
				p.z = static_cast<float>(pz / 1000.0);

				p.r = r;
				p.g = g;
				p.b = b;

				points_valid.push_back(p2d);
            }else{
                if (distance == 64001)
                {
                    points_low_amplitude.push_back(p2d);
                } else if (distance == 64002)
                {
                    points_saturation_overflow.push_back(p2d);
                } else if (distance == 64003)
                {
                    points_saturation_overflow.push_back(p2d);
                }

                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            }
            uint32_t label = ((uint32_t)x << 12 | (uint32_t)y);
            p.label = label;
        }
    }
	//std::cout << "   ===> points_valid: " << points_valid.size() << ", points_low_amplitude: " << points_low_amplitude.size() << ", points_saturation_overflow: " << points_saturation_overflow.size() << std::endl;

    //if ((! is_process_busy) && imu_enabled)
    if (imu_enabled)
    {
		if (testFrame != 0)
		{
			//*_cloud_raw_ = *_cloud_test_frame_; // this used for process in another thread
			*_cloud_raw = *_cloud_test_frame_;

			// test a few times only
			//test_count --;
			//if (test_count <= 0)
			//{
			//	_cloud_test_frame_->points.clear();
			//} else
			//{
			//	std::cout << "   ===> test_count: " << test_count << std::endl;
			//}

		} else
		{
			//*_cloud_raw_ = *cloud; // this used for process in another thread
			*_cloud_raw = *cloud;
		}

		_process();

		tof_frame_ready = true;
	}
}
/*
void sceneProcess()
{
	while(ros::ok())
	{
		if (ros_initialized)
		{
			if (tof_frame_ready)
			{
				tof_frame_ready = false;
				*_cloud_raw = *_cloud_raw_;

				_process();
			} else
			{
				usleep(1);
			}
		}
	}
}
*/
void start_imu_0()
{
	imu_0->start();
}

void _pause_camera(bool mode)
{
	if (data_replay) return;

	//if (mode == mode_manual) return;

    mode_manual = mode;
	std::cout << "Mode set to => " << mode_manual << std::endl;

    if (mode_manual)
    {
        if (tof_started) stopStreaming();
    } else
    {
        if (! tof_started) startStreaming();
    }
}

void read_test_data()
{
	std::string pcd_file = package_path + "/data/pcd/" + std::to_string(testFrame) + ".pcd"; //00644 01787 01840
	std::cout << "===> test_frame pcd_file: " << pcd_file << std::endl;
	std::string image_file = package_path + "/data/images/" + std::to_string(testFrame) + ".jpg"; //00644 01787 01840
	std::cout << "===> test_frame image_file: " << image_file << std::endl;

	pcl::io::loadPCDFile<pcl::PointXYZRGBL> (pcd_file, *_cloud_test_frame_);
	_image_test_frame_ = cv::imread(image_file);
    std::cout << "===> _cloud_test_frame_: " << _cloud_test_frame_->points.size() << std::endl;
}

void display_video()
{
	//std::cout << "   => vertex_computed: " << vertex_computed << std::endl;
	int min_2d_x = 319;
	int min_2d_y = 239;
	int max_2d_x = 0;
	int max_2d_y = 0;

	try
	{
		if(ros_initialized)
		{
			cv::Mat imgDisplay(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
			cv::Mat img = _image.clone();
			//std::cout << "1===> img.cols: " << img.cols << std::endl;
			//std::cout << "1===> img.rows: " << img.rows << std::endl;
			float cr = 1080.0 / ((float)img.rows);
			//std::cout << " ===> cr: " << cr << std::endl;
			cv::resize(img, img, cv::Size(), cr, cr); // then becomes 1440 x 1080
			//std::cout << "2===> img.cols: " << img.cols << std::endl;
			//std::cout << "2===> img.rows: " << img.rows << std::endl;

			if (! mode_manual)
			{
				if (vertex_computed_disp)
				{

					Eigen::Affine3d t_sensor_inverse = t_sensor.inverse();

					pcl::PointXYZRGBL p_fall = cloud_fall_point->points[0];
					pcl::PointXYZRGBL p_dest = cloud_dest_point->points[0];
					pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBL>);
					cloud_tmp->points.push_back(p_fall);
					cloud_tmp->points.push_back(p_dest);
					pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_tmp_inverse(new pcl::PointCloud<pcl::PointXYZRGBL>);
					pcl::transformPointCloud (*cloud_tmp, *cloud_tmp_inverse, t_sensor_inverse);
					pcl::PointXYZRGBL p_fall_inverse = cloud_tmp_inverse->points[0];
					pcl::PointXYZRGBL p_dest_inverse = cloud_tmp_inverse->points[1];

					cv::Point p2d_fall, p2d_dest;

					p2d_fall.x = 4.5 * (p_fall_inverse.x * 181.296 / p_fall_inverse.z) + 720;
					p2d_fall.y = 4.5 * (p_fall_inverse.y * 181.079 / p_fall_inverse.z) + 540;
					p2d_dest.x = 4.5 * (p_dest_inverse.x * 181.296 / p_dest_inverse.z) + 720;
					p2d_dest.y = 4.5 * (p_dest_inverse.y * 181.079 / p_dest_inverse.z) + 540;

					if (p2d_fall.x < 60) p2d_fall.x = 60;
					if (p2d_fall.x > 1380) p2d_fall.x = 1380;

					if (p2d_dest.x < 60) p2d_dest.x = 60;
					if (p2d_dest.x > 1380) p2d_dest.x = 1380;

					if (p2d_fall.y < 110) p2d_fall.y = 110;
					if (p2d_fall.y > 1020) p2d_fall.x = 1020;

					if (p2d_dest.y < 110) p2d_dest.y = 110;
					if (p2d_dest.y > 1020) p2d_dest.y = 1020;

					cv::drawMarker(img, cv::Point(p2d_fall.x, p2d_fall.y - 50), cv::Scalar(0, 0, 255), cv::MARKER_TRIANGLE_DOWN, 100, 8, cv::LINE_AA);
					cv::drawMarker(img, cv::Point(p2d_dest.x, p2d_dest.y - 50), cv::Scalar(0, 255, 0), cv::MARKER_TRIANGLE_DOWN, 100, 8, cv::LINE_AA);

					cv::drawMarker(imgDisplay, cv::Point(1650, 160), cv::Scalar(0, 255, 0), cv::MARKER_TRIANGLE_DOWN, 60, 8, cv::LINE_AA);

				} else
				{
					//cv::putText(
					//	imgDisplay, "No Truck Detected!", cv::Point(1500, 168),
					//	cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
					//);
				}

				if (display_ctrl_available && ctrl_signal_life_count > 0)
				{

					if (display_ctrl_available && ctrl_signal_life_count > 0)
					{
						if (ctrl_current_ctrl_type == 0)
						{
							if (display_ctr_p1.x < display_ctr_p2.x)
							{
								cv::arrowedLine(imgDisplay, cv::Point(1600, 360), cv::Point(1700, 360), cv::Scalar(0, 0, 255), 12, cv::LINE_AA, 0, 0.4);
							} else
							{
								cv::arrowedLine(imgDisplay, cv::Point(1700, 360), cv::Point(1600, 360), cv::Scalar(0, 0, 255), 12, cv::LINE_AA, 0, 0.4);
							}
						} else if (ctrl_current_ctrl_type == 1)
						{
							if (display_ctr_p1.y > display_ctr_p2.y)
							{
								cv::arrowedLine(imgDisplay, cv::Point(1650, 310), cv::Point(1650, 410), cv::Scalar(0, 0, 255), 12, cv::LINE_AA, 0, 0.4);
							} else
							{
								cv::arrowedLine(imgDisplay, cv::Point(1650, 410), cv::Point(1650, 310), cv::Scalar(0, 0, 255), 12, cv::LINE_AA, 0, 0.4);
							}
						}
					}

				}
			}

			cv::putText(
				imgDisplay, "Truck Detected:", cv::Point(1500, 60),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.75, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);
			cv::line(imgDisplay, cv::Point(1500, 76), cv::Point(1860, 76), cv::Scalar(255,255,255), 2, cv::LINE_AA);

			cv::putText(
				imgDisplay, "Command:", cv::Point(1500, 260),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.75, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);
			cv::line(imgDisplay, cv::Point(1500, 276), cv::Point(1860, 276), cv::Scalar(255,255,255), 2, cv::LINE_AA);

			cv::putText(
				imgDisplay, "Truck Located:", cv::Point(1500, 460),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.75, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);
			cv::line(imgDisplay, cv::Point(1500, 476), cv::Point(1860, 476), cv::Scalar(255,255,255), 2, cv::LINE_AA);

			if (truck_behind)
			{
				cv::putText(
					imgDisplay, "Behind", cv::Point(1500, 560),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
				);
			} else
			{
				cv::putText(
					imgDisplay, "At Side", cv::Point(1500, 560),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,255,0), 1, cv::LINE_AA
				);
			}

			cv::putText(
				imgDisplay, "Mode:", cv::Point(1500, 660),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.75, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);
			cv::line(imgDisplay, cv::Point(1500, 676), cv::Point(1860, 676), cv::Scalar(255,255,255), 2, cv::LINE_AA);

			if (mode_manual)
			{
				cv::putText(
					imgDisplay, "Manual", cv::Point(1500, 720),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
				);
			} else 
			{
				cv::putText(
					imgDisplay, "Automatic", cv::Point(1500, 720),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,255,0), 1, cv::LINE_AA
				);
			}

			if (! comm_ctrl_enabled)
			{
				cv::putText(
					imgDisplay, "Ctrl Disconnected", cv::Point(1500, 760),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
				);
			} else 
			{
				cv::putText(
					imgDisplay, "Ctrl Connected", cv::Point(1500, 760),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,255,0), 1, cv::LINE_AA
				);
			} 

			if (distance_ctrl)
			{
				cv::putText(
					imgDisplay, "Distance Ctrl ON", cv::Point(1500, 800),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,255,0), 1, cv::LINE_AA
				);
			} else
			{
				cv::putText(
					imgDisplay, "Distance Ctrl OFF", cv::Point(1500, 800),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
				);
			}

			if (balanced_fill)
			{
				cv::putText(
					imgDisplay, "Balanced Fill", cv::Point(1500, 840),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,255,0), 1, cv::LINE_AA
				);
			} else
			{
				cv::putText(
					imgDisplay, "Centered Fill", cv::Point(1500, 840),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
				);
			}

			if (! error_msg.empty())
			{
				cv::putText(
					imgDisplay, error_msg, cv::Point(1500, 900),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 1, cv::LINE_AA
				);
			}

			cv::Mat overlay(imgDisplay, cv::Rect(0, 0, 1440, 1080));
			img.copyTo(overlay);

			cv::putText(
				imgDisplay, "Powered by:", cv::Point(1500, 960),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);
			cv::line(imgDisplay, cv::Point(1500, 978), cv::Point(1860, 978), cv::Scalar(255,255,255), 1, cv::LINE_AA);
			cv::putText(
				imgDisplay, "Suojian", cv::Point(1500, 1010),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);
			cv::putText(
				imgDisplay, "Visionary Semiconductor", cv::Point(1500, 1040),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 1.1, cv::Scalar(255,255,255), 1, cv::LINE_AA
			);

			cv::imshow("Video", imgDisplay);
		}
	} catch (...)
	{
		std::cerr << "GUI failed ..." << std::endl;
	}
}

void cloud_extract(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	if (is_process_busy) return;

	pcl::PCLPointCloud2 pcl_point_cloud2;
	pcl_conversions::toPCL(*cloud_msg, pcl_point_cloud2);

	pcl::fromPCLPointCloud2( pcl_point_cloud2, *_cloud_raw);
	_cloud_raw->resize(_cloud_raw->points.size());
	_cloud_raw->width = _cloud_raw->points.size();
	_cloud_raw->height = 1;
	_cloud_raw->is_dense = false;

	cloud_received = true;

	if (cloud_received && pose_received)
	{
		_process();
	}
}
void pose_extract(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (is_process_busy) return;

	geometry_msgs::Pose p = msg->pose;
	geometry_msgs::Quaternion orientation = p.orientation;
	double x = orientation.x;
	double y = orientation.y;
	double z = orientation.z;
	double w = orientation.w;
	_pose.x = x;
	_pose.y = y;
	_pose.z = z;
	_pose.w = w;

	pose_received = true;
	if (cloud_received && pose_received)
	{
		_process();
	}
}
void video_extract(const sensor_msgs::Image::ConstPtr& msg)
{
	cv::Mat mat(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
	if ((!mat.empty()) && mat.cols > 0 && mat.rows > 0)
	{
		_image = mat.clone();
		image_received = true;

		if (ros_initialized)
		{
			display_video();
			cv::waitKey(1);
		}
	}
}

ros::Subscriber sub_cloud;
ros::Subscriber sub_video;
ros::Subscriber sub_pose;

int8_t encode(uint8_t from) {
    // implicit conversion from unsigned to signed
    return from;
}

uint16_t combine(uint8_t a, uint8_t b) {
    return ((uint16_t)a << 8) + b;
}

void explode( uint16_t from, int8_t &to1, int8_t &to2 ) {
    to2 = from;
    to1 = (from >> 8);
}
int main(int argc, char **argv)
{
	cartesianTransform_wf.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, 0); // 0 WF, 1 SF
	cartesianTransform_sf.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, 1); // 0 WF, 1 SF

    imageColorizer = new ImageColorizer();
    imageColorizer->setRange(0, 15000);

    std::cerr << "Starting ROS ... " << std::endl;

    ros::init(argc, argv, "harv_node");

    std::cerr << "Initialising ... " << std::endl;

    initialise();

	if (data_replay)
	{
		cv::namedWindow("Video", cv::WND_PROP_FULLSCREEN);
		cv::setWindowProperty("Video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	} else
	{
		videoCapture = new cv::VideoCapture();
		std::cerr << "Starting RGB camera: " << video_camera << std::endl;
		try {
			video_ok = videoCapture->open(video_camera);
			if (video_ok)
			{
				videoCapture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
				videoCapture->set(cv::CAP_PROP_FRAME_WIDTH, 640);
				videoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, 480);

				std::cerr << "RGB camera started." << std::endl;

				std::thread th_start_RGB(&start_RGB);
				th_start_RGB.detach();
			} else
			{
				std::cerr << "Can't initialize RGB camera." << std::endl;
				return 1;
			}
		} catch (...)
		{
			std::cerr << "Exception: can't initialize RGB camera." << std::endl;
			return 1;
		}
	}

    ros_initialized = true;

	package_path = ros::package::getPath("vsemi_tof_harv");
	std::cout << "package_path: " << package_path << std::endl;
	pose_file_path = package_path + "/camera/pose.dat";

	Eigen::Affine3d t;
	Eigen::Matrix4d _p_4d;
	read_camera_pose(pose_file_path.c_str(), _p_4d);
	t_calibration.matrix().block<3, 3>(0, 0) = _p_4d.block<3, 3>(0, 0);
	std::cout << "Camera pose (pre-calibrated): \n" << t_calibration.matrix() << std::endl;
	camera_height = _p_4d.coeff(2, 3);
	std::cout << "Camera height: " << camera_height << std::endl;

    ros::NodeHandle nh("~");

	if (! data_replay)
	{
		std::cerr << "Starting IMU ... " << std::endl;

		std::cerr << "Connecting to IMU: " << imu_port << std::endl;

		// transform to world coordinates
		Eigen::Matrix3f m;
		m <<
		1, 0, 0,
		0, 0, 1,
		0, -1, 0;
		t_sensor_2_world.rotate (m);

		imu_0 = new IMU(imu_port.c_str(), 9600, 8, 'N', 1);

		std::thread th_start_imu_0(&start_imu_0);
		th_start_imu_0.detach();

		std::cerr << "IMU started." << std::endl;
	}

    imu_sensor = imu_0;
    imu_enabled = true;

	if (comm_ctrl_type == 1)
	{	
		nh.getParam("serial_port", serial_port_name);

		std::cerr << "Open Serial port: " << serial_port_name << std::endl;

		if (serial.openConnection(serial_port_name))
		{
			comm_ctrl_enabled = true;
			std::cerr << "Serial port: " << serial_port_name << " connected." << std::endl;
		} else
		{
			std::cout << "Cannot open serial socket, serial signal will be disabled!" << std::endl;
		}
	} 
	else if (comm_ctrl_type == 2)
	{
		nh.getParam("socket_can", socket_can_name);

		std::cerr << "Connecting to CAN port: " << socket_can_name << std::endl;

		if (socket_can.open(socket_can_name) == scpp::STATUS_OK)
		{
			comm_ctrl_enabled = true;
			std::cerr << "CAN port: " << socket_can_name << " connected." << std::endl;
		} else
		{
			std::cout << "Cannot open CAN socket, CAN signal will be disabled!" << std::endl;
		}
	}
	std::cout << "Ctrl Comm hardware type:    " << comm_ctrl_type << std::endl;
	std::cout << "Ctrl Comm hardware enabled: " << comm_ctrl_enabled << std::endl;

	std::thread th_ctrl_rcv(&ctrl_rcv);
	th_ctrl_rcv.detach();

	std::thread th_ctrl_send(&ctrl_send);
	th_ctrl_send.detach();

	nh.getParam("test_frame", testFrame);
	if (testFrame != 0)
	{
		std::cerr << "===> testFrame: " << testFrame << std::endl;
		read_test_data();
	}

    st_time = std::chrono::steady_clock::now();
	if (! data_replay)
	{
		setParameters();
		startStreaming();
		if (testFrame != 0)
		{
			mode_manual = false;
		} else
		{
			_pause_camera(true);
		}
	}

	startup_complited = true;

	if (data_replay)
	{
		mode_manual = false;

		//sub_cloud  = nh.subscribe ("/vsemi_c320_camera/camera/cloud", 1, cloud_extract);
		//sub_video  = nh.subscribe ("/vsemi_c320_camera/camera/video", 1, video_extract);
		//sub_pose   = nh.subscribe ("/vsemi_c320_camera/camera/camera", 1, pose_extract);

		sub_cloud  = nh.subscribe ("/vsemi_tof_harv/camera/cloud",  1, cloud_extract);
		sub_video  = nh.subscribe ("/vsemi_tof_harv/camera/video",  1, video_extract);
		sub_pose   = nh.subscribe ("/vsemi_tof_harv/camera/camera", 1, pose_extract);

		std::cerr << "Subscribed to data_replay. " << std::endl;
	}

	//std::thread th_sceneProcess(&sceneProcess);
	//th_sceneProcess.detach();

    ros::spin();

    ros_initialized = false;

    en_time = std::chrono::steady_clock::now();
    interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
    frame_rate = ((double) n_frames) / interval;
    std::cout << "Frames: " << n_frames << ", time spent: " << interval << ", frame rate: " << frame_rate << std::endl;

	std::cerr << "Shutdown TOF ... " << std::endl;

    interface->stopStream();
	std::cerr << "Shutdown RGB camera ... " << std::endl;
    videoCapture->release();

	std::cerr << "Shutdown IMU ... " << std::endl;
    if (imu_enabled)
    {
        imu_0->stop();
    }

	usleep(2000000);
	std::cerr << "Shutdown ROS ... " << std::endl;

    delete imu_0;
    delete videoCapture;
    delete interface;

    ros::shutdown();
}
