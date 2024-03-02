#include <thread>
#include <ros/ros.h>
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
#include "cartesian_transform.hpp"
#include "interface.hpp"

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/don.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "color.hpp"

#include "imu.h"

#include <vsemi_c320_camera/vsemi_c320_cameraConfig.h>

using namespace espros;

//////////////////////////////////////////////////////////////////////////////////////////////////
void initD3F(double* d3X, double* d3Y, double* d3Z, int width, int height, double angle_x, double angle_y)
{
	int l = width * height;

	double THETA_H = M_PI * angle_x / 180.0f;
	double ALPHA_H = (M_PI - THETA_H) / 2;

	double THETA_V = M_PI * angle_y / 180.0f;
	double ALPHA_V = 2 * M_PI - (THETA_V / 2);

	for (int i = 0; i < l; i ++)
	{
		int x = i % width;
		int y = i / width;

		d3Z[i] = fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height)));

		d3X[i] = (fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height))) / tan(ALPHA_H + (double)x * (THETA_H / width)));

		d3Y[i] = -1.0 * fabs(fabs(0.001 * sin(ALPHA_H + (double)x * (THETA_H / width))) * cos(ALPHA_V + (double)y * (THETA_V / height))) * tan(ALPHA_V + (double)y * (THETA_V / height));
	}
}

void cal3DXYZ(int i, int width, int height, unsigned int val, double* d3X, double* d3Y, double* d3Z, float &X, float &Y, float &Z)
{
	Z = static_cast<double>(val) * d3Z[i];

	X = static_cast<double>(val) * d3X[i];

	Y = static_cast<double>(val) * d3Y[i];
}

double* d3X;
double* d3Y;
double* d3Z;

float X, Y, Z;
/////////////////////////////////////////////////////////////////////////////////////////////////////

int imageType; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int lensType;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
bool medianFilter;
bool averageFilter;
double temporalFilterFactor;
int temporalFilterThreshold;
int edgeThreshold;
int temporalEdgeThresholdLow;
int temporalEdgeThresholdHigh;
int interferenceDetectionLimit;
bool startStream;
bool useLastValue;
bool publishPointCloud;
bool cartesian;
int channel;
int frequencyModulation;
int int0 = 1500, int1, int2, intGr;
int hdr_mode;
int offset, minAmplitude;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;
int old_lensCenterOffsetX = 0;
int old_lensCenterOffsetY = 0;

int roi_leftX = 0;
int roi_topY = 0;
int roi_rightX = 319;
int roi_bottomY = 239;

const int width   = 320;
const int width2  = 160;
const int height  = 240;
const int height2 = 120;
const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

bool align = false;

std::string video_camera = "";
std::string imu_port = "";

uint32_t frameSeq;
boost::signals2::connection connectionFrames;
boost::signals2::connection connectionCameraInfo;

ros::Publisher video_publisher;
ros::Publisher amplitude_publisher;
ros::Publisher depth_raw_Publisher;
ros::Publisher depth_bgr_Publisher;
ros::Publisher cloud_publisher;
ros::Publisher cloud_raw_publisher;
ros::Publisher pose_camera_publisher;

sensor_msgs::CameraInfo cameraInfo;

bool video_ok = false;

ImageColorizer *imageColorizer;

static std::string strFrameID = "sensor_frame";

CartesianTransform cartesianTransform;

Interface* interface;
cv::VideoCapture* videoCapture;

IMU* imu_0;
//IMU* imu_1;
IMU* imu_sensor;
//IMU* imu_vehicle;

Eigen::Affine3f t_sensor_2_world = Eigen::Affine3f::Identity();

std::chrono::steady_clock::time_point st_time;
std::chrono::steady_clock::time_point en_time;
double interval, frame_rate;
int n_frames = 0;

//=======================================================================
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

void setParameters()
{
    std::cerr << "Set parameters ... " << std::endl;
    interface->stopStream();
    //interface->setOffset(offset);
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
        //ROS_INFO("   ---> initLensTransform done");
    }

    ROS_INFO("set parameters...");
    ROS_DEBUG("lens_type %d", lensType);
    ROS_DEBUG("lens_center_offset_x %d", lensCenterOffsetX);
    ROS_DEBUG("lens_center_offset_y %d", lensCenterOffsetY);
    ROS_DEBUG("image_type %d", imageType);
    ROS_DEBUG("start_stream %d", startStream);
    ROS_DEBUG("integration_time0 %d", int0);
    ROS_DEBUG("integration_time1 %d", int1);
    ROS_DEBUG("integration_time_gray %d", intGr);
    ROS_DEBUG("min_amplitude %d", minAmplitude);
    //ROS_DEBUG("offset %d", offset);
    ROS_DEBUG("frequency_modulation %d", frequencyModulation);
    ROS_DEBUG("channel %d ", channel);
    ROS_DEBUG("median_filter %d ", medianFilter);
    ROS_DEBUG("average_filter %d", averageFilter);
    ROS_DEBUG("temporal_filter_factor %f", temporalFilterFactor);
    ROS_DEBUG("temporal_filter_threshold %d ", temporalFilterThreshold);
    ROS_DEBUG("edge_filter_threshold %d", edgeThreshold);
    //ROS_DEBUG("temporal_edge_threshold_low %d ", temporalEdgeThresholdLow);
    //ROS_DEBUG("temporal_edge_threshold_high %d", temporalEdgeThresholdHigh);
    ROS_DEBUG("interference_detection_limit %d ", interferenceDetectionLimit);
    ROS_DEBUG("use_last_value %d", useLastValue);
    ROS_DEBUG("cartesian %d", cartesian);
    ROS_DEBUG("publish_point_cloud %d", publishPointCloud);
    std::cerr << "Set parameters .completed. " << std::endl;
}

void updateConfig(vsemi_c320_camera::vsemi_c320_cameraConfig &config, uint32_t level)
{
    std::cerr << "Updating config ... " << std::endl;
    lensType   = config.lens_type;
    imageType = config.image_type;

    frequencyModulation = config.frequency_modulation;
    
    minAmplitude = config.min_amplitude;
    int0 = config.integration_time_tof_1;
    
    //medianFilter = config.median_filter;
    //averageFilter = config.average_filter;

    align = config.align;
    
    setParameters();

    st_time = std::chrono::steady_clock::now();
    n_frames = 0;
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
    std::cerr << "Streaming started." << std::endl;
}

void updateCameraInfo(std::shared_ptr<CameraInfo> ci)
{
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
}
cv::Mat* video_frame;
bool rgb_ready = false;
void start_RGB()
{
	video_frame = new cv::Mat();
    if (video_ok) 
    {
        while(ros::ok())
	{
		//rgb_ready = false;
		//if (!rgb_ready) {
			//videoCapture >> video_frame;
			videoCapture->read(*video_frame);
			rgb_ready = true;
			if (align)
			{
			    //cv::Scalar color(255, 0, 0);
			    //cv::line(video_frame, cv::Point(0, 0.5 * video_frame.rows), cv::Point(video_frame.cols, 0.5 * video_frame.rows), color, 2.0, cv::LINE_8);
			    //cv::line(video_frame, cv::Point(0.5 * video_frame.cols, 0), cv::Point(0.5 * video_frame.cols, video_frame.rows), color, 2.0, cv::LINE_8);
			}
		//} else {
		//	usleep(1000);
		//}
		//publish_image(video_publisher, video_frame, curTime);
	}
    }
}

std::chrono::steady_clock::time_point prev_time;
std::chrono::steady_clock::time_point current_time;
double ctr_interval;
double ctr_frame_rate = 10.0;
void updateFrame(std::shared_ptr<Frame> frame)
{
		current_time = std::chrono::steady_clock::now();
		ctr_interval = ((double)std::chrono::duration_cast<std::chrono::microseconds>(current_time - prev_time).count()) / 1000000.0;
		if (ctr_interval < (1.0 / ctr_frame_rate))
		{
			return;
		}
		prev_time = std::chrono::steady_clock::now();
    //ROS_INFO("updateFrame ...");
    float angle_x = M_PI * imu_sensor->Angle[0] / 180.0;
    float angle_y = M_PI * imu_sensor->Angle[1] / 180.0;
    float angle_z_sensor = M_PI * imu_sensor->Angle[2] / 180.0;
    //imu_vehicle
    //float angle_z_vehicle = M_PI * imu_vehicle->Angle[2] / 180.0;
    //float d_angle_z = angle_z_sensor - angle_z_sensor;
    float d_angle_z = 0.0f;
    //std::cout << "imu_sensor -> angle_x: " << imu_sensor->Angle[0] << " angle_y: " << imu_sensor->Angle[1] << " angle_z: " << imu_sensor->Angle[2] << std::endl;
    //std::cout << "imu_vehicle -> angle_x: " << imu_vehicle->Angle[0] << " angle_y: " << imuimu_vehicle_1->Angle[1] << " angle_z: " << imu_vehicle->Angle[2] << std::endl;
    Eigen::Matrix3f r_imu;
    r_imu = 
          Eigen::AngleAxisf(angle_z_sensor, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(angle_y,        Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(angle_x,        Eigen::Vector3f::UnitX());
    Eigen::Affine3f t_imu = Eigen::Affine3f::Identity();
    t_imu.rotate (r_imu);
    
    Eigen::Matrix3f r_imu_d;
    r_imu_d = 
            Eigen::AngleAxisf(d_angle_z, Eigen::Vector3f::UnitZ());
    Eigen::Affine3f t_imu_d = Eigen::Affine3f::Identity();
    t_imu_d.rotate (r_imu_d);

    Eigen::Affine3f t_imu_related = t_imu_d * t_imu;

    int x, y, k, l;
    ros::Time curTime = ros::Time::now();

    const size_t nPixel = frame->width * frame->height;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
    cloud->header.frame_id = strFrameID;
    cloud->header.stamp = pcl_conversions::toPCL(curTime);
    cloud->points.resize(nPixel);
    //std::cout << " nPixel: " << nPixel << std::endl;

    uint16_t distance, amplitude;
    double px, py, pz;
    double max_X = 0.0, z_max_X = 0.0;
    double min_X = 1000.0, z_min_X = 0.0;

    Color c;
    int rgb;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    //ROS_INFO("Compose frame ...");
    cv::Mat mat_depth_bgr(frame->height, frame->width, CV_8UC3, cv::Scalar(0, 0, 0));
    //cv::Mat mat_depth_raw(frame->height, frame->width, CV_64F, 0.0);
    //std_msgs::Float64MultiArray float64MultiArray;
    //cv::Mat mat_amplitude(frame->height, frame->width, CV_32F, 0.0);
    //float amplitude_min = 100000.0, amplitude_max = -100000.0;
    int i_data = 0;
    for(k=0, l=0, y=0; y< frame->height; y++){
        for(x=0; x< frame->width; x++, k++, l+=2){
            pcl::PointXYZRGBL &p = cloud->points[k];
            distance = (frame->distData[l+1] << 8) + frame->distData[l];

            //mat_depth_raw.at<double>(y,x) = 0.001 * distance;
            //float64MultiArray.data.push_back(0.001 * distance);

            c = imageColorizer->getColor(distance);

            r = c.r;
            g = c.g;
            b = c.b;

            cv::Vec3b &p2d = mat_depth_bgr.at<cv::Vec3b>(y,x);
            p2d[0] = b;
            p2d[1] = g;
            p2d[2] = r;

            if (distance > 0 && distance < 65000){

                //if (distance > 64000) std::cout << " ---distance: " << distance << std::endl;

                if(cartesian){
                    cartesianTransform.transformPixel(x, y, distance, px, py, pz);
                    p.x = -static_cast<float>(px / 1000.0); //mm -> m
                    p.y = static_cast<float>(py / 1000.0);
                    p.z = static_cast<float>(pz / 1000.0);

                    p.r = r;
                    p.g = g;
                    p.b = b;

                }else{
                    p.x = x / 100.0;
                    p.y = y / 100.0;
                    p.z = distance / 1000.0;
                }

/*
				cal3DXYZ(i_data, 320, 240, distance, d3X, d3Y, d3Z, X, Y, Z);
				pcl::PointXYZRGB _p;
				p.x = X;
				p.y = -Y;
				p.z = Z;
				p.r = r;
				p.g = g;
				p.b = b;
*/
            }else{
                
                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            }
            uint32_t label = ((uint32_t)x << 12 | (uint32_t)y);
            p.label = label;

            i_data ++;
        }
    }

    //float d_amplitude = amplitude_max - amplitude_min;
    //std::cout << " amplitude_min: " << amplitude_min << " amplitude_max: " << amplitude_max << " d_amplitude: " << d_amplitude << std::endl;
    //if (d_amplitude != 0) 
    //{
    //    mat_amplitude = 255 * mat_amplitude / d_amplitude;
    //}
    //mat_amplitude.convertTo(mat_amplitude, CV_8U);
    //cv::cvtColor(mat_amplitude,  mat_amplitude,  cv::COLOR_GRAY2BGR);
    //cv::Mat mat_amplitude_flipped; 
    //cv::flip(mat_amplitude, mat_amplitude_flipped, 1);

    //cv::Mat mat_depth_bgr_flipped; 
    //cv::flip(mat_depth_bgr, mat_depth_bgr_flipped, 1);
    /*
    if (video_ok) 
    {
        cv::Mat video_frame;
        //videoCapture >> video_frame;
        videoCapture->read(video_frame);
        if (align)
        {
            cv::Scalar color(255, 0, 0);
            cv::line(video_frame, cv::Point(0, 0.5 * video_frame.rows), cv::Point(video_frame.cols, 0.5 * video_frame.rows), color, 2.0, cv::LINE_8);
            cv::line(video_frame, cv::Point(0.5 * video_frame.cols, 0), cv::Point(0.5 * video_frame.cols, video_frame.rows), color, 2.0, cv::LINE_8);
        }
        publish_image(video_publisher, video_frame, curTime);
    }*/
	if (rgb_ready) {
        if (align)
        {
            cv::Scalar color(255, 0, 0);
            cv::line(*video_frame, cv::Point(0, 0.5 * video_frame->rows), cv::Point(video_frame->cols, 0.5 * video_frame->rows), color, 2.0, cv::LINE_8);
            cv::line(*video_frame, cv::Point(0.5 * video_frame->cols, 0), cv::Point(0.5 * video_frame->cols, video_frame->rows), color, 2.0, cv::LINE_8);
        }
		publish_image(video_publisher, *video_frame, curTime);
		rgb_ready = false;
	}

    //pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGBL>());
    //pcl::transformPointCloud (*cloud, *cloud_transformed, t_imu_related * t_sensor_2_world);
    //ROS_INFO("Tranform done ...");
    Eigen::Quaternionf q_camera_imu(r_imu);
    Eigen::Quaternionf q_camera_d(r_imu_d);
    Eigen::Quaternionf q_camera = q_camera_d * q_camera_imu;
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

    //float64MultiArray.data = *mat_depth_raw.data;
    //ROS_INFO("Publish frame ...");

    if (align)
    {
		cv::Scalar color(255, 0, 0);
		cv::line(mat_depth_bgr, cv::Point(0, 0.5 * mat_depth_bgr.rows), cv::Point(mat_depth_bgr.cols, 0.5 * mat_depth_bgr.rows), color, 2.0, cv::LINE_8);
		cv::line(mat_depth_bgr, cv::Point(0.5 * mat_depth_bgr.cols, 0), cv::Point(0.5 * mat_depth_bgr.cols, mat_depth_bgr.rows), color, 2.0, cv::LINE_8);
    }
/*
			pcl::PointXYZRGBL min_XYZ, max_XYZ;
			pcl::getMinMax3D(*cloud, min_XYZ, max_XYZ);
            float dx = max_XYZ.x - min_XYZ.x;
            float dy = max_XYZ.y - min_XYZ.y;
            float dz = max_XYZ.z - min_XYZ.z;
            std::cout << " max_XYZ.x: " << max_XYZ.x << " max_XYZ.y: " << max_XYZ.y << " max_XYZ.z: " << max_XYZ.z << std::endl;
            std::cout << " dx: " << dx << " dy: " << dy << " dz: " << dz << std::endl;
*/
    cloud_publisher.publish(cloud);
    //cloud_raw_publisher.publish(cloud);
    //publish_image(amplitude_publisher, mat_amplitude_flipped, curTime);
    publish_image(depth_bgr_Publisher, mat_depth_bgr, curTime);
    //depth_raw_Publisher.publish(float64MultiArray);
    
    //ROS_INFO("Publish pose ...");
    pose_camera_publisher.publish(pose_camera);
    //ROS_INFO("Publish done ...");

    n_frames ++;
}

//===================================================
void initialise()
{
    frameSeq = 0;
    ros::NodeHandle nh("~");

    nh.getParam("lens_Type", lensType);
    nh.getParam("lens_center_offset_x", lensCenterOffsetX);
    nh.getParam("lens_center_offset_y", lensCenterOffsetY);
    nh.getParam("start_stream", startStream);
    nh.getParam("image_type", imageType);
    nh.getParam("hdr_mode", hdr_mode);
    nh.getParam("int0", int0);
    nh.getParam("int1", int1);
    nh.getParam("int1", int2);
    nh.getParam("int_gray", intGr);
    nh.getParam("frequency_modulation", frequencyModulation);
    nh.getParam("channel", channel);
    nh.getParam("offset", offset);
    nh.getParam("min_amplitude", minAmplitude);
    nh.getParam("median_filter", medianFilter);
    nh.getParam("average_filter", averageFilter);
    nh.getParam("temporal_filter_factor", temporalFilterFactor);
    nh.getParam("temporal_filter_threshold", temporalFilterThreshold);
    nh.getParam("edge_threshold", edgeThreshold);
    nh.getParam("temporal_edge_threshold_low", temporalEdgeThresholdLow);
    nh.getParam("temporal_edge_threshold_high", temporalEdgeThresholdHigh);
    nh.getParam("interference_detection_limit", interferenceDetectionLimit);
    nh.getParam("use_last_value", useLastValue);
    nh.getParam("cartesian", cartesian);
    nh.getParam("publish_point_cloud", publishPointCloud);
    nh.getParam("video_camera", video_camera);
    nh.getParam("imu_port", imu_port);
    
    //video_camera = "/dev/v4l/by-id/" + video_camera;

    video_publisher = nh.advertise<sensor_msgs::Image>("video", 1);
    //amplitude_publisher = nh.advertise<sensor_msgs::Image>("amplitude", 1);
    cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBL> > ("cloud", 1);
    //cloud_raw_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBL> > ("cloud_raw", 1);
	//depth_raw_Publisher = nh.advertise<std_msgs::Float64MultiArray>("depth_raw", 1);
	depth_bgr_Publisher = nh.advertise<sensor_msgs::Image>("depth_bgr", 1);
	pose_camera_publisher     = nh.advertise<geometry_msgs::PoseStamped>("camera", 1);

    //connect to interface
    connectionCameraInfo = interface->subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
    connectionFrames = interface->subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });

    cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType); //0.02 mm - sensor pixel size
    old_lensCenterOffsetX = lensCenterOffsetX;
    old_lensCenterOffsetY = lensCenterOffsetY;
    old_lensType = lensType;

    //ROS_INFO("camera 660 driver eth version 1.7.0");
}

void start_imu_0()
{	
	imu_0->start();
}
//void start_imu_1()
//{	
//	imu_1->start();
//}

//==========================================================================
int main(int argc, char **argv)
{
	int n_points = 320 * 240;
	d3X = new double[n_points];
	d3Y = new double[n_points];
	d3Z = new double[n_points];
	initD3F(d3X, d3Y, d3Z, 320, 240, 108.0, 81.0); //80.0, 60.0); //85.0, 63.75);

    interface = new Interface();

    std::cerr << "Starting IMU ... " << std::endl;
    ROS_INFO("Starting IMU ...");

    // transform to world coordinates
    Eigen::Matrix3f m;
    m << 
    1, 0, 0,
    0, 0, 1,
    0, -1, 0;	
    t_sensor_2_world.rotate (m);

	//usleep(1000000);
	// to identify which is sensor IMU and which is vehicle IMU
	//float imu_0_angle_x = imu_0->Angle[0];
	//float imu_0_angle_y = imu_0->Angle[1];

	//float imu_1_angle_x = imu_1->Angle[0];
	//float imu_1_angle_y = imu_1->Angle[1];

	//std::cerr << "\n" << std::endl;
	//std::cerr << "Initial pose of imu_0: ";
	//std::cerr << "   angle_x = " << imu_0_angle_x << " angle_y = " << imu_0_angle_y << std::endl;

	//std::cerr << "Initial pose of imu_1: ";
	//std::cerr << "   angle_x = " << imu_1_angle_x << " angle_y = " << imu_1_angle_y << std::endl;

	//std::cerr << "\nBased on assumption that sensor IMU should have larger X angle, identified " ;
	//if (fabs(imu_0_angle_x) > fabs(imu_1_angle_x))
	//{
	//	imu_sensor = imu_0;
	//	imu_vehicle = imu_1;
	//	std::cerr << "imu_0 is the sensor IMU" << std::endl;
	//} else
	//{
	//	imu_sensor = imu_1;
	//	imu_vehicle = imu_0;
	//	std::cerr << "imu_1 is the sensor IMU" << std::endl;
	//}
	//std::cerr << "\n" << std::endl;

    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) ros::console::notifyLoggerLevelsChanged();
    std::cerr << "Starting ROS ... " << std::endl;

    ros::init(argc, argv, "c320_camera_node");
    //std::cerr << "Starting config server ... " << std::endl;
    dynamic_reconfigure::Server<vsemi_c320_camera::vsemi_c320_cameraConfig> server;
    dynamic_reconfigure::Server<vsemi_c320_camera::vsemi_c320_cameraConfig>::CallbackType f;
    //std::cerr << "Binding config ... " << std::endl;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    imageColorizer = new ImageColorizer();
    imageColorizer->setRange(0, 15000);
	
    initialise();
    setParameters();

    std::cerr << "IMU port: " << imu_port << std::endl;
    imu_0 = new IMU(imu_port.c_str(), 9600, 8, 'N', 1);
	
	std::thread th_start_imu_0(&start_imu_0);
	th_start_imu_0.detach();

	//imu_1 = new IMU("/dev/ttyUSB1", 9600, 8, 'N', 1);
	//std::thread th_start_imu_1(&start_imu_1);
	//th_start_imu_1.detach();

	//std::cerr << "IMU started. "<< std::endl;
    ROS_INFO("IMU started.");

    imu_sensor = imu_0;

    videoCapture = new cv::VideoCapture();
    std::cerr << "Starting RGB camera: " << video_camera << std::endl;
	video_ok = videoCapture->open(video_camera);
	if (video_ok)
	{
        videoCapture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        videoCapture->set(cv::CAP_PROP_FRAME_WIDTH, 320);
        videoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, 240);
	    std::cerr << "RGB camera started." << std::endl;

	std::thread th_start_RGB(&start_RGB);
	th_start_RGB.detach();
    } else 
    {
		std::cerr << "Can't initialize RGB camera." << std::endl;
	}

    st_time = std::chrono::steady_clock::now();
    startStreaming();

    ros::spin();

    en_time = std::chrono::steady_clock::now();
    interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
    frame_rate = ((double) n_frames) / interval;
    std::cout << "Frames: " << n_frames << ", time spent: " << interval << ", frame rate: " << frame_rate << std::endl;

	std::cerr << "Shutdown TOF ... " << std::endl;

    interface->stopStream();
	std::cerr << "Shutdown RGB camera ... " << std::endl;
    videoCapture->release();

	std::cerr << "Shutdown IMU ... " << std::endl;
    imu_0->stop();
    //imu_1->stop();

	usleep(2000000);
	std::cerr << "Shutdown ROS ... " << std::endl;

    delete imu_0;
    delete videoCapture;
    delete interface;

	delete[] d3X;
	delete[] d3Y;
	delete[] d3Z;
    //delete imu_1;

    ros::shutdown();
}