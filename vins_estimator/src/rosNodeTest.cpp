/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <gazebo_msgs/ModelStates.h>

Estimator estimator;

sensor_msgs::Temperature temperature_msg;
bool get_temperature = false;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

ros::Publisher pub_mag_lpf;
ros::Publisher		pub_baro_alt;
ros::Publisher		pub_raw_baro_alt;
ros::Publisher      gt_pose_pub;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

void mag_callback(const sensor_msgs::MagneticFieldConstPtr &mag_msg) {
	double	 t	= mag_msg->header.stamp.toSec();
	double	 mx = mag_msg->magnetic_field.x;
	double	 my = mag_msg->magnetic_field.y;
	double	 mz = mag_msg->magnetic_field.z;
	Vector3d mag(mx, my, mz); // 传到后端为ENU坐标系
    static Utility::LPF_3D mag_lpf(1.0 / 80, MAG_LPF_CUTOFF_FREQ);
	Eigen::Vector3d mag_lpfed = mag_lpf.update(mag);
    sensor_msgs::MagneticField mag_lpf_msg;
    mag_lpf_msg.header = mag_msg->header;
    mag_lpf_msg.magnetic_field.x = mag_lpfed.x();
    mag_lpf_msg.magnetic_field.y = mag_lpfed.y();
    mag_lpf_msg.magnetic_field.z = mag_lpfed.z();
    pub_mag_lpf.publish(mag_lpf_msg);
	estimator.inputMag(t, mag_lpfed);
}

void temperature_callback(const sensor_msgs::Temperature &_temperature_msg) {
	temperature_msg = _temperature_msg;
	get_temperature = true;
}
float PressureToAltitude(float pressure_pa, float temperature)  {
    static constexpr float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15f; // °C
    static constexpr float CONSTANTS_AIR_GAS_CONST		   = 287.1f;   // J/(kg * K)
    static constexpr float CONSTANTS_ONE_G				   = 9.80665f; // m/s^2
    // calculate altitude using the hypsometric equation
    static constexpr float T1 = 15.f - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // temperature at base height in Kelvin
    static constexpr float a  = -6.5f / 1000.f;							// temperature gradient in degrees per metre

    static constexpr float sens_baro_qnh = 1013.25f;

    // current pressure at MSL in kPa (QNH in hPa)
    const float p1 = sens_baro_qnh * 0.1f;

    // measured pressure in kPa
    const float p = pressure_pa * 0.001f;

    /*
    * Solve:
    *
    *     /        -(aR / g)     \
    *    | (p / p1)          . T1 | - T1
    *     \                      /
    * h = -------------------------------  + h1
    *                   a
    */
    float altitude = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;

    return altitude;
}

void baro_callback(const sensor_msgs::FluidPressurePtr &baro_msg) {

	static bool			   init_baro_alt = false;
	static float		   home_alt		 = 0;
    static const int       baro_sample_rate = 40;
	static Utility::LPF_1D baro_lpf(1.0 / baro_sample_rate , BARO_LPF_CUTOFF_FREQ);
	if (!get_temperature){
        ROS_WARN("No temperature message received, baro altitude will not be caculated.");
        return;
    }
	double t			   = baro_msg->header.stamp.toSec();
	double pressure_pa	   = baro_msg->fluid_pressure;
	float  baro_alt		   = PressureToAltitude(pressure_pa, temperature_msg.temperature);
	double baro_alt_smooth = baro_lpf.update(baro_alt);
	if (!init_baro_alt) {
		home_alt	  = baro_alt_smooth;
		init_baro_alt = true;
	}
	std_msgs::Float32 baro_alt_msg;
	baro_alt_msg.data = baro_alt_smooth - home_alt;
	pub_baro_alt.publish(baro_alt_msg);

	std_msgs::Float32 raw_baro_alt_msg;
	raw_baro_alt_msg.data = baro_alt - home_alt;
	pub_raw_baro_alt.publish(raw_baro_alt_msg);
	estimator.inputBaro(t, baro_alt_smooth - home_alt);
}

void ground_truth_callback(const gazebo_msgs::ModelStatesPtr &msg) {
	// write result to file
    assert(int(msg->name.size()) > GT_MODEL_ID && "GT_MODEL_ID out of model range");
    auto time_now = ros::Time().now();
    geometry_msgs::Pose &pose = msg->pose[GT_MODEL_ID];
	ofstream foutC(GROUND_TRUTH_PATH, ios::app);
	foutC.setf(ios::fixed, ios::floatfield);
	foutC.precision(9);
	foutC <<time_now.toSec() << " ";
	foutC.precision(5);
	foutC << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
		  << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z
		  << " " << pose.orientation.w << endl;
	foutC.close();
    // geometry_msgs::PoseStamped gt_pose_msg;
    // gt_pose_msg.header.stamp = time_now;
    // gt_pose_msg.header.frame_id = "world";
    // gt_pose_msg.pose = pose;
    // gt_pose_pub.publish(gt_pose_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu;
    ros::Subscriber sub_mag;
    ros::Subscriber sub_baro;
    ros::Subscriber sub_imu_temperautre;
    ros::Subscriber sub_gt;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 10, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    if(USE_MAG)
    {
        sub_mag = n.subscribe(MAG_TOPIC, 10, mag_callback,ros::TransportHints().tcpNoDelay());
        pub_mag_lpf = n.advertise<sensor_msgs::MagneticField>("/vins_estimator/mag_lpf", 10);
    }
    if(USE_BARO){
        sub_baro = n.subscribe(BARO_TOPIC, 10, baro_callback,ros::TransportHints().tcpNoDelay());
        sub_imu_temperautre = n.subscribe(TEMPERATURE_TOPIC, 10, temperature_callback,ros::TransportHints().tcpNoDelay());
        pub_baro_alt	 = n.advertise<std_msgs::Float32>("baro_alt", 10);
		pub_raw_baro_alt = n.advertise<std_msgs::Float32>("raw_baro_alt", 10);
    }
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_img1;
    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback,ros::TransportHints().tcpNoDelay());
    }
    if(USE_EVALUATION)
    {
        sub_gt = n.subscribe(GT_TOPIC, 10, ground_truth_callback,ros::TransportHints().tcpNoDelay());
        //gt_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/vins_estimator/ground_truth", 10);
    }


    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
