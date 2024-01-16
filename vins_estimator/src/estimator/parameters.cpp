/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;


bool		USE_MAG;
std::string MAG_TOPIC;
double	   MAG_MEASURE_NOISE;
double	   MAG_WORLD_NOISE;
double	   MAG_BIAS_NOISE;
double	   MAG_LPF_CUTOFF_FREQ;

bool	    USE_BARO;
std::string BARO_TOPIC;
std::string TEMPERATURE_TOPIC;
double	   BARO_MEASURE_NOISE;	 // 单位:m
double	   BARO_BIAS_NOISE;		 // 单位:m
double	   BARO_LPF_CUTOFF_FREQ; // 单位:Hz

bool		USE_EVALUATION;
std::string GT_TOPIC;
int GT_MODEL_ID;
std::string GROUND_TRUTH_PATH;

bool USE_GPU;



cv::FileStorage config;

template <typename T>
T readParam(std::string name) {
	T			 ans;
	cv::FileNode n = config[name];
	if (n.empty()) {
		ROS_ERROR_STREAM("Failed to find " << name << " in config file.");
		exit(-1);
	}
	n >> ans;
	return ans;
}
void readParameters(std::string config_file)
{
	try {
		config.open(config_file.c_str(), cv::FileStorage::READ);
	} catch (cv::Exception &ex) {
		std::cerr << "ERROR:" << ex.what() << " Can't open config file" << std::endl;
	}
	if (!config.isOpened()) {
		std::cerr << "ERROR: Wrong path to settings" << std::endl;
	}
    IMAGE0_TOPIC = readParam<std::string>("image0_topic");
    IMAGE1_TOPIC = readParam<std::string>("image1_topic");
    MAX_CNT = readParam<int>("max_cnt");
    MIN_DIST = readParam<int>("min_dist");
    F_THRESHOLD = readParam<double>("F_threshold");
    SHOW_TRACK = readParam<int>("show_track");
    FLOW_BACK = readParam<int>("flow_back");
    MULTIPLE_THREAD = readParam<int>("multiple_thread");

    USE_GPU = readParam<bool>("use_gpu");
    USE_IMU = readParam<int>("imu");
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        IMU_TOPIC = readParam<std::string>("imu_topic");
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());

        double acc_n_base = readParam<double>("acc_n");
        double acc_w_base = readParam<double>("acc_w");
        double gyr_n_base = readParam<double>("gyr_n");
        double gyr_w_base = readParam<double>("gyr_w");
        double acc_n_factor = readParam<double>("acc_n_factor");
        double acc_w_factor = readParam<double>("acc_w_factor");
        double gyr_n_factor = readParam<double>("gyr_n_factor");
        double gyr_w_factor = readParam<double>("gyr_w_factor");
        ACC_N = acc_n_base * acc_n_factor;
        ACC_W = acc_w_base * acc_w_factor;
        GYR_N = gyr_n_base * gyr_n_factor;
        GYR_W = gyr_w_base * gyr_w_factor;
        G.z() = readParam<double>("g_norm");
    }
    USE_MAG = readParam<bool>("use_mag");
	if (USE_MAG) {
        double mag_measure_factor = readParam<double>("mag_measure_factor");
        double mag_world_factor = readParam<double>("mag_world_factor");
        double mag_bias_factor = readParam<double>("mag_bias_factor");
		MAG_TOPIC			= readParam<std::string>("mag_topic");
		MAG_MEASURE_NOISE	= readParam<double>("mag_measure_noise")*mag_measure_factor;
		MAG_WORLD_NOISE		= readParam<double>("mag_world_noise")*mag_world_factor;
		MAG_BIAS_NOISE		= readParam<double>("mag_bias_noise")*mag_bias_factor;
		MAG_LPF_CUTOFF_FREQ = readParam<double>("mag_lpf_cutoff_freq");
	}
    USE_BARO = readParam<int>("use_baro");
	if (USE_BARO) {
        double baro_measure_factor = readParam<double>("baro_measure_factor");
        double baro_bias_factor = readParam<double>("baro_bias_factor");
		BARO_TOPIC			 = readParam<std::string>("baro_topic");
		TEMPERATURE_TOPIC	 = readParam<std::string>("temperature_topic");
		BARO_MEASURE_NOISE	 = readParam<double>("baro_measure_noise")*baro_measure_factor;
		BARO_BIAS_NOISE		 = readParam<double>("baro_bias_noise")*baro_bias_factor;
		BARO_LPF_CUTOFF_FREQ = readParam<double>("baro_lpf_cutoff_freq");
	}
    USE_EVALUATION = readParam<bool>("use_evaluation");
	if (USE_EVALUATION) {
		GT_TOPIC = readParam<std::string>("gt_topic");
        GT_MODEL_ID = readParam<int>("gt_model_id");
	}

    SOLVER_TIME = readParam<double>("max_solver_time");
    NUM_ITERATIONS = readParam<int>("max_num_iterations");
    MIN_PARALLAX = readParam<double>("keyframe_parallax");
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    OUTPUT_FOLDER = readParam<std::string>("output_path");
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
	GROUND_TRUTH_PATH = OUTPUT_FOLDER + "/groundtruth.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();
    std::ofstream fout_gt(GROUND_TRUTH_PATH, std::ios::out);
	fout_gt.close();

    ESTIMATE_EXTRINSIC = readParam<int>("estimate_extrinsic");
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T = readParam<cv::Mat>("body_T_cam0");
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    NUM_OF_CAM = readParam<int>("num_of_cam");
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib = readParam<std::string>("cam0_calib");
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib = readParam<std::string>("cam1_calib");
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T = readParam<cv::Mat>("body_T_cam1");
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = readParam<double>("td");
    ESTIMATE_TD = readParam<int>("estimate_td");
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = readParam<int>("image_height");
    COL = readParam<int>("image_width");

    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }
    config.release();
}
