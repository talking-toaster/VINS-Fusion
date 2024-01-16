/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"
#include "../utility/tic_toc.h"

Estimator::Estimator() : f_manager{Rs} {
	ROS_INFO("init begins");
	initThreadFlag = false;
	clearState();
}

Estimator::~Estimator() {
	if (MULTIPLE_THREAD) {
		processThread.join();
		printf("join thread \n");
	}
}

void Estimator::clearState() {
	mProcess.lock();
	while (!accBuf.empty())
		accBuf.pop();
	while (!gyrBuf.empty())
		gyrBuf.pop();
	while (!featureBuf.empty())
		featureBuf.pop();

	prevTime		  = -1;
	curTime			  = 0;
	openExEstimation  = 0;
	initP			  = Eigen::Vector3d(0, 0, 0);
	initR			  = Eigen::Matrix3d::Identity();
	inputImageCnt	  = 0;
	initFirstPoseFlag = false;

	for (int i = 0; i < WINDOW_SIZE + 1; i++) {
		Rs[i].setIdentity();
		Ps[i].setZero();
		Vs[i].setZero();
		Bas[i].setZero();
		Bgs[i].setZero();
		Mw[i].setZero();
		Bms[i].setZero();
		mag_measure[i].setZero();
		baro_measure[i] = 0;
		Bbs[i]			= 0;
		dt_buf[i].clear();
		linear_acceleration_buf[i].clear();
		angular_velocity_buf[i].clear();

		if (pre_integrations[i] != nullptr) {
			delete pre_integrations[i];
		}
		pre_integrations[i] = nullptr;
	}

	for (int i = 0; i < NUM_OF_CAM; i++) {
		tic[i] = Vector3d::Zero();
		ric[i] = Matrix3d::Identity();
	}

	first_imu = false, sum_of_back = 0;
	sum_of_front	  = 0;
	frame_count		  = 0;
	solver_flag		  = INITIAL;
	initial_timestamp = 0;
	all_image_frame.clear();

	if (tmp_pre_integration != nullptr)
		delete tmp_pre_integration;
	if (last_marginalization_info != nullptr)
		delete last_marginalization_info;

	tmp_pre_integration		  = nullptr;
	last_marginalization_info = nullptr;
	last_marginalization_parameter_blocks.clear();

	f_manager.clearState();

	failure_occur = 0;

	mProcess.unlock();
}

void Estimator::setParameter() {
	mProcess.lock();
	for (int i = 0; i < NUM_OF_CAM; i++) {
		tic[i] = TIC[i];
		ric[i] = RIC[i];
		cout << " exitrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
	}
	f_manager.setRic(ric);
	ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
	ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
	ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
	td										  = TD;
	g										  = G;
	cout << "set g " << g.transpose() << endl;
	featureTracker.readIntrinsicParameter(CAM_NAMES);

	std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
	if (MULTIPLE_THREAD && !initThreadFlag) {
		initThreadFlag = true;
		processThread  = std::thread(&Estimator::processMeasurements, this);
	}
	mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo) {
	bool restart = false;
	mProcess.lock();
	if (!use_imu && !use_stereo)
		printf("at least use two sensors! \n");
	else {
		if (USE_IMU != use_imu) {
			USE_IMU = use_imu;
			if (USE_IMU) {
				// reuse imu; restart system
				restart = true;
			} else {
				if (last_marginalization_info != nullptr)
					delete last_marginalization_info;

				tmp_pre_integration		  = nullptr;
				last_marginalization_info = nullptr;
				last_marginalization_parameter_blocks.clear();
			}
		}

		STEREO = use_stereo;
		printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
	}
	mProcess.unlock();
	if (restart) {
		clearState();
		setParameter();
	}
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1) {
	static TicToc t_front;
	t_front.tic();

	map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;

	if (_img1.empty())
		featureFrame = featureTracker.trackImage(t, _img);
	else
		featureFrame = featureTracker.trackImage(t, _img, _img1);

	if (SHOW_TRACK) {
		cv::Mat imgTrack = featureTracker.getTrackImage();
		pubTrackImage(imgTrack, t);
	}

	if (MULTIPLE_THREAD) {
		if (++inputImageCnt % 2 == 0) {
			// if (++inputImageCnt > 1) {
			// mBuf.lock();
			featureBuf.push(make_pair(t, featureFrame));
			// mBuf.unlock();
		}
	}

	else {
		// mBuf.lock();
		featureBuf.push(make_pair(t, featureFrame));
		// mBuf.unlock();
		TicToc processTime;
		processMeasurements();
		printf("process time: %f\n", processTime.toc());
	}
	t_front.toc();
	if (inputImageCnt > 2) {
		ROS_INFO("%s[frontend] used: cur: %3.1fms avg: %3.1fms%s%s", ANSI_COLOR_BLUE, t_front.cur_ms, t_front.avg_ms,
				 ANSI_COLOR_BLUE, ANSI_COLOR_RESET);
	} else {
		t_front.reset();
	}
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity) {
	mBuf.lock();
	accBuf.push(make_pair(t, linearAcceleration));
	gyrBuf.push(make_pair(t, angularVelocity));
	// printf("input imu with time %f \n", t);
	mBuf.unlock();

	if (solver_flag == NON_LINEAR) {
		mPropagate.lock();
		fastPredictIMU(t, linearAcceleration, angularVelocity);
		pubLatestOdometry(latest_P, latest_Q, latest_V, t);
		mPropagate.unlock();
	}
}
void Estimator::inputMag(double t, const Vector3d &mag) {
	if (magBuf.size() > 100)
		magBuf.pop();
	magBuf.push(make_pair(t, mag));
}
void Estimator::inputBaro(double t, const float baro_rel_alt) {
	if (baroBuf.size() > 100)
		baroBuf.pop();
	baroBuf.push(make_pair(t, baro_rel_alt));
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame) {
	featureBuf.push(make_pair(t, featureFrame));

	if (!MULTIPLE_THREAD)
		processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
							   vector<pair<double, Eigen::Vector3d>> &gyrVector) {
	if (accBuf.empty()) {
		printf("not receive imu\n");
		return false;
	}
	// printf("get imu from %f %f\n", t0, t1);
	// printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
	if (t1 <= accBuf.back().first) {
		while (accBuf.front().first <= t0) {
			accBuf.pop();
			gyrBuf.pop();
		}
		while (accBuf.front().first < t1) {
			accVector.push_back(accBuf.front());
			accBuf.pop();
			gyrVector.push_back(gyrBuf.front());
			gyrBuf.pop();
		}
		accVector.push_back(accBuf.front());
		gyrVector.push_back(gyrBuf.front());
	} else {
		printf("wait for imu\n");
		return false;
	}
	return true;
}

bool Estimator::IMUAvailable(double t) {
	if (!accBuf.empty() && t <= accBuf.back().first)
		return true;
	else
		return false;
}
bool Estimator::getMag(double t, Eigen::Vector3d &mag) {
	if (magBuf.empty()) {
		printf("not receive mag\n");
		return false;
	}
	double			t1 = 0, t2;
	Eigen::Vector3d prev_mag;
	Eigen::Vector3d next_mag;

	while (magBuf.front() && magBuf.front()->first < t) {
		std::pair<double, Eigen::Vector3d> mag_stamped;
		if (!magBuf.try_pop(mag_stamped)) {
			ROS_WARN("magBuf try_pop failed");
			return false;
		}
		prev_mag = mag_stamped.second;
		t1		 = mag_stamped.first;
		mag		 = prev_mag;
	}
	// assert(!magBuf.empty() && "magBuf should not be empty");
	if (!magBuf.empty()) {
		next_mag = magBuf.front()->second;
		t2		 = magBuf.front()->first;
		assert(t1 >= 0 && t2 >= 0 && "t1 t2 should be positive");
		mag = (t - t1) / (t2 - t1) * (next_mag - prev_mag) + prev_mag;
		// ROS_WARN_STREAM("t1: " << t1 << " prev_mag: " << prev_mag << " t2: " << t2 << " next_mag:" << next_mag
		// 					   << " t:" << t << " mag:" << mag);
	} else {
		mag = prev_mag;
	}
	return true;
}
bool Estimator::getBaro(double t, double &baro_rel_alt) {
	if (baroBuf.empty()) {
		printf("not receive baro\n");
		return false;
	}
	double t1 = 0, t2 = 0;
	double prev_baro = INFINITY;
	double next_baro = 0;
	assert(!baroBuf.empty() && "baroBuf should not be empty");
	assert(baroBuf.front() && "baroBuf.front() should not be empty");

	while (baroBuf.front() && baroBuf.front()->first < t) {
		prev_baro = baroBuf.front()->second;
		t1		  = t - baroBuf.front()->first;
		baroBuf.pop();
	}
	if (!baroBuf.empty()) {
		next_baro = baroBuf.front()->second;
		t2		  = baroBuf.front()->first - t;
		if (std::isinf(prev_baro)) {
			baro_rel_alt = next_baro - Vs[frame_count].z() * t2;
			return true;
		} else {
			double dt	 = t1 + t2;
			baro_rel_alt = (t2 / dt) * prev_baro + (t1 / dt) * next_baro;
			return true;
		}
	} else { // no next_baro
		baro_rel_alt = prev_baro + Vs[frame_count].z() * t1;
		return true;
	}
}


void Estimator::processMag(const Vector3d &mag) {
	if (frame_count == 0) {
		Mw[0]  = Rs[0] * mag;
		Bms[0] = Vector3d(0, 0, 0);
	} else {
		Mw[frame_count]	 = Mw[frame_count - 1];
		Bms[frame_count] = Bms[frame_count - 1];
	}
	mag_measure[frame_count] = mag;
}
void Estimator::processBaro(const double baro_rel_alt) {
	baro_measure[frame_count] = baro_rel_alt;
	Bbs[frame_count]		  = frame_count == 0 ? 0 : Bbs[frame_count - 1];
}
void Estimator::processMeasurements() {
	while (1) {
		static TicToc														   t_process;
		pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;
		vector<pair<double, Eigen::Vector3d>>								   accVector, gyrVector;
		Eigen::Vector3d														   mag			= Eigen::Vector3d(0, 0, 0);
		double																   baro_rel_alt = 0.0;
		bool																   have_mag = false, have_baro = false;
		if (featureBuf.try_pop(feature)) {
			if (solver_flag == Estimator::SolverFlag::NON_LINEAR) {
				while (featureBuf.try_pop(feature))
					;
			}


			curTime = feature.first + td;
			while (1) {
				if ((!USE_IMU || IMUAvailable(curTime)))
					break;
				else {
					printf("wait for imu ... \n");
					if (!MULTIPLE_THREAD)
						return;
					std::chrono::milliseconds dura(5);
					std::this_thread::sleep_for(dura);
				}
			}
			mBuf.lock();
			if (USE_IMU)
				getIMUInterval(prevTime, curTime, accVector, gyrVector);
			mBuf.unlock();

			t_process.tic();
			if (USE_IMU) {
				if (!initFirstPoseFlag) {
					if (USE_MAG) {
						while (!getMag(curTime, mag)) {
							ROS_WARN("wait for mag");
							std::this_thread::sleep_for(std::chrono::milliseconds(5));
						}
						have_mag = true;
					}
					if (USE_BARO) {
						while (!getBaro(curTime, baro_rel_alt)) {
							ROS_WARN("wait for baro");
							std::this_thread::sleep_for(std::chrono::milliseconds(5));
						}
						have_baro = true;
					}
					initFirstIMUPose(accVector, mag, baro_rel_alt, have_mag, have_baro);
				}
				for (size_t i = 0; i < accVector.size(); i++) {
					double dt;
					if (i == 0)
						dt = accVector[i].first - prevTime;
					else if (i == accVector.size() - 1)
						dt = curTime - accVector[i - 1].first;
					else
						dt = accVector[i].first - accVector[i - 1].first;
					processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
				}
				if (USE_MAG) {
					if (!have_mag)
						getMag(curTime, mag);
					processMag(mag);
				}
				if (USE_BARO) {
					if (!have_baro)
						getBaro(curTime, baro_rel_alt);
					processBaro(baro_rel_alt);
				}
			}
			mProcess.lock();
			processImage(feature.second, feature.first);
			prevTime = curTime;

			printStatistics(*this, 0);

			std_msgs::Header header;
			header.frame_id = "world";
			header.stamp	= ros::Time(feature.first);

			pubOdometry(*this, header);
			pubKeyPoses(*this, header);
			pubCameraPose(*this, header);
			pubPointCloud(*this, header);
			pubKeyframe(*this);
			pubTF(*this, header);
			mProcess.unlock();


			t_process.toc();
			ROS_INFO("%s[backend] used: cur: %3.1fms avg: %3.1fms%s%s", ANSI_COLOR_BLUE, t_process.cur_ms,
					 t_process.avg_ms, ANSI_COLOR_BLUE, ANSI_COLOR_RESET);
			// for(int i=0;i<WINDOW_SIZE+1;i++){
			//     std::cout<<Bbs[i]<<" ";
			// }
			// std::cout<<"\n";
			// for(int i=0;i<WINDOW_SIZE+1;i++){
			//     std::cout<<baro_measure[i]<<" ";
			// }
			// std::cout<<"\n";
			// for(int i=0;i<WINDOW_SIZE+1;i++){
			//     std::cout<<Ps[i][2]<<" ";
			// }
			// std::cout<<"\n";
		}

		if (!MULTIPLE_THREAD)
			break;

		std::chrono::milliseconds dura(2);
		std::this_thread::sleep_for(dura);
	}
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector, const Eigen::Vector3d &mag,
								 double baro_ref_alt, bool have_mag, bool have_baro) {
	printf("init first imu pose\n");
	initFirstPoseFlag = true;
	// return;
	Eigen::Vector3d averAcc(0, 0, 0);
	int				n = (int)accVector.size();
	for (size_t i = 0; i < accVector.size(); i++) {
		averAcc = averAcc + accVector[i].second;
	}
	averAcc = averAcc / n;
	printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
	Matrix3d R0	 = Utility::g2R(averAcc);
	double	 yaw = Utility::R2ypr(R0).x();
	R0			 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
	ROS_INFO_STREAM("mag:" << mag.transpose());
	if (USE_MAG && have_mag) {
		Vector3d mag_enu = R0 * mag;
		if (mag_enu.y() == 0)
			mag_enu.y() = 1e-20;
		double mag_yaw = atan2(mag_enu.x(), mag_enu.y()) * 180 / M_PI; // 正东为0度
		mag_yaw += 7;
		R0 = Utility::ypr2R(Eigen::Vector3d{mag_yaw, 0, 0}) * R0;
		ROS_WARN("init yaw by mag %f", mag_yaw);
	}
	Rs[0] = R0;
	std::cout << "init R0 " << endl << Rs[0] << endl;
	ROS_INFO_STREAM("init ypr:" << Utility::R2ypr(R0).transpose());
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r) {
	Ps[0] = p;
	Rs[0] = r;
	initP = p;
	initR = r;
}


void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity) {
	if (!first_imu) {
		first_imu = true;
		acc_0	  = linear_acceleration;
		gyr_0	  = angular_velocity;
	}

	if (!pre_integrations[frame_count]) {
		pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
	}
	if (frame_count != 0) {
		pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
		// if(solver_flag != NON_LINEAR)
		tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

		dt_buf[frame_count].push_back(dt);
		linear_acceleration_buf[frame_count].push_back(linear_acceleration);
		angular_velocity_buf[frame_count].push_back(angular_velocity);

		int		 j		  = frame_count;
		Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
		Vector3d un_gyr	  = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
		Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
		Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
		Vector3d un_acc	  = 0.5 * (un_acc_0 + un_acc_1);
		Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
		Vs[j] += dt * un_acc;
	}
	acc_0 = linear_acceleration;
	gyr_0 = angular_velocity;
}



bool Estimator::initialStructure() {
	TicToc t_sfm;
	// check imu observibility
	{
		map<double, ImageFrame>::iterator frame_it;
		Vector3d						  sum_g;
		for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
			double	 dt	   = frame_it->second.pre_integration->sum_dt;
			Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
			sum_g += tmp_g;
		}
		Vector3d aver_g;
		aver_g	   = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
		double var = 0;
		for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
			double	 dt	   = frame_it->second.pre_integration->sum_dt;
			Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
			var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
			// cout << "frame g " << tmp_g.transpose() << endl;
		}
		var = sqrt(var / ((int)all_image_frame.size() - 1));
		// ROS_WARN("IMU variation %f!", var);
		if (var < 0.25) {
			ROS_INFO("IMU excitation not enouth!");
			// return false;
		}
	}
	// global sfm
	Quaterniond		   Q[frame_count + 1];
	Vector3d		   T[frame_count + 1];
	map<int, Vector3d> sfm_tracked_points;
	vector<SFMFeature> sfm_f;
	for (auto &it_per_id : f_manager.feature) {
		int		   imu_j = it_per_id.start_frame - 1;
		SFMFeature tmp_feature;
		tmp_feature.state = false;
		tmp_feature.id	  = it_per_id.feature_id;
		for (auto &it_per_frame : it_per_id.feature_per_frame) {
			imu_j++;
			Vector3d pts_j = it_per_frame.point;
			tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
		}
		sfm_f.push_back(tmp_feature);
	}
	Matrix3d relative_R;
	Vector3d relative_T;
	int		 l;
	if (!relativePose(relative_R, relative_T, l)) {
		ROS_INFO("Not enough features or parallax; Move device around");
		return false;
	}
	GlobalSFM sfm;
	if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f, sfm_tracked_points)) {
		ROS_DEBUG("global SFM failed!");
		marginalization_flag = MARGIN_OLD;
		return false;
	}

	// solve pnp for all frame
	map<double, ImageFrame>::iterator frame_it;
	map<int, Vector3d>::iterator	  it;
	frame_it = all_image_frame.begin();
	for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
		// provide initial guess
		cv::Mat r, rvec, t, D, tmp_r;
		if ((frame_it->first) == Headers[i]) {
			frame_it->second.is_key_frame = true;
			frame_it->second.R			  = Q[i].toRotationMatrix() * RIC[0].transpose();
			frame_it->second.T			  = T[i];
			i++;
			continue;
		}
		if ((frame_it->first) > Headers[i]) {
			i++;
		}
		Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
		Vector3d P_inital = -R_inital * T[i];
		cv::eigen2cv(R_inital, tmp_r);
		cv::Rodrigues(tmp_r, rvec);
		cv::eigen2cv(P_inital, t);

		frame_it->second.is_key_frame = false;
		vector<cv::Point3f> pts_3_vector;
		vector<cv::Point2f> pts_2_vector;
		for (auto &id_pts : frame_it->second.points) {
			int feature_id = id_pts.first;
			for (auto &i_p : id_pts.second) {
				it = sfm_tracked_points.find(feature_id);
				if (it != sfm_tracked_points.end()) {
					Vector3d	world_pts = it->second;
					cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
					pts_3_vector.push_back(pts_3);
					Vector2d	img_pts = i_p.second.head<2>();
					cv::Point2f pts_2(img_pts(0), img_pts(1));
					pts_2_vector.push_back(pts_2);
				}
			}
		}
		cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
		if (pts_3_vector.size() < 6) {
			cout << "pts_3_vector size " << pts_3_vector.size() << endl;
			ROS_DEBUG("Not enough points for solve pnp !");
			return false;
		}
		if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
			ROS_DEBUG("solve pnp fail!");
			return false;
		}
		cv::Rodrigues(rvec, r);
		MatrixXd R_pnp, tmp_R_pnp;
		cv::cv2eigen(r, tmp_R_pnp);
		R_pnp = tmp_R_pnp.transpose();
		MatrixXd T_pnp;
		cv::cv2eigen(t, T_pnp);
		T_pnp			   = R_pnp * (-T_pnp);
		frame_it->second.R = R_pnp * RIC[0].transpose();
		frame_it->second.T = T_pnp;
	}
	if (visualInitialAlign())
		return true;
	else {
		ROS_INFO("misalign visual structure with IMU");
		return false;
	}
}

bool Estimator::visualInitialAlign() {
	TicToc	 t_g;
	VectorXd x;
	// solve scale
	bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
	if (!result) {
		ROS_DEBUG("solve g failed!");
		return false;
	}

	// change state
	for (int i = 0; i <= frame_count; i++) {
		Matrix3d Ri								 = all_image_frame[Headers[i]].R;
		Vector3d Pi								 = all_image_frame[Headers[i]].T;
		Ps[i]									 = Pi;
		Rs[i]									 = Ri;
		all_image_frame[Headers[i]].is_key_frame = true;
	}

	double s = (x.tail<1>())(0);
	for (int i = 0; i <= WINDOW_SIZE; i++) {
		pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
	}
	for (int i = frame_count; i >= 0; i--)
		Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
	int								  kv = -1;
	map<double, ImageFrame>::iterator frame_i;
	for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) {
		if (frame_i->second.is_key_frame) {
			kv++;
			Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
		}
	}

	Matrix3d R0	 = Utility::g2R(g);
	double	 yaw = Utility::R2ypr(R0 * Rs[0]).x();
	R0			 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
	g			 = R0 * g;
	// Matrix3d rot_diff = R0 * Rs[0].transpose();
	Matrix3d rot_diff = R0;
	for (int i = 0; i <= frame_count; i++) {
		Ps[i] = rot_diff * Ps[i];
		Rs[i] = rot_diff * Rs[i];
		Vs[i] = rot_diff * Vs[i];
	}
	ROS_DEBUG_STREAM("g0     " << g.transpose());
	ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

	f_manager.clearDepth();
	f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

	return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l) {
	// find previous frame which contians enough correspondance and parallex with newest frame
	for (int i = 0; i < WINDOW_SIZE; i++) {
		vector<pair<Vector3d, Vector3d>> corres;
		corres = f_manager.getCorresponding(i, WINDOW_SIZE);
		if (corres.size() > 20) {
			double sum_parallax = 0;
			double average_parallax;
			for (int j = 0; j < int(corres.size()); j++) {
				Vector2d pts_0(corres[j].first(0), corres[j].first(1));
				Vector2d pts_1(corres[j].second(0), corres[j].second(1));
				double	 parallax = (pts_0 - pts_1).norm();
				sum_parallax	  = sum_parallax + parallax;
			}
			average_parallax = 1.0 * sum_parallax / int(corres.size());
			if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
				l = i;
				ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure",
						  average_parallax * 460, l);
				return true;
			}
		}
	}
	return false;
}

void Estimator::vector2double() {
	for (int i = 0; i <= WINDOW_SIZE; i++) {
		para_Pose[i][0] = Ps[i].x();
		para_Pose[i][1] = Ps[i].y();
		para_Pose[i][2] = Ps[i].z();
		Quaterniond q{Rs[i]};
		para_Pose[i][3] = q.x();
		para_Pose[i][4] = q.y();
		para_Pose[i][5] = q.z();
		para_Pose[i][6] = q.w();

		if (USE_IMU) {
			para_SpeedBias[i][0] = Vs[i].x();
			para_SpeedBias[i][1] = Vs[i].y();
			para_SpeedBias[i][2] = Vs[i].z();

			para_SpeedBias[i][3] = Bas[i].x();
			para_SpeedBias[i][4] = Bas[i].y();
			para_SpeedBias[i][5] = Bas[i].z();

			para_SpeedBias[i][6] = Bgs[i].x();
			para_SpeedBias[i][7] = Bgs[i].y();
			para_SpeedBias[i][8] = Bgs[i].z();
		}
		if (USE_MAG) {
			para_mag[i][0] = Mw[i].x();
			para_mag[i][1] = Mw[i].y();
			para_mag[i][2] = Mw[i].z();
			para_mag[i][3] = Bms[i].x();
			para_mag[i][4] = Bms[i].y();
			para_mag[i][5] = Bms[i].z();
		}
		if (USE_BARO) {
			para_baro[i][0] = Bbs[i];
		}
	}

	for (int i = 0; i < NUM_OF_CAM; i++) {
		para_Ex_Pose[i][0] = tic[i].x();
		para_Ex_Pose[i][1] = tic[i].y();
		para_Ex_Pose[i][2] = tic[i].z();
		Quaterniond q{ric[i]};
		para_Ex_Pose[i][3] = q.x();
		para_Ex_Pose[i][4] = q.y();
		para_Ex_Pose[i][5] = q.z();
		para_Ex_Pose[i][6] = q.w();
	}


	VectorXd dep = f_manager.getDepthVector();
	for (int i = 0; i < f_manager.getFeatureCount(); i++)
		para_Feature[i][0] = dep(i);

	para_Td[0][0] = td;
}

void Estimator::double2vector() {
	Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
	Vector3d origin_P0 = Ps[0];

	if (failure_occur) {
		origin_R0	  = Utility::R2ypr(last_R0);
		origin_P0	  = last_P0;
		failure_occur = 0;
	}

	if (USE_IMU) {
		Vector3d origin_R00 = Utility::R2ypr(
			Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5]).toRotationMatrix());
		double y_diff = origin_R0.x() - origin_R00.x();
		// TODO
		Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
		if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
			ROS_DEBUG("euler singular point!");
			rot_diff = Rs[0] * Quaterniond(para_Pose[0][6], para_Pose[0][3], para_Pose[0][4], para_Pose[0][5])
								   .toRotationMatrix()
								   .transpose();
		}

		for (int i = 0; i <= WINDOW_SIZE; i++) {

			Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
								   .normalized()
								   .toRotationMatrix();

			Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0], para_Pose[i][1] - para_Pose[0][1],
										para_Pose[i][2] - para_Pose[0][2]) +
					origin_P0;


			Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2]);

			Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5]);

			Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8]);
		}
	} else {
		for (int i = 0; i <= WINDOW_SIZE; i++) {
			Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5])
						.normalized()
						.toRotationMatrix();

			Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
		}
	}

	if (USE_IMU) {
		for (int i = 0; i < NUM_OF_CAM; i++) {
			tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
			ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3], para_Ex_Pose[i][4], para_Ex_Pose[i][5])
						 .normalized()
						 .toRotationMatrix();
		}
	}
	if (USE_MAG) {
		for (int i = 0; i <= WINDOW_SIZE; i++) {
			Mw[i]  = Vector3d(para_mag[i][0], para_mag[i][1], para_mag[i][2]);
			Bms[i] = Vector3d(para_mag[i][3], para_mag[i][4], para_mag[i][5]);
		}
	}
	if (USE_BARO) {
		for (int i = 0; i <= WINDOW_SIZE; i++) {
			Bbs[i] = para_baro[i][0];
		}
	}

	VectorXd dep = f_manager.getDepthVector();
	for (int i = 0; i < f_manager.getFeatureCount(); i++)
		dep(i) = para_Feature[i][0];
	f_manager.setDepth(dep);

	if (USE_IMU)
		td = para_Td[0][0];
}

bool Estimator::failureDetection() {
	return false;
	if (f_manager.last_track_num < 2) {
		ROS_INFO(" little feature %d", f_manager.last_track_num);
		// return true;
	}
	if (Bas[WINDOW_SIZE].norm() > 2.5) {
		ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
		return true;
	}
	if (Bgs[WINDOW_SIZE].norm() > 1.0) {
		ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
		return true;
	}
	/*
	if (tic(0) > 1)
	{
		ROS_INFO(" big extri param estimation %d", tic(0) > 1);
		return true;
	}
	*/
	Vector3d tmp_P = Ps[WINDOW_SIZE];
	if ((tmp_P - last_P).norm() > 5) {
		// ROS_INFO(" big translation");
		// return true;
	}
	if (abs(tmp_P.z() - last_P.z()) > 1) {
		// ROS_INFO(" big z translation");
		// return true;
	}
	Matrix3d	tmp_R	= Rs[WINDOW_SIZE];
	Matrix3d	delta_R = tmp_R.transpose() * last_R;
	Quaterniond delta_Q(delta_R);
	double		delta_angle;
	delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
	if (delta_angle > 50) {
		ROS_INFO(" big delta_angle ");
		// return true;
	}
	return false;
}



void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
	T					= Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = Rs[frame_count];
	T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
	T					= Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = Rs[index];
	T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame() {
	// printf("predict pts in next frame\n");
	if (frame_count < 2)
		return;
	// predict next pose. Assume constant velocity motion
	Eigen::Matrix4d curT, prevT, nextT;
	getPoseInWorldFrame(curT);
	getPoseInWorldFrame(frame_count - 1, prevT);
	nextT = curT * (prevT.inverse() * curT);
	map<int, Eigen::Vector3d> predictPts;

	for (auto &it_per_id : f_manager.feature) {
		if (it_per_id.estimated_depth > 0) {
			int firstIndex = it_per_id.start_frame;
			int lastIndex  = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
			// printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
			if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count) {
				double	 depth		 = it_per_id.estimated_depth;
				Vector3d pts_j		 = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
				Vector3d pts_w		 = Rs[firstIndex] * pts_j + Ps[firstIndex];
				Vector3d pts_local	 = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
				Vector3d pts_cam	 = ric[0].transpose() * (pts_local - tic[0]);
				int		 ptsIndex	 = it_per_id.feature_id;
				predictPts[ptsIndex] = pts_cam;
			}
		}
	}
	featureTracker.setPrediction(predictPts);
	// printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici, Matrix3d &Rj,
									Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, double depth, Vector3d &uvi,
									Vector3d &uvj) {
	Vector3d pts_w	  = Ri * (rici * (depth * uvi) + tici) + Pi;
	Vector3d pts_cj	  = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
	Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
	double	 rx		  = residual.x();
	double	 ry		  = residual.y();
	return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex) {
	// return;
	int feature_index = -1;
	for (auto &it_per_id : f_manager.feature) {
		double err		   = 0;
		int	   errCnt	   = 0;
		it_per_id.used_num = it_per_id.feature_per_frame.size();
		if (it_per_id.used_num < 4)
			continue;
		feature_index++;
		int		 imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
		Vector3d pts_i = it_per_id.feature_per_frame[0].point;
		double	 depth = it_per_id.estimated_depth;
		for (auto &it_per_frame : it_per_id.feature_per_frame) {
			imu_j++;
			if (imu_i != imu_j) {
				Vector3d pts_j	 = it_per_frame.point;
				double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j], ric[0],
													 tic[0], depth, pts_i, pts_j);
				err += tmp_error;
				errCnt++;
				// printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
			}
			// need to rewrite projecton factor.........
			if (STEREO && it_per_frame.is_stereo) {

				Vector3d pts_j_right = it_per_frame.pointRight;
				if (imu_i != imu_j) {
					double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
														 ric[1], tic[1], depth, pts_i, pts_j_right);
					err += tmp_error;
					errCnt++;
					// printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
				} else {
					double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], Rs[imu_j], Ps[imu_j],
														 ric[1], tic[1], depth, pts_i, pts_j_right);
					err += tmp_error;
					errCnt++;
					// printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
				}
			}
		}
		double ave_err = err / errCnt;
		if (ave_err * FOCAL_LENGTH > 3)
			removeIndex.insert(it_per_id.feature_id);
	}
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity) {
	double dt				 = t - latest_time;
	latest_time				 = t;
	Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
	Eigen::Vector3d un_gyr	 = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
	latest_Q				 = latest_Q * Utility::deltaQ(un_gyr * dt);
	Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
	Eigen::Vector3d un_acc	 = 0.5 * (un_acc_0 + un_acc_1);
	latest_P				 = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
	latest_V				 = latest_V + dt * un_acc;
	latest_acc_0			 = linear_acceleration;
	latest_gyr_0			 = angular_velocity;
}

void Estimator::updateLatestStates() {
	mPropagate.lock();
	latest_time			= Headers[frame_count] + td;
	latest_P			= Ps[frame_count];
	latest_Q			= Rs[frame_count];
	latest_V			= Vs[frame_count];
	latest_Ba			= Bas[frame_count];
	latest_Bg			= Bgs[frame_count];
	latest_Mw			= Mw[frame_count];
	latest_Bm			= Bms[frame_count];
	latest_mag_measure	= mag_measure[frame_count];
	latest_baro_measure = baro_measure[frame_count];
	latest_Bb			= Bbs[frame_count];
	latest_acc_0		= acc_0;
	latest_gyr_0		= gyr_0;
	mBuf.lock();
	queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
	queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
	mBuf.unlock();
	while (!tmp_accBuf.empty()) {
		double			t	= tmp_accBuf.front().first;
		Eigen::Vector3d acc = tmp_accBuf.front().second;
		Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
		fastPredictIMU(t, acc, gyr);
		tmp_accBuf.pop();
		tmp_gyrBuf.pop();
	}
	mPropagate.unlock();
}
