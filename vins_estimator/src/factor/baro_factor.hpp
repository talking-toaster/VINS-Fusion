#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>

class BaroFactor {
  public:
	BaroFactor(const double &_measurement, const double &_sqrt_info)
		: measurement(_measurement), sqrt_info(_sqrt_info) {
	}

	template <typename T>
	bool operator()(const T *const para_Pose, const T *const bias_baro, T *residual_ptr) const {
		const T pose_z	 = para_Pose[2];
		const T residual = pose_z - (T)measurement + (*bias_baro);
		*residual_ptr	 = sqrt_info * residual;
		return true;
	}

	static ceres::CostFunction *Create(const double &_measurement, const double &_sqrt_info) {
		return (new ceres::AutoDiffCostFunction<BaroFactor, 1, 7, 1>(new BaroFactor(_measurement, _sqrt_info)));
	}

  private:
	const double measurement;
	const double sqrt_info;
};

class BaroBiasFactor {
  public:
	BaroBiasFactor(const double &_sqrt_info) : sqrt_info(_sqrt_info) {
	}

	template <typename T>
	bool operator()(const T *const bias_baro_i, const T *const bias_baro_j, T *residual_ptr) const {
		const T residual = (*bias_baro_i) - (*bias_baro_j);
		*residual_ptr	 = sqrt_info * residual;
		return true;
	}

	static ceres::CostFunction *Create(const double &_sqrt_info) {
		return (new ceres::AutoDiffCostFunction<BaroBiasFactor, 1, 1, 1>(new BaroBiasFactor(_sqrt_info)));
	}

  private:
	const double sqrt_info;
};