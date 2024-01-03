#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>

class MagFactor {
  public:
	MagFactor(const Eigen::Vector3d &_measurement, const Eigen::Matrix<double, 3, 3> &_sqrt_info)
		: measurement(_measurement), sqrt_info(_sqrt_info) {
	}

	template <typename T>
	bool operator()(const T *const param_Pose, const T *const para_m, T *residual_ptr) const {
		Eigen::Map<const Eigen::Quaternion<T>>	 q(param_Pose + 3); // P q
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> Mw(para_m);
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> Bms(para_m + 3);
		Eigen::Map<Eigen::Matrix<T, 3, 1>>		 residual(residual_ptr);

		residual = q * (measurement.cast<T>() - Bms) - Mw;
		residual = sqrt_info * residual;
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d			 &_measurement,
									   const Eigen::Matrix<double, 3, 3> &_sqrt_info) {
		return (new ceres::AutoDiffCostFunction<MagFactor, 3, 7, 6>(new MagFactor(_measurement, _sqrt_info)));
	}

  private:
	const Eigen::Vector3d			  measurement;
	const Eigen::Matrix<double, 3, 3> sqrt_info;
};

class MagWorldFactor {
  public:
	MagWorldFactor(const Eigen::Matrix<double, 3, 3> &_sqrt_info) : sqrt_info(_sqrt_info) {
	}

	template <typename T>
	bool operator()(const T *const param_mag_i, const T *const param_mag_j, T *residual_ptr) const {
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> Mw_i(param_mag_i);
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> Mw_j(param_mag_j);
		Eigen::Map<Eigen::Matrix<T, 3, 1>>		 residual(residual_ptr);

		residual = sqrt_info * (Mw_i - Mw_j);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Matrix<double, 3, 3> _sqrt_info) {
		return (new ceres::AutoDiffCostFunction<MagWorldFactor, 3, 6, 6>(new MagWorldFactor(_sqrt_info)));
	}

  private:
	const Eigen::Matrix<double, 3, 3> sqrt_info;
};

class MagBiasFactor {
  public:
	MagBiasFactor(const Eigen::Matrix<double, 3, 3> &_sqrt_info) : sqrt_info(_sqrt_info) {
	}

	template <typename T>
	bool operator()(const T *const param_mag_i, const T *const param_mag_j, T *residual_ptr) const {
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> Bms_i(param_mag_i + 3);
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> Bms_j(param_mag_j + 3);
		Eigen::Map<Eigen::Matrix<T, 3, 1>>		 residual(residual_ptr);

		residual = sqrt_info * (Bms_i - Bms_j);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Matrix<double, 3, 3> _sqrt_info) {
		return (new ceres::AutoDiffCostFunction<MagBiasFactor, 3, 6, 6>(new MagBiasFactor(_sqrt_info)));
	}

  private:
	const Eigen::Matrix<double, 3, 3> sqrt_info;
};