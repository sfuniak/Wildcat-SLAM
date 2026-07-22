#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <memory>
#include <stdexcept>

#include "odometry/lidar_odometry.h"
#include "sensor/imu_resampler.h"

namespace py = pybind11;

class WildcatProcessor {
 public:
  explicit WildcatProcessor(int imu_rate, int solver_threads, const std::string &sparse_backend)
      : odometry_(make_odometry(solver_threads, sparse_backend)), imu_resampler_(imu_rate) {}

  void add_imu(double timestamp,
               py::array_t<double, py::array::c_style | py::array::forcecast> linear_acceleration,
               py::array_t<double, py::array::c_style | py::array::forcecast> angular_velocity) {
    if (linear_acceleration.size() != 3 || angular_velocity.size() != 3) {
      throw std::invalid_argument("IMU vectors must contain exactly three values");
    }
    const auto acc = linear_acceleration.unchecked<1>();
    const auto gyro = angular_velocity.unchecked<1>();
    ImuData data{timestamp,
                 Vector3d(acc(0), acc(1), acc(2)),
                 Vector3d(gyro(0), gyro(1), gyro(2))};
    imu_resampler_.AddImuData(data);
    if (auto resampled = imu_resampler_.AdvanceGetResampledImuData()) {
      py::gil_scoped_release release;
      odometry_->AddImuData(*resampled);
    }
  }

  void add_lidar(
      py::array_t<float, py::array::c_style | py::array::forcecast> xyz,
      py::array_t<float, py::array::c_style | py::array::forcecast> intensity,
      py::array_t<double, py::array::c_style | py::array::forcecast> timestamp,
      py::array_t<std::uint16_t, py::array::c_style | py::array::forcecast> ring) {
    if (xyz.ndim() != 2 || xyz.shape(1) != 3) {
      throw std::invalid_argument("xyz must have shape (N, 3)");
    }
    const auto count = xyz.shape(0);
    if (intensity.size() != count || timestamp.size() != count || ring.size() != count) {
      throw std::invalid_argument("All point arrays must have the same length");
    }

    auto cloud = std::make_shared<pcl::PointCloud<hilti_ros::Point>>();
    cloud->reserve(count);
    const auto coordinates = xyz.unchecked<2>();
    const auto intensities = intensity.unchecked<1>();
    const auto timestamps = timestamp.unchecked<1>();
    const auto rings = ring.unchecked<1>();
    for (py::ssize_t i = 0; i < count; ++i) {
      hilti_ros::Point point{};
      point.x = coordinates(i, 0);
      point.y = coordinates(i, 1);
      point.z = coordinates(i, 2);
      point.intensity = intensities(i);
      point.time = timestamps(i);
      point.ring = rings(i);
      cloud->push_back(point);
    }
    py::gil_scoped_release release;
    odometry_->AddLidarScan(cloud);
  }

  py::object latest_pose() const {
    const auto pose = odometry_->LatestPose();
    if (!pose) {
      return py::none();
    }
    py::dict result;
    result["timestamp"] = pose->timestamp;
    result["position"] = py::make_tuple(pose->position.x(), pose->position.y(), pose->position.z());
    result["orientation_xyzw"] = py::make_tuple(
        pose->orientation.x(), pose->orientation.y(), pose->orientation.z(), pose->orientation.w());
    return std::move(result);
  }

  py::object snapshot() const {
    const auto state = odometry_->Snapshot();
    if (!state) {
      return py::none();
    }

    py::array_t<double> position(3);
    py::array_t<double> orientation(4);
    auto position_data = position.mutable_unchecked<1>();
    auto orientation_data = orientation.mutable_unchecked<1>();
    for (py::ssize_t axis = 0; axis < 3; ++axis) {
      position_data(axis) = state->pose.position[axis];
    }
    orientation_data(0) = state->pose.orientation.x();
    orientation_data(1) = state->pose.orientation.y();
    orientation_data(2) = state->pose.orientation.z();
    orientation_data(3) = state->pose.orientation.w();

    const auto count = static_cast<py::ssize_t>(state->surfels.size());
    py::array_t<double> centers({count, py::ssize_t{3}});
    py::array_t<double> normals({count, py::ssize_t{3}});
    py::array_t<double> covariances({count, py::ssize_t{3}, py::ssize_t{3}});
    py::array_t<double> timestamps(count);
    py::array_t<std::uint8_t> fixed_window(count);
    auto center_data = centers.mutable_unchecked<2>();
    auto normal_data = normals.mutable_unchecked<2>();
    auto covariance_data = covariances.mutable_unchecked<3>();
    auto timestamp_data = timestamps.mutable_unchecked<1>();
    auto fixed_window_data = fixed_window.mutable_unchecked<1>();
    for (py::ssize_t index = 0; index < count; ++index) {
      const auto &surfel = state->surfels[index];
      timestamp_data(index) = surfel.timestamp;
      fixed_window_data(index) = surfel.fixed_window;
      for (py::ssize_t row = 0; row < 3; ++row) {
        center_data(index, row) = surfel.center[row];
        normal_data(index, row) = surfel.normal[row];
        for (py::ssize_t column = 0; column < 3; ++column) {
          covariance_data(index, row, column) = surfel.covariance(row, column);
        }
      }
    }

    py::dict surfels;
    surfels["centers"] = std::move(centers);
    surfels["normals"] = std::move(normals);
    surfels["covariances"] = std::move(covariances);
    surfels["timestamps"] = std::move(timestamps);
    surfels["fixed_window"] = std::move(fixed_window);

    py::dict result;
    result["timestamp"] = state->pose.timestamp;
    result["position"] = std::move(position);
    result["orientation_xyzw"] = std::move(orientation);
    result["surfels"] = std::move(surfels);
    return std::move(result);
  }

  int sweep_count() const { return odometry_->SweepCount(); }

  py::dict timing_averages() const {
    const auto averages = odometry_->GetTimingAverages();
    py::dict result;
    result["collect_scan_to_sweep"] = averages.stage_seconds[0];
    result["integrate_imu_poses"] = averages.stage_seconds[1];
    result["undistort_sweep"] = averages.stage_seconds[2];
    result["extract_surfels"] = averages.stage_seconds[3];
    result["match_surfels"] = averages.stage_seconds[4];
    result["build_residual_graph"] = averages.stage_seconds[5];
    result["ceres_solve"] = averages.stage_seconds[6];
    result["update_states"] = averages.stage_seconds[7];
    return result;
  }

 private:
  static std::shared_ptr<LidarOdometry> make_odometry(int solver_threads, const std::string &sparse_backend) {
    if (solver_threads < 1) {
      throw std::invalid_argument("solver_threads must be at least 1");
    }
    if (sparse_backend == "suite") {
      return std::make_shared<LidarOdometry>(solver_threads, ceres::SUITE_SPARSE);
    }
    if (sparse_backend == "eigen") {
      return std::make_shared<LidarOdometry>(solver_threads, ceres::EIGEN_SPARSE);
    }
    if (sparse_backend == "accelerate") {
      return std::make_shared<LidarOdometry>(solver_threads, ceres::ACCELERATE_SPARSE);
    }
    if (sparse_backend == "cuda") {
      return std::make_shared<LidarOdometry>(solver_threads, ceres::CUDA_SPARSE);
    }
    throw std::invalid_argument("sparse_backend must be one of: suite, eigen, accelerate, cuda");
  }

  std::shared_ptr<LidarOdometry> odometry_;
  ImuResampler imu_resampler_;
};

PYBIND11_MODULE(wildcat_slam, module) {
  module.doc() = "Python bindings for the ROS-independent Wildcat SLAM core";
  py::class_<WildcatProcessor>(module, "Processor")
      .def(py::init<int, int, const std::string &>(),
           py::arg("imu_rate") = 200,
           py::arg("solver_threads") = 1,
           py::arg("sparse_backend") = "suite")
      .def("add_imu", &WildcatProcessor::add_imu,
           py::arg("timestamp"), py::arg("linear_acceleration"), py::arg("angular_velocity"))
      .def("add_lidar", &WildcatProcessor::add_lidar,
           py::arg("xyz"), py::arg("intensity"), py::arg("timestamp"), py::arg("ring"))
      .def("snapshot", &WildcatProcessor::snapshot)
      .def_property_readonly("latest_pose", &WildcatProcessor::latest_pose)
      .def_property_readonly("timing_averages", &WildcatProcessor::timing_averages)
      .def_property_readonly("sweep_count", &WildcatProcessor::sweep_count);
}
