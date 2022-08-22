#include <fstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/common/core-gflags.h"
#include "phaser/controller/cloud-controller.h"
#include "phaser/distribution/bingham.h"
#include "phaser/distribution/gaussian.h"

DEFINE_string(
    registration_algorithm, "sph-opt",
    "Defines the registration algorithm to use.");
DEFINE_string(target_cloud, "", "Defines the path to the target cloud.");
DEFINE_string(source_cloud, "", "Defines the path to the source cloud.");
DEFINE_string(reg_cloud, "", "Defines the path to the registered cloud.");

static model::PointCloudPtr readPointCloud(const std::string& path_to_ply) {
  CHECK(!path_to_ply.empty());
  LOG(INFO) << "Reading point cloud from " << path_to_ply;
  model::PointCloudPtr cloud = std::make_shared<model::PointCloud>(path_to_ply);
  return cloud;
}

static void writePointCloud(
    const std::string& reg_file, model::PointCloudPtr cloud) {
  CHECK(!reg_file.empty());
  CHECK_NOTNULL(cloud);
  pcl::io::savePLYFileASCII(reg_file, *cloud->getRawCloud());
  LOG(INFO) << "Wrote registered cloud to " << reg_file;
}

static void registerCloud(
    const std::string& target, const std::string& source,
    const std::string& reg_cloud) {
  model::PointCloudPtr target_cloud = readPointCloud(target);
  model::PointCloudPtr source_cloud = readPointCloud(source);
  CHECK_NOTNULL(target_cloud);
  CHECK_NOTNULL(source_cloud);
  if (phaser_core::FLAGS_save_registered_clouds) {
    CHECK(!reg_cloud.empty());
  }
  auto ctrl = std::make_unique<phaser_core::CloudController>(
      FLAGS_registration_algorithm.c_str());
  std::vector<model::RegistrationResult> results =
      ctrl->registerPointCloud(target_cloud, source_cloud);

  std::ofstream results_csv;
  results_csv.open(phaser_core::FLAGS_result_folder + "results.csv");
  results_csv << "peak_n x_r y_r z_r x_t y_t z_t" << std::endl;
  for (auto result : results) {
    LOG(INFO) << "Registration number " << result.getPeakIndex();
    LOG(INFO) << "Registration result dual quaternion: "
              << result.getStateAsVec().transpose();
    LOG(INFO) << "Registration rotation: " << result.getRotation().transpose();
    LOG(INFO) << "Registration translation: "
              << result.getTranslation().transpose();
    results_csv << result.getPeakIndex() << " "
                << result.getRotation().transpose()(0) << " "
                << result.getRotation().transpose()(1) << " "
                << result.getRotation().transpose()(2) << " "
                << result.getTranslation().transpose()(0) << " "
                << result.getTranslation().transpose()(1) << " "
                << result.getTranslation().transpose()(2) << std::endl;
    if (phaser_core::FLAGS_dump_covariances) {
      std::ofstream b_uncertainty_csv;
      b_uncertainty_csv.open(
          phaser_core::FLAGS_result_folder + "bingham_cov" +
          std::to_string(result.getPeakIndex()) + ".csv");
      b_uncertainty_csv << std::static_pointer_cast<common::Bingham>(
                               result.getRotUncertaintyEstimate())
                               ->gaussianCovariance(false)
                        << std::endl;
      std::ofstream g_uncertainty_csv;
      g_uncertainty_csv.open(
          phaser_core::FLAGS_result_folder + "gaussian_cov" +
          std::to_string(result.getPeakIndex()) + ".csv");
      g_uncertainty_csv << std::static_pointer_cast<common::Gaussian>(
                               result.getPosUncertaintyEstimate())
                               ->getCov()
                        << std::endl;
    }
    // LOG(INFO) << "Bingham M: "
    //           << std::static_pointer_cast<common::Bingham>(
    //                  result.getRotUncertaintyEstimate())
    //                  ->getM()
    //                  .transpose();
    // LOG(INFO) << "Bingham Z: "
    //           << std::static_pointer_cast<common::Bingham>(
    //                  result.getRotUncertaintyEstimate())
    //                  ->getZ()
    //                  .transpose();
    // LOG(INFO) << "Bingham F: "
    //           << std::static_pointer_cast<common::Bingham>(
    //                  result.getRotUncertaintyEstimate())
    //                  ->getF();
    LOG(INFO) << "Rotation covariance: "
              << std::static_pointer_cast<common::Bingham>(
                     result.getRotUncertaintyEstimate())
                     ->gaussianCovariance(false);
    LOG(INFO) << "Translation covariance: "
              << std::static_pointer_cast<common::Gaussian>(
                     result.getPosUncertaintyEstimate())
                     ->getCov();
    std::string reg_cloud_n = phaser_core::FLAGS_result_folder + reg_cloud +
                              std::to_string(result.getPeakIndex()) + ".ply";
    if (phaser_core::FLAGS_save_registered_clouds) {
      LOG(INFO) << "Writing point cloud to: " << reg_cloud_n;
      writePointCloud(reg_cloud_n, result.getRegisteredCloud());
    }
  }
  results_csv.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "phaser_core_driver");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  VLOG(1) << "=== PHASER CORE DRIVER =====================";

  registerCloud(FLAGS_target_cloud, FLAGS_source_cloud, FLAGS_reg_cloud);
  return 0;
}
