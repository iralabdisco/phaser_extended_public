#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/controller/cloud-controller.h"
#include "phaser/distribution/bingham.h"
#include "phaser/distribution/gaussian.h"

DEFINE_string(target_cloud, "", "Defines the path to the target cloud.");
DEFINE_string(source_cloud, "", "Defines the path to the source cloud.");
DEFINE_string(reg_cloud, "", "Defines the path to the registered cloud.");
DEFINE_bool(
    phaser_core_multiple_peaks, false,
    "Return multiple estimates for registration if true.");

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
  CHECK(!reg_cloud.empty());

  auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");
  model::RegistrationResult result =
      ctrl->registerPointCloud(target_cloud, source_cloud);

  LOG(INFO) << "Registration result dual quaternion: "
            << result.getStateAsVec().transpose();
  LOG(INFO) << "Registration rotation: " << result.getRotation().transpose();
  LOG(INFO) << "Registration translation: "
            << result.getTranslation().transpose();
  LOG(INFO) << "Translation gaussian mean: "
            << std::static_pointer_cast<common::Gaussian>(
                   result.getPosUncertaintyEstimate())
                   ->getMean();
  LOG(INFO) << "Translation gaussian cov: "
            << std::static_pointer_cast<common::Gaussian>(
                   result.getPosUncertaintyEstimate())
                   ->getCov();
  LOG(INFO) << "Rotation bingham M: "
            << std::static_pointer_cast<common::Bingham>(
                   result.getRotUncertaintyEstimate())
                   ->getM();
  LOG(INFO) << "Rotation bingham Z: "
            << std::static_pointer_cast<common::Bingham>(
                   result.getRotUncertaintyEstimate())
                   ->getZ();
  writePointCloud(reg_cloud, result.getRegisteredCloud());
}

static void registerCloudMultiplePeaks(
    const std::string& target, const std::string& source,
    const std::string& reg_cloud) {
  model::PointCloudPtr target_cloud = readPointCloud(target);
  model::PointCloudPtr source_cloud = readPointCloud(source);
  CHECK_NOTNULL(target_cloud);
  CHECK_NOTNULL(source_cloud);
  CHECK(!reg_cloud.empty());

  auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");
  std::vector<model::RegistrationResult> results =
      ctrl->registerPointCloudMultiplePeaks(target_cloud, source_cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "phaser_core_driver");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  VLOG(1) << "=== PHASER CORE DRIVER =====================";

  if (FLAGS_phaser_core_multiple_peaks) {
    registerCloudMultiplePeaks(
        FLAGS_target_cloud, FLAGS_source_cloud, FLAGS_reg_cloud);
  } else {
    registerCloud(FLAGS_target_cloud, FLAGS_source_cloud, FLAGS_reg_cloud);
  }
  return 0;
}
