#include "phaser/backend/registration/mock/sph-registration-mock-transformed.h"
#include "phaser/common/rotation-utils.h"

#include <glog/logging.h>
#include <vector>

namespace phaser_core {

std::vector<model::RegistrationResult>
SphRegistrationMockTransformed::registerPointCloud(
    model::PointCloudPtr cloud_prev, model::PointCloudPtr) {
  cloud_prev->initialize_kd_tree();

  std::vector<model::RegistrationResult> results;
  model::RegistrationResult result = model::RegistrationResult();
  results.push_back(result);
  return results;
}

}  // namespace phaser_core
