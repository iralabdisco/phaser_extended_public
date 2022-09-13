#ifndef PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_CUTTED_H_
#define PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_CUTTED_H_

#include <memory>
#include <string>
#include <vector>

#include "phaser/backend/registration/sph-registration.h"

namespace phaser_core {

class SphRegistrationMockCutted : public SphRegistration {
 public:
  ~SphRegistrationMockCutted() = default;
  std::vector<model::RegistrationResult> registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

 private:
  model::PointCloudPtr cutPointCloud(
      common::PointCloud_tPtr& cloud, double min, double max,  // NOLINT
      std::string&& dim) const;                                // NOLINT
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_CUTTED_H_
