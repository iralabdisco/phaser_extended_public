#ifndef PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_TRANSFORMED_H_
#define PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_TRANSFORMED_H_

#include <vector>

#include "phaser/backend/registration/sph-registration.h"

namespace phaser_core {

class SphRegistrationMockTransformed : public SphRegistration {
 public:
  ~SphRegistrationMockTransformed() = default;
  std::vector<model::RegistrationResult> registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

 private:
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_TRANSFORMED_H_
