#ifndef PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_TRANSLATED_H_
#define PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_TRANSLATED_H_

#include <vector>

#include "phaser/backend/registration/sph-registration.h"

namespace phaser_core {

class SphRegistrationMockTranslated : public SphRegistration {
 public:
  SphRegistrationMockTranslated();
  ~SphRegistrationMockTranslated() = default;
  std::vector<model::RegistrationResult> registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

  void setRandomTranslation(
      const double mock_trans_x, const double mock_trans_y,
      const double mock_trans_z);

 private:
  model::PointCloud pertubPointCloud(
      model::PointCloud& cloud, const float x, const float y,  // NOLINT
      const float z) const;

  std::vector<model::FunctionValue> pertubFunctionValues(        // NOLINT
      std::vector<model::FunctionValue>& values, const float x,  // NOLINT
      const float y, const float z) const;

  double mock_trans_x_;
  double mock_trans_y_;
  double mock_trans_z_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_REGISTRATION_MOCK_SPH_REGISTRATION_MOCK_TRANSLATED_H_
