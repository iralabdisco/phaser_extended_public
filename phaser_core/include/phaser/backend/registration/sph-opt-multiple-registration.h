#ifndef PHASER_BACKEND_REGISTRATION_SPH_OPT_MULTIPLE_REGISTRATION_H_
#define PHASER_BACKEND_REGISTRATION_SPH_OPT_MULTIPLE_REGISTRATION_H_

#include <array>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/backend/registration/base-registration.h"
#include "phaser/backend/uncertainty/base-eval.h"
#include "phaser/backend/uncertainty/phase-correlation-eval.h"
#include "phaser/common/spherical-sampler.h"
#include "phaser/common/thread-pool.h"

namespace phaser_core {

class SphOptMultipleRegistration : public BaseRegistration {
 public:
  SphOptMultipleRegistration();
  virtual ~SphOptMultipleRegistration();

  std::vector<model::RegistrationResult> registerPointCloud(
      model::PointCloudPtr cloud_prev, model::PointCloudPtr cloud_cur) override;

  void getStatistics(common::StatisticsManager* manager) const
      noexcept override;

  std::vector<model::RegistrationResult> estimateMultipleRotation(
      model::PointCloudPtr cloud_cur, std::vector<double> corr,
      std::vector<uint32_t> peaks);
  model::RegistrationResult estimateRotation(
      model::PointCloudPtr cloud_cur, std::vector<double> corr, int32_t index);

  std::vector<model::RegistrationResult> estimateMultipleTranslation(
      model::PointCloudPtr cloud_prev,
      std::vector<model::RegistrationResult>* results);
  model::RegistrationResult estimateTranslation(
      model::RegistrationResult* result, std::vector<double> corr,
      int32_t index);

  void setBandwith(const int bandwith);

  BaseEval& getRotEvaluation();
  BaseEval& getPosEvaluation();

 protected:
  std::vector<SphericalCorrelation> correlatePointcloud(
      model::PointCloudPtr target, model::PointCloudPtr source);

  const uint32_t bandwidth_;
  common::SphericalSampler sampler_;
  std::vector<model::FunctionValue> f_values_;
  std::vector<model::FunctionValue> h_values_;
  phaser_core::PhaseAligner aligner_;
  PhaseCorrelationEvalPtr correlation_eval_;
  common::ThreadPool th_pool_;
};

using SphOptMultipleRegistrationPtr =
    std::unique_ptr<SphOptMultipleRegistration>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_REGISTRATION_SPH_OPT_MULTIPLE_REGISTRATION_H_
