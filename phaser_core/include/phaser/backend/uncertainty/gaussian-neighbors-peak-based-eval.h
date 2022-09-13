#ifndef PHASER_BACKEND_UNCERTAINTY_GAUSSIAN_NEIGHBORS_PEAK_BASED_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_GAUSSIAN_NEIGHBORS_PEAK_BASED_EVAL_H_

#include <vector>

#include "phaser/backend/uncertainty/neighbors-eval.h"
#include "phaser/distribution/gaussian.h"

namespace phaser_core {

class GaussianNeighborsPeakBasedEval : public NeighborsEval {
 public:
  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound,
      const std::vector<double>& normalized_corr, int32_t index) const override;

 private:
  common::Gaussian fitGaussianDistribution(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, const std::vector<double>& norm_corr,
      int32_t index) const;

  void retrievePeakNeighbors(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, int32_t index,
      const std::vector<double>& norm_corr, Eigen::ArrayXXd* samples,
      Eigen::VectorXd* weights) const;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_GAUSSIAN_NEIGHBORS_PEAK_BASED_EVAL_H_
