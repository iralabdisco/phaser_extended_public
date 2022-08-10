#include "phaser/backend/uncertainty/gaussian-neighbors-peak-based-eval.h"

#include <glog/logging.h>

#include "phaser/common/core-gflags.h"
#include "phaser/common/grid-utils.h"
#include "phaser/common/signal-utils.h"
#include "phaser/common/translation-utils.h"
#include "phaser/distribution/gaussian.h"

namespace phaser_core {

common::BaseDistributionPtr
GaussianNeighborsPeakBasedEval::evaluatePeakBasedCorrelation(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound,
    const std::vector<double>& normalized_corr, int32_t index) const {
  common::GaussianPtr gaussian =
      std::make_shared<common::Gaussian>(fitGaussianDistribution(
          n_voxels, discretize_lower_bound, discretize_upper_bound,
          normalized_corr, index));
  return gaussian;
}

common::Gaussian GaussianNeighborsPeakBasedEval::fitGaussianDistribution(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const std::vector<double>& norm_corr,
    int32_t index) const {
  Eigen::ArrayXXd samples;
  Eigen::VectorXd weights;

  retrievePeakNeighbors(
      n_voxels, discretize_lower_bound, discretize_upper_bound, index,
      norm_corr, &samples, &weights);

  // Calculate mean and covariance.
  return common::Gaussian(samples, weights);
}

void GaussianNeighborsPeakBasedEval::retrievePeakNeighbors(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, int32_t index,
    const std::vector<double>& norm_corr, Eigen::ArrayXXd* samples,
    Eigen::VectorXd* weights) const {
  std::vector<int32_t> neighbors_indexes;
  common::GridUtils::getNeighbors(
      index, n_voxels, FLAGS_gaussian_peak_neighbors_radius,
      &neighbors_indexes);

  // getNeighbors returns only the neighbors, we also need the max itself for
  // Gaussian fit
  neighbors_indexes.push_back(index);

  samples->resize(4, neighbors_indexes.size());
  samples->setZero();

  weights->resize(neighbors_indexes.size());
  weights->setZero();

  // Extract translational estimates.
  uint32_t k = 0u;
  for (auto neighbor_index : neighbors_indexes) {
    std::array<uint32_t, 3> xyz =
        common::SignalUtils::Ind2Sub(neighbor_index, n_voxels, n_voxels);
    (*samples)(0, k) = common::TranslationUtils::ComputeTranslationFromIndex(
        static_cast<double>(xyz[0]), n_voxels, discretize_lower_bound,
        discretize_upper_bound);
    (*samples)(1, k) = common::TranslationUtils::ComputeTranslationFromIndex(
        static_cast<double>(xyz[1]), n_voxels, discretize_lower_bound,
        discretize_upper_bound);
    (*samples)(2, k) = common::TranslationUtils::ComputeTranslationFromIndex(
        static_cast<double>(xyz[2]), n_voxels, discretize_lower_bound,
        discretize_upper_bound);
    (*weights)(k) = norm_corr.at(neighbor_index);
    ++k;
  }

  // TODO(fdila) all weights need to be > 0, don't know if this is a good
  // workaround
  for (uint32_t i = 0; i < neighbors_indexes.size(); ++i) {
    if ((*weights)(i) < 0) {
      (*weights)(i) = 0;
    }
  }

  const double weight_sum = weights->array().sum();
  CHECK_GT(weight_sum, 0);
  (*weights) = weights->array() / weight_sum;

  return;
}

}  // namespace phaser_core
