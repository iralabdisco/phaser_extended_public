#include "phaser/backend/uncertainty/bingham-neighbors-peak-based-eval.h"

#include <algorithm>
#include <glog/logging.h>

#include "phaser/common/core-gflags.h"
#include "phaser/common/grid-utils.h"
#include "phaser/common/rotation-utils.h"

namespace phaser_core {

common::BaseDistributionPtr
BinghamNeighborsPeakBasedEval::evaluatePeakBasedCorrelation(
    const uint32_t bw, const std::vector<double>& normalized_corr,
    int32_t index) const {
  common::BinghamPtr bingham = std::make_shared<common::Bingham>(
      fitRotationalBinghamDistribution(bw, normalized_corr, index));
  return bingham;
}

common::Bingham BinghamNeighborsPeakBasedEval::fitRotationalBinghamDistribution(
    const uint32_t bw, const std::vector<double>& norm_corr,
    int32_t index) const {
  Eigen::MatrixXd samples;
  Eigen::RowVectorXd weights;
  retrievePeakNeighbors(bw, norm_corr, index, &samples, &weights);
  return common::Bingham::fit(samples, weights);
}

void BinghamNeighborsPeakBasedEval::retrievePeakNeighbors(
    const uint32_t bw, const std::vector<double>& norm_corr,
    const int32_t index, Eigen::MatrixXd* samples,
    Eigen::RowVectorXd* weights) const {
  CHECK_NOTNULL(samples);
  CHECK_NOTNULL(weights);

  std::vector<int32_t> neighbors_indexes;
  common::GridUtils::getNeighbors(
      index, bw * 2, FLAGS_bingham_samples_radius, &neighbors_indexes);

  // getNeighbors returns only the neighbors, we also need the max itself for
  // Bingham fit
  neighbors_indexes.push_back(index);

  samples->resize(4, neighbors_indexes.size());
  samples->setZero();

  weights->resize(neighbors_indexes.size());
  weights->setZero();

  std::size_t k = 0u;

  for (auto neighbor_index : neighbors_indexes) {
    std::array<double, 3> zyz =
        common::RotationUtils::GetZYZFromIndex(neighbor_index, bw);
    Eigen::Quaterniond q = common::RotationUtils::ConvertZYZtoQuaternion(zyz);
    (*samples)(0, k) = q.w();
    (*samples)(1, k) = q.x();
    (*samples)(2, k) = q.y();
    (*samples)(3, k) = q.z();
    (*weights)(k) = norm_corr.at(neighbor_index);
    ++k;
  }
  if (k == 1) {
    return;
  }

  // TODO(fdila) all weights need to be > 0, don't know if this is a good
  // workaround
  for (int i = 0; i < neighbors_indexes.size(); ++i) {
    if ((*weights)(i) < 0) {
      (*weights)(i) = 0;
    }
  }

  const double weight_sum = weights->array().sum();
  CHECK_GT(weight_sum, 0);
  (*weights) = weights->array() / weight_sum;
}

}  // namespace phaser_core
