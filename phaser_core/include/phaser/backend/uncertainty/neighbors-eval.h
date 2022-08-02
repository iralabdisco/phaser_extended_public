#ifndef PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_EVAL_H_

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "phaser/backend/uncertainty/base-eval.h"
#include "phaser/backend/uncertainty/neighbors-peak-extraction.h"
#include "phaser/common/statistics-manager.h"

namespace phaser_core {

class NeighborsEval : public BaseEval {
 public:
  NeighborsEval();

  common::BaseDistributionPtr evaluateCorrelationFromTranslation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound,
      const std::vector<double>& corr) override;
  common::BaseDistributionPtr evaluateCorrelationFromRotation(
      const uint32_t bw, const std::vector<double>& corr) override;

  virtual common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t bw, const std::vector<double>& normalized_corr,
      int32_t index) const;
  virtual common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound,
      const std::vector<double>& normalized_corr, int32_t index) const;

  NeighborsPeakExtraction& getPeakExtraction();

  void normalizeCorrelationVector(
      const std::vector<double>& corr, std::vector<double>* n_corr_ds);

  common::StatisticsManager manager_;
  NeighborsPeakExtraction peak_extraction_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_EVAL_H_
