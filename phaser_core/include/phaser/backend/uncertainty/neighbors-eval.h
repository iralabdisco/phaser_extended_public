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

 private:
  void evaluateCorrelationVector(
      const std::vector<double>& corr, std::set<uint32_t>* signals,
      std::vector<double>* n_corr_ds);

  common::StatisticsManager manager_;
  NeighborsPeakExtraction peak_extractor_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_EVAL_H_
