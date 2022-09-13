#ifndef PHASER_BACKEND_UNCERTAINTY_BINGHAM_NEIGHBORS_PEAK_BASED_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_BINGHAM_NEIGHBORS_PEAK_BASED_EVAL_H_

#include <set>
#include <vector>

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/uncertainty/neighbors-eval.h"
#include "phaser/common/core-gflags.h"
#include "phaser/distribution/bingham.h"

namespace phaser_core {

class BinghamNeighborsPeakBasedEval : public NeighborsEval {
 public:
  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t bw, const std::vector<double>& normalized_corr,
      int32_t index) const override;

 private:
  common::Bingham fitRotationalBinghamDistribution(
      const uint32_t bw, const std::vector<double>& norm_corr,
      int32_t index) const;

  void retrievePeakNeighbors(
      const uint32_t bw, const std::vector<double>& norm_corr,
      const int32_t index, Eigen::MatrixXd* samples,
      Eigen::RowVectorXd* weights) const;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_BINGHAM_NEIGHBORS_PEAK_BASED_EVAL_H_
