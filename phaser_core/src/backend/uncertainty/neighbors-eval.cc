#include "phaser/backend/uncertainty/neighbors-eval.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <numeric>
#include <sstream>
#include <vector>

#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/uncertainty/signal-analysis.h"
#include "phaser/distribution/bingham.h"
#include "phaser/distribution/gaussian.h"
#include "phaser/visualization/plotty-visualizer.h"

namespace phaser_core {

NeighborsEval::NeighborsEval() : manager_("neighbors-eval") {}

common::BaseDistributionPtr NeighborsEval::evaluateCorrelationFromTranslation(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const std::vector<double>& corr) {
  std::set<uint32_t> signals;
  std::vector<double> n_corr_ds;
  normalizeCorrelationVector(corr, &n_corr_ds);
  return evaluatePeakBasedCorrelation(
      n_voxels, discretize_lower_bound, discretize_upper_bound, n_corr_ds, 0);
}

common::BaseDistributionPtr NeighborsEval::evaluateCorrelationFromRotation(
    const uint32_t bw, const std::vector<double>& corr) {
  std::vector<double> n_corr_ds;
  normalizeCorrelationVector(corr, &n_corr_ds);
  return evaluatePeakBasedCorrelation(bw, n_corr_ds, 0);
}

void NeighborsEval::normalizeCorrelationVector(
    const std::vector<double>& corr, std::vector<double>* n_corr_ds) {
  n_corr_ds->clear();

  // Normalize correlation.
  auto max = std::max_element(corr.cbegin(), corr.cend());
  CHECK(max != corr.cend());

  std::transform(
      corr.begin(), corr.end(), std::back_inserter(*n_corr_ds),
      [&](const double val) { return val / *max; });

  VLOG(1) << "max is at: " << *max;

  return;
}

common::BaseDistributionPtr NeighborsEval::evaluatePeakBasedCorrelation(
    const uint32_t bw, const std::vector<double>& normalized_corr,
    int32_t index) const {
  LOG(FATAL)
      << "Peak based eval using bw is not implemented for this correlation.";
}

common::BaseDistributionPtr NeighborsEval::evaluatePeakBasedCorrelation(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound,
    const std::vector<double>& normalized_corr, int32_t index) const {
  LOG(FATAL) << "Peak based eval using voxels and bounds is not implemented "
                "for this correlation.";
}

NeighborsPeakExtraction& NeighborsEval::getPeakExtraction() {
  return peak_extraction_;
}

}  // namespace phaser_core
