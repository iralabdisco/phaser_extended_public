#include "phaser/backend/uncertainty/neighbors-peak-extraction.h"

#include <algorithm>
#include <glog/logging.h>
#include <set>
#include <vector>

#include "phaser/common/core-gflags.h"
#include "phaser/common/grid-utils.h"

namespace phaser_core {

NeighborsPeakExtraction::NeighborsPeakExtraction()
    : manager_("neighbors-peaks"),
      grid_size_(0),
      peaks_discard_threshold_(FLAGS_peaks_discard_threshold),
      neighbors_radius_(0),
      max_peaks_number_(FLAGS_max_peaks_number) {}

NeighborsPeakExtraction::NeighborsPeakExtraction(
    int32_t grid_size, int32_t neighbor_radius)
    : manager_("neighbors-peaks"),
      grid_size_(grid_size),
      peaks_discard_threshold_(FLAGS_peaks_discard_threshold),
      neighbors_radius_(neighbor_radius),
      max_peaks_number_(FLAGS_max_peaks_number) {
  CHECK_GT(neighbors_radius_, 0);
  CHECK_GT(max_peaks_number_, 0);
}

void NeighborsPeakExtraction::extractPeaks(
    const std::vector<double>& corr, std::set<uint32_t>* peaks) {
  VLOG(1) << "Extracting peaks...";

  int32_t corr_size = corr.size();
  VLOG(1) << "Correlation size: " << corr_size;
  VLOG(1) << "Using grid size: " << grid_size_;

  std::vector<int32_t> neighbors;
  bool is_max = true;

  peaks->clear();

  for (int32_t i = 0; i < corr_size; i++) {
    if (corr.at(i) > peaks_discard_threshold_) {
      common::GridUtils::getNeighbors(
          i, grid_size_, neighbors_radius_, &neighbors);
      for (auto neighbor : neighbors) {
        if (corr.at(i) < corr.at(neighbor)) {
          is_max = false;
          break;
        }
      }
      if (is_max) {
        uint32_t uint_i = (uint32_t)i;
        peaks->insert(uint_i);
      }
      is_max = true;
    }
  }
  VLOG(2) << "Found " << peaks->size() << " peaks";
  return;
}

void NeighborsPeakExtraction::getMaxPeaks(
    const std::set<uint32_t>* peaks, const std::vector<double>* norm_corr,
    std::set<uint32_t>* max_peaks) {
  std::vector<std::pair<int32_t, double>> peaks_with_idx;

  if (peaks->size() <= max_peaks_number_) {
    for (auto peak : *peaks)
      max_peaks->insert(peak);
    return;
  }

  for (auto peak : *peaks) {
    peaks_with_idx.push_back(std::make_pair(peak, norm_corr->at(peak)));
  }

  // sort descending based on the correlation
  std::sort(
      peaks_with_idx.begin(), peaks_with_idx.end(),
      [](auto& left, auto& right) { return left.second > right.second; });

  max_peaks->clear();

  for (int i = 0; i < max_peaks_number_; i++) {
    max_peaks->insert(peaks_with_idx.at(i).first);
  }

  return;
}

}  // namespace phaser_core
