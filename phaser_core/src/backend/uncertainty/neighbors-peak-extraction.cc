#include "phaser/backend/uncertainty/neighbors-peak-extraction.h"

#include <algorithm>
#include <glog/logging.h>
#include <omp.h>
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
      max_peaks_number_(0) {}

NeighborsPeakExtraction::NeighborsPeakExtraction(
    int32_t grid_size, int32_t neighbor_radius, int32_t max_peaks_number)
    : manager_("neighbors-peaks"),
      grid_size_(grid_size),
      peaks_discard_threshold_(FLAGS_peaks_discard_threshold),
      neighbors_radius_(neighbor_radius),
      max_peaks_number_(max_peaks_number) {
  CHECK_GT(neighbors_radius_, 0);
  CHECK_GT(max_peaks_number_, 0);
}

void NeighborsPeakExtraction::extractPeaks(
    const std::vector<double>& corr, std::set<uint32_t>* peaks) {
  VLOG(1) << "Extracting peaks...";

  peaks->clear();

  auto max = std::max_element(corr.cbegin(), corr.cend());
  CHECK(max != corr.cend());

  double discard_threshold = *max * peaks_discard_threshold_;

  int32_t corr_size = corr.size();

#pragma omp parallel for num_threads(8)
  for (int32_t i = 0; i < corr_size; i++) {
    int32_t current_index = i;
    if (corr.at(current_index) > discard_threshold) {
      std::vector<int32_t> neighbors;
      bool is_max = true;
      common::GridUtils::getNeighbors(
          current_index, grid_size_, neighbors_radius_, &neighbors);
      for (auto neighbor : neighbors) {
        if (neighbor >= corr_size)
          continue;
        if (corr.at(current_index) < corr.at(neighbor)) {
          is_max = false;
          break;
        }
      }
      if (is_max) {
        uint32_t uint_i = (uint32_t)current_index;
#pragma omp critical
        peaks->insert(uint_i);
      }
      is_max = true;
    }
  }
  VLOG(2) << "Found " << peaks->size() << " peaks";
  return;
}

void NeighborsPeakExtraction::getMaxPeaks(
    const std::set<uint32_t>* peaks, const std::vector<double>* corr,
    std::vector<uint32_t>* max_peaks) {
  std::vector<std::pair<double, int32_t>> peaks_with_idx;

  for (auto peak : *peaks) {
    peaks_with_idx.push_back(std::make_pair(corr->at(peak), peak));
  }

  // sort descending based on the correlation
  std::sort(peaks_with_idx.rbegin(), peaks_with_idx.rend());
  max_peaks->clear();
  
  if (peaks->size() < max_peaks_number_) {
    for (int i = 0; i < peaks->size(); i++) {
      max_peaks->push_back(peaks_with_idx.at(i).second);
    }
    return;
  }

  for (int i = 0; i < max_peaks_number_; i++) {
    max_peaks->push_back(peaks_with_idx.at(i).second);
  }

  return;
}

}  // namespace phaser_core
