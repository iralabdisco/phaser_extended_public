#include "phaser/backend/uncertainty/neighbors-peak-extraction.h"

#include <glog/logging.h>

DEFINE_int32(
    peak_extraction_neighbors, 2,
    "The number of neighbors used to find peaks.");
DEFINE_int32(
    max_peaks_number, 4, "Take only the first best max_peak_number peaks.");
DEFINE_double(
    peaks_discard_threshold, 0.5,
    "Do not evaluate peaks that are under threshold*maximum correlation.");

namespace phaser_core {
NeighborsPeakExtraction::NeighborsPeakExtraction(int32_t grid_size)
    : manager_("neighbors-peaks"),
      grid_size_(grid_size),
      peaks_discard_threshold_(FLAGS_peaks_discard_threshold),
      neighbors_radius_(FLAGS_peak_extraction_neighbors),
      max_peaks_number_(FLAGS_max_peaks_number) {
  CHECK_GT(neighbors_radius_, 0);
  CHECK_GT(max_peaks_number_, 0);
}

void NeighborsPeakExtraction::extractPeaks(
    const std::vector<double>& corr, std::set<uint32_t>* peaks) {
  int32_t corr_size = corr.size();
  std::vector<int32_t> neighbors;
  bool is_max = true;

  for (int32_t i = 0; i < corr_size; i++) {
    // TODO(fdila) add max to threshold
    if (corr.at(i) > peaks_discard_threshold_) {
      getNeighbors(i, grid_size_, neighbors);
      for (auto neighbor : neighbors) {
        if (corr.at(i) < corr.at(neighbor)) {
          is_max = false;
          break;
        }
      }
      if (is_max) {
        peaks->insert((uint32_t)i);
      }
      is_max = true;
    }
  }
  return;
}

void NeighborsPeakExtraction::getNeighbors(
    const int32_t index, const int32_t grid_size,
    std::vector<int32_t> neighbors_indexes) const {
  neighbors_indexes.clear();

  int32_t index_x = 0;
  int32_t index_y = 0;
  int32_t index_z = 0;

  grid_indexes_t grid_indexes = ind2sub(index, grid_size);

  for (int i = -this->neighbors_radius_; i < this->neighbors_radius_; i++) {
    index_x = grid_indexes.x + i;
    if (index_x < 0 || index_x > grid_size - 1)
      continue;
    for (int j = -this->neighbors_radius_; j < this->neighbors_radius_; j++) {
      index_y = grid_indexes.y + j;
      if (index_y < 0 || index_y > grid_size - 1)
        continue;
      for (int k = -this->neighbors_radius_; k < this->neighbors_radius_; k++) {
        index_z = grid_indexes.z + k;
        if (index_z < 0 || index_z > grid_size - 1)
          continue;

        if (index_x == grid_indexes.x && index_y == grid_indexes.y &&
            index_z == grid_indexes.z)
          continue;

        int32_t current_index = sub2ind(index_x, index_y, index_z, grid_size);
        neighbors_indexes.push_back(current_index);
      }
    }
  }
  return;
}

grid_indexes_t NeighborsPeakExtraction::ind2sub(
    const int32_t index, const int32_t grid_size) const {
  // %https://www.alecjacobson.com/weblog/?p=1425
  int32_t i = index % grid_size;
  int32_t j = ((index - i) / grid_size) % grid_size;
  int32_t k = ((index - i) / grid_size - j) / grid_size;
  grid_indexes_t grid_indexes;
  grid_indexes.x = i;
  grid_indexes.y = j;
  grid_indexes.z = k;
  return grid_indexes;
}

int32_t NeighborsPeakExtraction::sub2ind(
    const int32_t i, const int32_t j, const int32_t k,
    const int32_t grid_size) const {
  // %https://www.alecjacobson.com/weblog/?p=1425
  return i + grid_size * j + grid_size * grid_size * k;
}

}  // namespace phaser_core
