#ifndef PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_PEAK_EXTRACTION_H_
#define PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_PEAK_EXTRACTION_H_

#include <set>
#include <vector>

#include "phaser/backend/uncertainty/base-peak-extraction.h"
#include "phaser/common/statistics-manager.h"

namespace phaser_core {

typedef struct grid_indexes {
  int32_t x;
  int32_t y;
  int32_t z;
} grid_indexes_t;

class NeighborsPeakExtraction : public BasePeakExtraction {
 public:
  NeighborsPeakExtraction();
  explicit NeighborsPeakExtraction(
      int32_t grid_size, int32_t neighbor_radius, int32_t max_peaks_number,
      bool rotation, bool translation);

  void extractPeaks(
      const std::vector<double>& corr, std::set<uint32_t>* peaks) override;

  void getMaxPeaks(
      const std::set<uint32_t>* peaks, const std::vector<double>* norm_corr,
      std::vector<uint32_t>* max_peaks);

  void getNeighbors(
      int32_t index, int32_t grid_size, int32_t neighbors_radius,
      std::vector<int32_t>* neighbors_indexes);

  int32_t getNeighborsRadius() const;
  int32_t& getNeighborsRadius();

  int32_t getMaxPeaksNumber() const;
  int32_t& getMaxPeaksNumber();

  common::StatisticsManager manager_;
  int32_t grid_size_;
  double peaks_discard_threshold_;
  int32_t neighbors_radius_;
  int32_t max_peaks_number_;
  bool rotation_;
  bool translation_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_NEIGHBORS_PEAK_EXTRACTION_H_
