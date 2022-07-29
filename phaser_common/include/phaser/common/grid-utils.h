#ifndef PHASER_COMMON_GRID_UTILS_H_
#define PHASER_COMMON_GRID_UTILS_H_

#include <array>
#include <stdint.h>
#include <vector>

typedef struct grid_indexes {
  int32_t x;
  int32_t y;
  int32_t z;
} grid_indexes_t;

namespace common {

class GridUtils {
 public:
  static void getNeighbors(
      int32_t index, int32_t grid_size, int32_t neighbors_radius,
      std::vector<int32_t>* neighbors_indexes);
  static grid_indexes_t ind2sub(int32_t index, int32_t grid_size);
  static int32_t sub2ind(int32_t i, int32_t j, int32_t k, int32_t grid_size);
};

}  // namespace common

#endif  // PHASER_COMMON_GRID_UTILS_H_
