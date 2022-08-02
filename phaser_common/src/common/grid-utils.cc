#include "phaser/common/grid-utils.h"

#include <glog/logging.h>

namespace common {

void GridUtils::getNeighbors(
    int32_t index, int32_t grid_size, int32_t neighbors_radius,
    std::vector<int32_t>* neighbors_indexes) {
  neighbors_indexes->clear();

  int32_t index_x = 0;
  int32_t index_y = 0;
  int32_t index_z = 0;

  grid_indexes_t grid_indexes = ind2sub(index, grid_size);

  for (int i = -neighbors_radius; i <= neighbors_radius; i++) {
    index_x = grid_indexes.x + i;
    if (index_x < 0 || index_x > grid_size - 1)
      continue;
    for (int j = -neighbors_radius; j <= neighbors_radius; j++) {
      index_y = grid_indexes.y + j;
      if (index_y < 0 || index_y > grid_size - 1)
        continue;
      for (int k = -neighbors_radius; k <= neighbors_radius; k++) {
        index_z = grid_indexes.z + k;
        if (index_z < 0 || index_z > grid_size - 1)
          continue;

        if (index_x == grid_indexes.x && index_y == grid_indexes.y &&
            index_z == grid_indexes.z)
          continue;

        int32_t current_index = sub2ind(index_x, index_y, index_z, grid_size);
        neighbors_indexes->push_back(current_index);
      }
    }
  }
  VLOG(3) << "Found " << neighbors_indexes->size() << " neighbors";
  return;
}

grid_indexes_t GridUtils::ind2sub(int32_t index, int32_t grid_size) {
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

int32_t GridUtils::sub2ind(int32_t i, int32_t j, int32_t k, int32_t grid_size) {
  // %https://www.alecjacobson.com/weblog/?p=1425
  return i + grid_size * j + grid_size * grid_size * k;
}

}  // namespace common
