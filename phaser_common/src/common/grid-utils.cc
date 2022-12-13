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
  VLOG(4) << "Found " << neighbors_indexes->size() << " neighbors";
  return;
}

void GridUtils::getNeighborsTranslation(
    int32_t index, int32_t grid_size, int32_t neighbors_radius,
    std::vector<int32_t>* neighbors_indexes) {
  neighbors_indexes->clear();

  grid_indexes_t grid_indexes = ind2sub(index, grid_size);

  std::vector<int32_t> indexes_temp_x;
  std::vector<int32_t> indexes_temp_y;
  std::vector<int32_t> indexes_temp_z;
  
  getIndexesTranslation(
      grid_indexes.x, grid_size, neighbors_radius, &indexes_temp_x);
  getIndexesTranslation(
      grid_indexes.y, grid_size, neighbors_radius, &indexes_temp_y);
  getIndexesTranslation(
      grid_indexes.z, grid_size, neighbors_radius, &indexes_temp_z);

  for (auto index_x : indexes_temp_x) {
    for (auto index_y : indexes_temp_y) {
      for (auto index_z : indexes_temp_z) {
        if (index_x == grid_indexes.x && index_y == grid_indexes.y &&
            index_z == grid_indexes.z)
          continue;
        int32_t current_index = sub2ind(index_x, index_y, index_z, grid_size);
        neighbors_indexes->push_back(current_index);
      }
    }
  }

  VLOG(4) << "Found " << neighbors_indexes->size() << " neighbors";
  return;
}

void GridUtils::getIndexesTranslation(
    int32_t index, int32_t grid_size, int32_t neighbors_radius,
    std::vector<int32_t>* neighbors_indexes) {
  
  /*
  The indexes from 0 to grid_size/2 are increasing positive translations,
  the indexes from grid_size/2+1 to grid_size-1 are decreasing negative
  (see common::TranslationUtils::ComputeTranslationFromIndex)

  example:
  n_voxels = 10
  voxel_size = 0.5
  indexes = 
  [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
  corresponding_translations = 
  [0, 0.05, 0.1, 0.15, 0.2, 0.25, -0.2, -0.15, -0.1, -0.05]
  */

  neighbors_indexes->clear();
  neighbors_indexes->push_back(index);
  // Positive translations
  if (index <= grid_size / 2) {
    // Left side
    for (int i = -neighbors_radius; i < 0; i++) {
      int tmp_index = index + i;
      if (tmp_index < 0)
        tmp_index = grid_size + tmp_index;
      if (tmp_index < grid_size / 2 && tmp_index >= index)
        continue;
      else
        neighbors_indexes->push_back(tmp_index);
    }
    // Right side
    for (int i = 1; i <= neighbors_radius; i++) {
      int tmp_index = index + i;
      if (tmp_index > grid_size / 2)
        continue;
      neighbors_indexes->push_back(tmp_index);
    }
  }

  // Negative translations
  if (index > grid_size / 2) {
    // Left side
    for (int i = -neighbors_radius; i < 0; i++) {
      int tmp_index = index + i;
      if (tmp_index <= grid_size / 2)
        continue;
      neighbors_indexes->push_back(tmp_index);
    }
    // Right side
    for (int i = 1; i <= neighbors_radius; i++) {
      int tmp_index = index + i;
      if (tmp_index >= grid_size) {
        tmp_index = tmp_index - grid_size;
        if (tmp_index >= index)
          continue;
      }
      neighbors_indexes->push_back(tmp_index);
    }
    return;
  }
}

void GridUtils::getNeighborsRotation(
    int32_t index, int32_t grid_size, int32_t neighbors_radius,
    std::vector<int32_t>* neighbors_indexes) {
  neighbors_indexes->clear();

  grid_indexes_t grid_indexes = ind2sub(index, grid_size);

  std::vector<int32_t> indexes_temp_x;
  std::vector<int32_t> indexes_temp_y;
  std::vector<int32_t> indexes_temp_z;

  getIndexesRotation(
      grid_indexes.x, grid_size, neighbors_radius, &indexes_temp_x);
  getIndexesRotation(
      grid_indexes.y, grid_size, neighbors_radius, &indexes_temp_y);
  getIndexesRotation(
      grid_indexes.z, grid_size, neighbors_radius, &indexes_temp_z);

  for (auto index_x : indexes_temp_x) {
    for (auto index_y : indexes_temp_y) {
      for (auto index_z : indexes_temp_z) {
        if (index_x == grid_indexes.x && index_y == grid_indexes.y &&
            index_z == grid_indexes.z)
          continue;
        int32_t current_index = sub2ind(index_x, index_y, index_z, grid_size);
        neighbors_indexes->push_back(current_index);
      }
    }
  }

  VLOG(4) << "Found " << neighbors_indexes->size() << " neighbors";
  return;
}

void GridUtils::getIndexesRotation(
    int32_t index, int32_t grid_size, int32_t neighbors_radius,
    std::vector<int32_t>* neighbors_indexes) {
  neighbors_indexes->clear();

  neighbors_indexes->push_back(index);
  for (int i = -neighbors_radius; i <= neighbors_radius; i++) {
    int tmp_index = index + i;
    if (tmp_index < 0)
      tmp_index = grid_size + tmp_index;
      neighbors_indexes->push_back(tmp_index);
      continue;
    if (tmp_index > grid_size)
      tmp_index = tmp_index - grid_size;
      neighbors_indexes->push_back(tmp_index);
      continue;
    neighbors_indexes->push_back(tmp_index);
  }
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
