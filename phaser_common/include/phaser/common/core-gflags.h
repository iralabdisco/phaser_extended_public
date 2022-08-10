#ifndef PHASER_COMMON_CORE_GFLAGS_H_
#define PHASER_COMMON_CORE_GFLAGS_H_

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace phaser_core {

// Spherical correlation.
DECLARE_int32(phaser_core_spherical_bandwidth);
DECLARE_int32(phaser_core_spherical_zero_padding);
DECLARE_int32(phaser_core_spherical_low_pass_lower_bound);
DECLARE_int32(phaser_core_spherical_low_pass_upper_bound);

// Spatial correlation.
DECLARE_int32(phaser_core_spatial_n_voxels);
DECLARE_int32(phaser_core_spatial_discretize_lower);
DECLARE_int32(phaser_core_spatial_discretize_upper);
DECLARE_int32(phaser_core_spatial_zero_padding);
DECLARE_int32(phaser_core_spatial_low_pass_lower_bound);
DECLARE_int32(phaser_core_spatial_low_pass_upper_bound);

// Distribution fitting parameters
DECLARE_int32(bingham_peak_neighbors);
DECLARE_int32(bingham_peak_neighbors_radius);
DECLARE_int32(gaussian_peak_neighbors_radius);

// Peak parameters
DECLARE_int32(max_peaks_number);
DECLARE_double(peaks_discard_threshold);

// Utils
DECLARE_bool(dump_correlation_to_file);
DECLARE_bool(dump_peaks_to_file);
DECLARE_string(result_folder);
DECLARE_bool(estimate_rotation);
DECLARE_bool(estimate_translation);
}  // namespace phaser_core

#endif  // PHASER_COMMON_CORE_GFLAGS_H_
