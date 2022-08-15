#include "phaser/common/core-gflags.h"

namespace phaser_core {

// Spherical correlation.
DEFINE_int32(
    phaser_core_spherical_bandwidth, 75, "Defines the spherical bandwith.");
DEFINE_int32(
    phaser_core_spherical_zero_padding, 0,
    "Specifies whether and how much zero padding should be applied to the "
    "spherical correlation.");
DEFINE_int32(
    phaser_core_spherical_low_pass_lower_bound, 0,
    "Defines the lower bound of the spherical low pass.");
DEFINE_int32(
    phaser_core_spherical_low_pass_upper_bound, 100000,
    "Defines the upper bound of the spherical low pass.");

// Spatial correlation.
DEFINE_int32(
    phaser_core_spatial_n_voxels, 100,
    "Defines the number of voxels used per dimension"
    " in the spatial correlation.");
DEFINE_int32(
    phaser_core_spatial_discretize_lower, -50,
    "Specifies the lower bound for the discretization.");
DEFINE_int32(
    phaser_core_spatial_discretize_upper, 50,
    "Specifies the upper bound for the discretization.");
DEFINE_int32(
    phaser_core_spatial_zero_padding, 0,
    "Specifies whether the spatial correlation should make use of zero "
    "padding.");
DEFINE_int32(
    phaser_core_spatial_low_pass_lower_bound, 0,
    "Defines the lower frequency bound of the spatial low pass filtering.");
DEFINE_int32(
    phaser_core_spatial_low_pass_upper_bound, 100000,
    "Defines the lower frequency bound of the spatial low pass filtering.");

// Distribution fitting parameters
DEFINE_int32(
    bingham_peak_neighbors, 0,
    "Determines the number of neighbors used for the Bingham calculation.");

DEFINE_int32(
    bingham_peak_neighbors_radius, 1,
    "Defines the radius used to find local maxima and to fit the bibgham "
    "distribuition.");

DEFINE_int32(
    gaussian_peak_neighbors_radius, 1,
    "Defines the radius used to find local maxima and to fit the gaussian "
    "distribuition.");

// Peak parameters
DEFINE_int32(
    max_peaks_number, 4, "Take only the first best max_peak_number peaks.");
DEFINE_double(
    peaks_discard_threshold, 0.5,
    "Do not evaluate peaks that are under threshold*maximum correlation.");

// Parameters for distribution fitting with neighbors.
DEFINE_int32(
    bingham_samples_radius, 1,
    "Defines the radius used to take the samples for bingham distribution "
    "fitting.");
DEFINE_int32(
    gaussian_samples_radius, 1,
    "Defines the radius used to take the samples for gaussian distribution "
    "fitting.");

// Utils
DEFINE_bool(
    dump_correlation_to_file, false,
    "Decide to dump the correlation to csv or not");
DEFINE_bool(
    dump_peaks_to_file, false, "Decide to dump the peaks to csv or not");
DEFINE_string(
    result_folder, "",
    "Defines the folder where the results should be dumped.");
DEFINE_bool(
    save_registered_clouds, true,
    "Defines if the registered clouds should be saved.");
DEFINE_bool(estimate_rotation, true, "Esttimates the rotation if true.");
DEFINE_bool(estimate_translation, true, "Esttimates the translation if true.");
}  // namespace phaser_core
