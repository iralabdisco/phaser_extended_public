#include "phaser/backend/correlation/spherical-combined-worker.h"

#include <glog/logging.h>

#include "phaser/backend/correlation/spherical-correlation-laplace.h"
#include "phaser/common/core-gflags.h"

namespace phaser_core {

SphericalCombinedWorker::SphericalCombinedWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values)
    : f_values_(f_values), h_values_(h_values) {
  sph_corr_.reset(new SphericalCorrelationLaplace(
      FLAGS_phaser_core_spherical_bandwidth,
      FLAGS_phaser_core_spherical_zero_padding));
}

void SphericalCombinedWorker::run() {
  CHECK_NOTNULL(sph_corr_);
  VLOG(1) << "[SphericalCombinedWorker] Estimating rotation...";

  // Get the intensities.
  SampledSignal f_intensities;
  SampledSignal h_intensities;
  std::function<double(const model::FunctionValue&)> func_intensities =
      [](const model::FunctionValue& v) { return v.getAveragedIntensity(); };
  convertFunctionValues(f_values_, func_intensities, &f_intensities);
  convertFunctionValues(h_values_, func_intensities, &h_intensities);

  // Get the ranges.
  SampledSignal f_range;
  SampledSignal h_range;
  std::function<double(const model::FunctionValue&)> func_range =
      [](const model::FunctionValue& v) { return v.getAveragedRange(); };
  convertFunctionValues(f_values_, func_range, &f_range);
  convertFunctionValues(h_values_, func_range, &h_range);

  // Get the reflectivities.
  SampledSignal f_reflectivity;
  SampledSignal h_reflectivity;
  std::function<double(const model::FunctionValue&)> func_reflectivity =
      [](const model::FunctionValue& v) { return v.getAveragedReflectivity();
      };
  convertFunctionValues(f_values_, func_reflectivity, &f_reflectivity);
  convertFunctionValues(h_values_, func_reflectivity, &h_reflectivity);

  // Get the ambient points.
  SampledSignal f_ambient;
  SampledSignal h_ambient;
  std::function<double(const model::FunctionValue&)> func_ambient =
      [](const model::FunctionValue& v) { return v.getAveragedAmbientNoise();
      };
  convertFunctionValues(f_values_, func_ambient, &f_ambient);
  convertFunctionValues(h_values_, func_ambient, &h_ambient);

  bool f_refl_zero = std::all_of(f_reflectivity.begin(), f_reflectivity.end(), [](int i) { return i==0; });
  bool h_refl_zero = std::all_of(h_reflectivity.begin(), h_reflectivity.end(), [](int i) { return i==0; });
  bool f_amb_zero = std::all_of(f_ambient.begin(), f_ambient.end(), [](int i) { return i==0; });
  bool h_amb_zero = std::all_of(h_ambient.begin(), h_ambient.end(), [](int i) { return i==0; });
  
  bool has_reflectivity = !f_refl_zero && !h_refl_zero;
  bool has_ambient = !f_amb_zero && !h_amb_zero;

  std::vector<phaser_core::SampledSignal> f_channels = {f_intensities, f_range};
  std::vector<phaser_core::SampledSignal> h_channels = {h_intensities, h_range};
  if (has_reflectivity) {
    VLOG(1) << "[SphericalCombinedWorker] Using reflectivity";
    f_channels.push_back(f_reflectivity);
    h_channels.push_back(h_reflectivity);
  }
  if (has_ambient) {
    VLOG(1) << "[SphericalCombinedWorker] Using ambient";
    f_channels.push_back(f_ambient);
    h_channels.push_back(h_ambient);
  }

  sph_corr_->correlateSampledSignals(f_channels, h_channels);
  is_completed_ = true;
}

std::vector<double> SphericalCombinedWorker::getCorrelation() const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return sph_corr_->getCorrelation();
}

const SphericalCorrelation& SphericalCombinedWorker::getCorrelationObject()
    const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return *sph_corr_;
}

void SphericalCombinedWorker::shutdown() {
  CHECK_NOTNULL(sph_corr_);
  sph_corr_->shutdown();
}

}  // namespace phaser_core
