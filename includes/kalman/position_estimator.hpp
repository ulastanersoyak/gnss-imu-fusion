#ifndef POSITION_ESTIMATOR_HPP
#define POSITION_ESTIMATOR_HPP

#include "kalman/kalman_filter.hpp"
#include "kalman/sensor_event.hpp"

#include <array>
#include <cstdint>
#include <optional>

struct estimator_config
{
  // GPS position noise: sigma = hdop * user equivalent range error.
  double gps_uere_m{ 2.0 };
  double gps_default_sigma_m{ 4.0 }; // when no HDOP is known
  double gps_min_sigma_m{ 1.0 };
  double gps_velocity_sigma_mps{ 0.3 };
  // Below this speed the receiver's course is unreliable, the fix is used as
  // a zero velocity measurement instead.
  double stationary_speed_mps{ 0.3 };
  double stationary_velocity_sigma_mps{ 0.3 };

  // Process noise, continuous white acceleration with spectral density
  // sigma^2 [m^2/s^3]: the velocity uncertainty grows by sigma per sqrt(s).
  // Small when the IMU acceleration is used as control input, large when
  // only GPS is available.
  double imu_accel_sigma_mps2{ 0.3 };
  double maneuver_accel_sigma_mps2{ 2.0 };

  // Chi-square gate for 2 dof, 99.9 %. After this many consecutive rejected
  // fixes the filter assumes it diverged and restarts from the GPS.
  double gate_nis{ 13.82 };
  std::size_t max_rejected_fixes{ 5 };

  // predict_ahead() extrapolates with the recent IMU acceleration, low pass
  // filtered with this time constant to suppress sensor noise.
  double prediction_accel_tau_s{ 0.2 };

  double imu_nominal_dt_s{ 0.01 }; // 100 Hz
  double imu_max_dt_s{ 0.5 };
};

struct position_estimate
{
  utm position{};    // time is the time of the latest GPS fix
  double age_s{ 0 }; // seconds of prediction since that fix
  double velocity_east_mps{ 0 };
  double velocity_north_mps{ 0 };
  double sigma_east_m{ 0 };
  double sigma_north_m{ 0 };
  double sigma_velocity_east_mps{ 0 };
  double sigma_velocity_north_mps{ 0 };
};

// Fuses GPS fixes (position, speed, course) with IMU acceleration in a
// constant velocity Kalman filter. State: [east, north, v_east, v_north] in
// the UTM grid of the first fix.
class position_estimator
{
public:
  enum class fix_result
  {
    initialized,
    accepted,
    rejected,
    reset
  };

  explicit position_estimator (estimator_config config = {});

  fix_result add_gps_fix (const gps_fix &fix);
  // Returns false if the sample was ignored (filter not initialized yet).
  bool add_imu_sample (const Xbus &sample);

  [[nodiscard]] bool
  initialized () const noexcept
  {
    return initialized_;
  }

  [[nodiscard]] std::optional<position_estimate> estimate () const;
  // Where the filter expects to be horizon_s seconds from now, assuming the
  // current (IMU measured) acceleration persists. Without an IMU this is a
  // constant velocity extrapolation.
  [[nodiscard]] std::optional<position_estimate>
  predict_ahead (double horizon_s) const;

private:
  void initialize (const gps_fix &fix);
  void propagate (kalman_filter<4> &filter, double dt,
                  const std::array<double, 2> *accel_grid) const;
  [[nodiscard]] position_estimate to_estimate (const kalman_filter<4> &filter,
                                               double age_s) const;

  estimator_config config_;
  kalman_filter<4> filter_{};
  bool initialized_{ false };
  std::uint32_t zone_number_{ 0 };
  char zone_char_{ '_' };
  double convergence_{ 0 };
  utc last_fix_time_{};
  double seconds_since_fix_{ 0 };
  std::size_t imu_samples_since_fix_{ 0 };
  std::size_t rejected_fixes_{ 0 };
  std::optional<std::uint32_t> last_imu_tick_{};
  std::optional<std::array<double, 2>> recent_accel_grid_{};
};

#endif // !POSITION_ESTIMATOR_HPP
