#include "kalman/position_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
constexpr double DEG_TO_RAD = std::numbers::pi / 180;

// Position measurement: picks east and north out of the state.
const matrix<2, 4> H_position{ 1, 0, 0, 0, //
                               0, 1, 0, 0 };
// Velocity measurement.
const matrix<2, 4> H_velocity{ 0, 0, 1, 0, //
                               0, 0, 0, 1 };

matrix<2, 2>
diagonal_noise (double sigma)
{
  return { sigma * sigma, 0, 0, sigma * sigma };
}

// Rotates a vector from true east/north to UTM grid east/north. The grid
// convergence is the clockwise angle from true north to grid north.
std::array<double, 2>
true_to_grid (double east, double north, double convergence)
{
  const double c = std::cos (convergence);
  const double s = std::sin (convergence);
  return { (east * c) - (north * s), (east * s) + (north * c) };
}

std::optional<vector<2>>
velocity_from_fix (const gps_fix &fix, double convergence)
{
  if (!fix.course_deg)
    {
      return std::nullopt;
    }
  const double course = *fix.course_deg * DEG_TO_RAD;
  const auto [ve, vn]
      = true_to_grid (fix.speed_mps * std::sin (course),
                      fix.speed_mps * std::cos (course), convergence);
  return vector<2>{ ve, vn };
}
} // namespace

position_estimator::position_estimator (estimator_config config)
    : config_ (config)
{
}

void
position_estimator::initialize (const gps_fix &fix)
{
  const double sigma_pos = fix.hdop ? std::max (config_.gps_min_sigma_m,
                                                *fix.hdop * config_.gps_uere_m)
                                    : config_.gps_default_sigma_m;

  convergence_ = fix.position.convergence ();
  vector<4> x0{ fix.position.easting (), fix.position.northing (), 0, 0 };
  double sigma_vel = 5.0; // unknown, allow anything a vehicle could do
  if (fix.speed_mps < config_.stationary_speed_mps)
    {
      sigma_vel = 1.0;
    }
  else if (const auto v = velocity_from_fix (fix, convergence_))
    {
      x0 (2, 0) = (*v) (0, 0);
      x0 (3, 0) = (*v) (1, 0);
      sigma_vel = config_.gps_velocity_sigma_mps * 2;
    }

  matrix<4, 4> P0{};
  P0 (0, 0) = P0 (1, 1) = sigma_pos * sigma_pos;
  P0 (2, 2) = P0 (3, 3) = sigma_vel * sigma_vel;

  filter_ = kalman_filter<4>{ x0, P0 };
  zone_number_ = fix.position.zone_number ();
  zone_char_ = fix.position.zone_char ();
  last_fix_time_ = fix.position.time ();
  seconds_since_fix_ = 0;
  imu_samples_since_fix_ = 0;
  rejected_fixes_ = 0;
  recent_accel_grid_.reset ();
  initialized_ = true;
}

void
position_estimator::propagate (kalman_filter<4> &filter, double dt,
                               const std::array<double, 2> *accel_grid) const
{
  const matrix<4, 4> F{ 1, 0, dt, 0,  //
                        0, 1, 0,  dt, //
                        0, 0, 1,  0,  //
                        0, 0, 0,  1 };

  const double sigma = accel_grid ? config_.imu_accel_sigma_mps2
                                  : config_.maneuver_accel_sigma_mps2;
  // Continuous white noise acceleration: the uncertainty added over a time
  // span does not depend on how many steps it is split into, so 100 Hz IMU
  // steps and 1 Hz GPS-only steps grow the covariance equally.
  const double q = sigma * sigma;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  matrix<4, 4> Q{};
  Q (0, 0) = Q (1, 1) = q * dt3 / 3;
  Q (0, 2) = Q (2, 0) = Q (1, 3) = Q (3, 1) = q * dt2 / 2;
  Q (2, 2) = Q (3, 3) = q * dt;

  vector<4> u{};
  if (accel_grid)
    {
      const auto [ae, an] = *accel_grid;
      u = vector<4>{ ae * dt2 / 2, an * dt2 / 2, ae * dt, an * dt };
    }
  filter.predict (F, Q, u);
}

bool
position_estimator::add_imu_sample (const Xbus &sample)
{
  double dt = config_.imu_nominal_dt_s;
  if (sample.sampleTimeFineAvailable)
    {
      if (last_imu_tick_)
        {
          // Unsigned subtraction handles the 32 bit counter wrapping.
          const std::uint32_t ticks = sample.sampleTimeFine - *last_imu_tick_;
          dt = ticks / Xbus::sampleTimeFineHz;
        }
      last_imu_tick_ = sample.sampleTimeFine;
    }
  if (dt <= 0 || dt > config_.imu_max_dt_s)
    {
      dt = config_.imu_nominal_dt_s;
    }

  if (!initialized_)
    {
      return false;
    }

  if (const auto accel = sample.horizontalAccelerationEN ())
    {
      const auto grid = true_to_grid ((*accel)[0], (*accel)[1], convergence_);
      propagate (filter_, dt, &grid);

      const double alpha = dt / (config_.prediction_accel_tau_s + dt);
      if (!recent_accel_grid_)
        {
          recent_accel_grid_ = grid;
        }
      for (std::size_t i = 0; i < 2; ++i)
        {
          (*recent_accel_grid_)[i]
              += alpha * (grid[i] - (*recent_accel_grid_)[i]);
        }
    }
  else
    {
      propagate (filter_, dt, nullptr);
    }

  seconds_since_fix_ += dt;
  ++imu_samples_since_fix_;
  return true;
}

position_estimator::fix_result
position_estimator::add_gps_fix (const gps_fix &fix)
{
  if (!initialized_ || fix.position.zone_number () != zone_number_
      || fix.position.is_southern () != (zone_char_ < 'N'))
    {
      // TODO: carry the state across UTM zone boundaries instead of
      // restarting.
      initialize (fix);
      return fix_result::initialized;
    }

  const double dt = fix.position.time () - last_fix_time_;
  if (dt <= 0)
    {
      return fix_result::rejected; // duplicate or out of order
    }
  if (imu_samples_since_fix_ == 0)
    {
      // No IMU: the GPS fixes themselves drive the prediction.
      propagate (filter_, dt, nullptr);
    }

  last_fix_time_ = fix.position.time ();
  seconds_since_fix_ = 0;
  imu_samples_since_fix_ = 0;
  convergence_ = fix.position.convergence ();

  const double sigma_pos = fix.hdop ? std::max (config_.gps_min_sigma_m,
                                                *fix.hdop * config_.gps_uere_m)
                                    : config_.gps_default_sigma_m;
  const vector<2> z{ fix.position.easting (), fix.position.northing () };
  const auto R = diagonal_noise (sigma_pos);

  if (filter_.nis (z, H_position, R) > config_.gate_nis)
    {
      if (++rejected_fixes_ > config_.max_rejected_fixes)
        {
          initialize (fix);
          return fix_result::reset;
        }
      return fix_result::rejected;
    }
  rejected_fixes_ = 0;
  filter_.update (z, H_position, R);

  if (fix.speed_mps < config_.stationary_speed_mps)
    {
      filter_.update (vector<2>{ 0, 0 }, H_velocity,
                      diagonal_noise (config_.stationary_velocity_sigma_mps));
    }
  else if (const auto v = velocity_from_fix (fix, convergence_))
    {
      const auto R_v = diagonal_noise (config_.gps_velocity_sigma_mps);
      if (filter_.nis (*v, H_velocity, R_v) <= config_.gate_nis)
        {
          filter_.update (*v, H_velocity, R_v);
        }
    }
  return fix_result::accepted;
}

position_estimate
position_estimator::to_estimate (const kalman_filter<4> &filter,
                                 double age_s) const
{
  const auto &x = filter.state ();
  const auto &P = filter.covariance ();
  return { .position = utm (x (0, 0), x (1, 0), zone_number_, zone_char_,
                            last_fix_time_),
           .age_s = age_s,
           .velocity_east_mps = x (2, 0),
           .velocity_north_mps = x (3, 0),
           .sigma_east_m = std::sqrt (P (0, 0)),
           .sigma_north_m = std::sqrt (P (1, 1)),
           .sigma_velocity_east_mps = std::sqrt (P (2, 2)),
           .sigma_velocity_north_mps = std::sqrt (P (3, 3)) };
}

std::optional<position_estimate>
position_estimator::estimate () const
{
  if (!initialized_)
    {
      return std::nullopt;
    }
  return to_estimate (filter_, seconds_since_fix_);
}

std::optional<position_estimate>
position_estimator::predict_ahead (double horizon_s) const
{
  if (!initialized_)
    {
      return std::nullopt;
    }
  auto future = filter_;
  propagate (future, horizon_s,
             recent_accel_grid_ ? &*recent_accel_grid_ : nullptr);
  return to_estimate (future, seconds_since_fix_ + horizon_s);
}
