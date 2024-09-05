#ifndef LOCAL_FRAME_HPP
#define LOCAL_FRAME_HPP

#include <array>
#include <cmath>
#include <numbers>

// Converts WGS84 latitude/longitude/height to a local east-north-up tangent
// plane, the same frame Gazebo uses for an ENU <spherical_coordinates> world.
class local_frame
{
  static constexpr double a = 6378137.0;
  static constexpr double f = 1.0 / 298.257223563;
  static constexpr double e2 = (2 * f) - (f * f);
  static constexpr double deg = std::numbers::pi / 180;

  double sin_lat0_;
  double cos_lat0_;
  double sin_lon0_;
  double cos_lon0_;
  std::array<double, 3> ecef0_;

  static std::array<double, 3>
  to_ecef (double lat_deg, double lon_deg, double h)
  {
    const double lat = lat_deg * deg;
    const double lon = lon_deg * deg;
    const double N = a / std::sqrt (1 - e2 * std::sin (lat) * std::sin (lat));
    return { (N + h) * std::cos (lat) * std::cos (lon),
             (N + h) * std::cos (lat) * std::sin (lon),
             (N * (1 - e2) + h) * std::sin (lat) };
  }

public:
  local_frame (double lat0_deg, double lon0_deg, double h0)
      : sin_lat0_ (std::sin (lat0_deg * deg)),
        cos_lat0_ (std::cos (lat0_deg * deg)),
        sin_lon0_ (std::sin (lon0_deg * deg)),
        cos_lon0_ (std::cos (lon0_deg * deg)),
        ecef0_ (to_ecef (lat0_deg, lon0_deg, h0))
  {
  }

  // {east, north, up} in metres.
  [[nodiscard]] std::array<double, 3>
  to_enu (double lat_deg, double lon_deg, double h) const
  {
    const auto p = to_ecef (lat_deg, lon_deg, h);
    const double dx = p[0] - ecef0_[0];
    const double dy = p[1] - ecef0_[1];
    const double dz = p[2] - ecef0_[2];
    return { (-sin_lon0_ * dx) + (cos_lon0_ * dy),
             (-sin_lat0_ * cos_lon0_ * dx) - (sin_lat0_ * sin_lon0_ * dy)
                 + (cos_lat0_ * dz),
             (cos_lat0_ * cos_lon0_ * dx) + (cos_lat0_ * sin_lon0_ * dy)
                 + (sin_lat0_ * dz) };
  }
};

#endif // !LOCAL_FRAME_HPP
