#include "utm/utm.hpp"
#include "time_and_location/coordinate.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string_view>

// Formulas from Snyder, "Map Projections - A Working Manual" (USGS 1395),
// pages 61-64.

inline constexpr double WGS84_SEMI_MAJOR_AXIS{ 6378137.0 };
inline constexpr double WGS84_FLATTENING{ 1.0 / 298.257223563 };
inline constexpr double ECCENTRICITY_2{
  (2 * WGS84_FLATTENING) - (WGS84_FLATTENING * WGS84_FLATTENING)
};
// Second eccentricity squared, e'^2.
inline constexpr double ECCENTRICITY_PRIME_2{ ECCENTRICITY_2
                                              / (1 - ECCENTRICITY_2) };
inline constexpr double SCALE_FACTOR{ 0.9996 };
inline constexpr double FALSE_EASTING{ 500000 };
inline constexpr double FALSE_NORTHING_NORTH{ 0 };
inline constexpr double FALSE_NORTHING_SOUTH{ 10000000 };
inline constexpr double DEG_TO_RAD{ std::numbers::pi / 180 };

inline constexpr std::string_view letters{ "CDEFGHJKLMNPQRSTUVWX" };

namespace
{
double
central_meridian (std::uint32_t zone)
{
  return ((static_cast<double> (zone) * 6) - 183) * DEG_TO_RAD;
}

std::uint32_t
zone_for (double latitude, double longitude)
{
  // Norway and Svalbard exceptions.
  if (latitude >= 56 && latitude < 64 && longitude >= 3 && longitude < 12)
    {
      return 32;
    }
  if (latitude >= 72 && longitude >= 0 && longitude < 42)
    {
      if (longitude < 9)
        return 31;
      if (longitude < 21)
        return 33;
      if (longitude < 33)
        return 35;
      return 37;
    }
  const auto zone
      = static_cast<std::uint32_t> (std::floor ((longitude + 180) / 6) + 1);
  return zone > 60 ? 1 : zone;
}

// Meridional arc length from the equator to latitude phi.
double
meridional_arc (double phi)
{
  constexpr double e2 = ECCENTRICITY_2;
  constexpr double e4 = e2 * e2;
  constexpr double e6 = e4 * e2;
  return WGS84_SEMI_MAJOR_AXIS
         * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
            - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin (2 * phi)
            + (15 * e4 / 256 + 45 * e6 / 1024) * std::sin (4 * phi)
            - (35 * e6 / 3072) * std::sin (6 * phi));
}
} // namespace

utm::utm (const rmc &rmc_)
    : utm (coordinate_to_decimal (rmc_.latitude_),
           coordinate_to_decimal (rmc_.longitude_), rmc_.time_)
{
}

utm::utm (double latitude, double longitude, utc time) : time_ (time)
{
  if (latitude < -80 || latitude > 84 || longitude < -180 || longitude > 180)
    {
      throw std::out_of_range ("latitude/longitude outside of utm coverage");
    }

  const auto zone = zone_for (latitude, longitude);
  const double lambda0 = central_meridian (zone);

  const double phi = latitude * DEG_TO_RAD;
  const double lambda = longitude * DEG_TO_RAD;
  const double sin_phi = std::sin (phi);
  const double cos_phi = std::cos (phi);
  const double tan_phi = std::tan (phi);

  const double N = WGS84_SEMI_MAJOR_AXIS
                   / std::sqrt (1 - ECCENTRICITY_2 * sin_phi * sin_phi);
  const double T = tan_phi * tan_phi;
  const double C = ECCENTRICITY_PRIME_2 * cos_phi * cos_phi;
  const double A = cos_phi * (lambda - lambda0);
  const double M = meridional_arc (phi);

  const double A2 = A * A;
  const double A3 = A2 * A;
  const double A4 = A3 * A;
  const double A5 = A4 * A;
  const double A6 = A5 * A;

  this->easting_
      = FALSE_EASTING
        + SCALE_FACTOR * N
              * (A + (1 - T + C) * A3 / 6
                 + (5 - 18 * T + T * T + 72 * C - 58 * ECCENTRICITY_PRIME_2)
                       * A5 / 120);

  const double false_northing
      = (latitude < 0) ? FALSE_NORTHING_SOUTH : FALSE_NORTHING_NORTH;
  this->northing_
      = false_northing
        + SCALE_FACTOR
              * (M
                 + N * tan_phi
                       * (A2 / 2 + (5 - T + 9 * C + 4 * C * C) * A4 / 24
                          + (61 - 58 * T + T * T + 600 * C
                             - 330 * ECCENTRICITY_PRIME_2)
                                * A6 / 720));

  this->zone_number_ = zone;
  const auto band
      = std::min (static_cast<std::size_t> (std::floor ((latitude + 80) / 8)),
                  letters.size () - 1);
  this->zone_char_ = letters[band];
  this->convergence_ = std::atan (std::tan (lambda - lambda0) * sin_phi);
}

utm::utm (double easting, double northing, std::uint32_t zone_number,
          char zone_char, utc time)
    : easting_ (easting), northing_ (northing), zone_number_ (zone_number),
      zone_char_ (zone_char), time_ (time)
{
  const auto [lat, lon] = to_latlon ();
  this->convergence_ = std::atan (
      std::tan ((lon * DEG_TO_RAD) - central_meridian (zone_number))
      * std::sin (lat * DEG_TO_RAD));
}

[[nodiscard]] std::pair<double, double>
utm::to_latlon () const noexcept
{
  constexpr double e2 = ECCENTRICITY_2;
  constexpr double ep2 = ECCENTRICITY_PRIME_2;
  const double e1 = (1 - std::sqrt (1 - e2)) / (1 + std::sqrt (1 - e2));

  const double x = easting_ - FALSE_EASTING;
  const double y
      = northing_
        - (is_southern () ? FALSE_NORTHING_SOUTH : FALSE_NORTHING_NORTH);

  const double M = y / SCALE_FACTOR;
  const double mu
      = M
        / (WGS84_SEMI_MAJOR_AXIS
           * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
  const double phi1
      = mu + (3 * e1 / 2 - 27 * std::pow (e1, 3) / 32) * std::sin (2 * mu)
        + (21 * e1 * e1 / 16 - 55 * std::pow (e1, 4) / 32) * std::sin (4 * mu)
        + (151 * std::pow (e1, 3) / 96) * std::sin (6 * mu)
        + (1097 * std::pow (e1, 4) / 512) * std::sin (8 * mu);

  const double sin_phi1 = std::sin (phi1);
  const double cos_phi1 = std::cos (phi1);
  const double tan_phi1 = std::tan (phi1);
  const double C1 = ep2 * cos_phi1 * cos_phi1;
  const double T1 = tan_phi1 * tan_phi1;
  const double N1
      = WGS84_SEMI_MAJOR_AXIS / std::sqrt (1 - e2 * sin_phi1 * sin_phi1);
  const double R1 = WGS84_SEMI_MAJOR_AXIS * (1 - e2)
                    / std::pow (1 - e2 * sin_phi1 * sin_phi1, 1.5);
  const double D = x / (N1 * SCALE_FACTOR);

  const double lat = phi1
                     - (N1 * tan_phi1 / R1)
                           * (D * D / 2
                              - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * ep2)
                                    * std::pow (D, 4) / 24
                              + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1
                                 - 252 * ep2 - 3 * C1 * C1)
                                    * std::pow (D, 6) / 720);
  const double lon
      = central_meridian (zone_number_)
        + (D - (1 + 2 * T1 + C1) * std::pow (D, 3) / 6
           + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * ep2 + 24 * T1 * T1)
                 * std::pow (D, 5) / 120)
              / cos_phi1;

  return { lat / DEG_TO_RAD, lon / DEG_TO_RAD };
}

std::ostream &
operator<< (std::ostream &os, const utm &utm)
{
  os << "Easting: " << utm.easting_ << ", " << "Northing: " << utm.northing_
     << ", " << "Zone Number: " << utm.zone_number_ << ", "
     << "Zone Char: " << utm.zone_char_ << ", " << "Time: " << utm.time_
     << "\n";
  return os;
}

[[nodiscard]] double
utm::get_time_diff_sec (const utm &other) const noexcept
{
  return this->time_ - other.time_;
}

[[nodiscard]] double
utm::get_distance (const utm &other) const noexcept
{
  return std::hypot (this->easting_ - other.easting_,
                     this->northing_ - other.northing_);
}

[[nodiscard]] directional_info
utm::get_direction (const utm &other) const noexcept
{
  const double d_east = this->easting_ - other.easting_;
  const double d_north = this->northing_ - other.northing_;
  const auto EW = d_east > 0 ? direction::EAST : direction::WEST;
  const auto NS = d_north > 0 ? direction::NORTH : direction::SOUTH;

  double bearing = std::atan2 (d_east, d_north) / DEG_TO_RAD;
  if (bearing < 0)
    {
      bearing += 360;
    }
  return { NS, EW, bearing };
}

void
utm::update_easting (double distance)
{
  this->easting_ += distance;
}

void
utm::update_northing (double distance)
{
  this->northing_ += distance;
}
