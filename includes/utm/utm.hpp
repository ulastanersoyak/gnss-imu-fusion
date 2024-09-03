#ifndef UTM_HPP
#define UTM_HPP

#include "rmc/rmc.hpp"
#include "time_and_location/directional_info.hpp"
#include "time_and_location/utc.hpp"

#include <cstddef>
#include <iostream>
#include <utility>

class utm
{
  double easting_{ 0 };
  double northing_{ 0 };
  std::uint32_t zone_number_{ 0 };
  char zone_char_{ '_' };
  // Angle from true north to grid north (radians, clockwise positive).
  double convergence_{ 0 };
  utc time_{ 0, 0, 0 };

public:
  utm () = default;
  explicit utm (const rmc &rmc_);
  // Throws std::out_of_range outside the UTM latitude band [-80, 84].
  utm (double latitude_deg, double longitude_deg, utc time = {});
  utm (double easting, double northing, std::uint32_t zone_number,
       char zone_char, utc time = {});

  [[nodiscard]] double
  easting () const noexcept
  {
    return easting_;
  }
  [[nodiscard]] double
  northing () const noexcept
  {
    return northing_;
  }
  [[nodiscard]] std::uint32_t
  zone_number () const noexcept
  {
    return zone_number_;
  }
  [[nodiscard]] char
  zone_char () const noexcept
  {
    return zone_char_;
  }
  [[nodiscard]] double
  convergence () const noexcept
  {
    return convergence_;
  }
  [[nodiscard]] utc
  time () const noexcept
  {
    return time_;
  }
  [[nodiscard]] bool
  is_southern () const noexcept
  {
    return zone_char_ < 'N';
  }

  // {latitude, longitude} in decimal degrees.
  [[nodiscard]] std::pair<double, double> to_latlon () const noexcept;

  [[nodiscard]] double get_time_diff_sec (const utm &other) const noexcept;
  [[nodiscard]] double get_distance (const utm &other) const noexcept;
  // Direction of travel from other to this. dir_degree is the grid bearing
  // in degrees, clockwise from grid north, in [0, 360).
  [[nodiscard]] directional_info
  get_direction (const utm &other) const noexcept;
  friend std::ostream &operator<< (std::ostream &os, const utm &utm);
  void update_easting (double distance);
  void update_northing (double distance);
};

#endif // !UTM_HPP
