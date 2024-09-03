#ifndef UTM_TEST_HPP
#define UTM_TEST_HPP

#include "check.hpp"
#include "utm/utm.hpp"

#include <array>
#include <string>

void
utm_conversion_test ()
{
  struct reference
  {
    double lat;
    double lon;
    std::uint32_t zone;
    char band;
    double easting;
    double northing;
  };

  // Reference values computed with pyproj (PROJ) 3.8.
  // clang-format off
  const std::array<reference, 6> references{ {
    {  39.97656,   32.73812, 36, 'S', 477638.347, 4425188.446 }, // Ankara
    { -33.8568,   151.2153,  56, 'H', 334900.570, 6252288.753 }, // Sydney
    {  40.6892,   -74.0445,  18, 'T', 580735.871, 4504695.165 }, // New York
    {  60.0,        5.0,     32, 'V', 276979.926, 6658157.202 }, // Norway
    {  -0.5,      -78.5,     17, 'M', 778265.778, 9944681.960 }, // Ecuador
    {  78.0,       15.0,     33, 'X', 500000.000, 8658369.586 }, // Svalbard
  } };
  // clang-format on

  for (const auto &ref : references)
    {
      const std::string name
          = std::to_string (ref.lat) + "," + std::to_string (ref.lon);
      const utm u (ref.lat, ref.lon);
      check (u.zone_number () == ref.zone, "utm zone " + name);
      check (u.zone_char () == ref.band, "utm band " + name);
      check_near (u.easting (), ref.easting, 0.01, "utm easting " + name);
      check_near (u.northing (), ref.northing, 0.01, "utm northing " + name);

      const auto [lat, lon] = u.to_latlon ();
      check_near (lat, ref.lat, 1e-7, "inverse latitude " + name);
      check_near (lon, ref.lon, 1e-7, "inverse longitude " + name);
    }

  // Grid convergence, also from pyproj.
  check_near (utm (39.97656, 32.73812).convergence () * 180 / 3.14159265358979,
              -0.168252, 1e-5, "convergence west of meridian");
  check_near (utm (-33.8568, 151.2153).convergence () * 180 / 3.14159265358979,
              0.994515, 1e-5, "convergence southern hemisphere");

  check_throws<std::out_of_range> ([] { const utm u (85.0, 0.0); },
                                   "latitude outside utm");
}

void
utm_direction_test ()
{
  const utm origin (500000, 4000000, 36, 'S');
  const utm north_east (500010, 4000010, 36, 'S', utc{ 0, 0, 2 });
  const utm south_west (499990, 3999990, 36, 'S');

  check_near (north_east.get_direction (origin).dir_degree, 45, 1e-9,
              "bearing north east");
  check_near (south_west.get_direction (origin).dir_degree, 225, 1e-9,
              "bearing south west");
  check_near (north_east.get_distance (origin), std::sqrt (200.0), 1e-9,
              "distance");
  check_near (north_east.get_time_diff_sec (origin), 2, 1e-9, "time diff");
}

#endif // !UTM_TEST_HPP
