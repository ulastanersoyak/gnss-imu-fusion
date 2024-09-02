#ifndef RMC_HPP
#define RMC_HPP

#include "time_and_location/coordinate.hpp"
#include "time_and_location/date.hpp"
#include "time_and_location/utc.hpp"

#include <optional>
#include <stdexcept>
#include <string_view>
#include <vector>

// Thrown for well formed sentences that report no valid position fix.
class no_fix_error : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Verifies the "*hh" checksum of an NMEA sentence and returns its comma
// separated fields (empty fields included, '$' and checksum stripped).
// Anything before the '$' is ignored. Throws std::invalid_argument.
[[nodiscard]] std::vector<std::string_view>
nmea_fields (std::string_view sentence);

class rmc
{
  utc time_{ 0, 0, 0 };
  coordinate latitude_{};
  coordinate longitude_{};
  float speed_over_ground_knot_{ 0 };
  std::optional<float> course_over_ground_deg_{};
  date date_{};
  std::uint8_t checksum{ 0 };
  char fix_status_{ '_' };

public:
  // Throws std::invalid_argument on malformed sentences and no_fix_error
  // when the receiver has no fix.
  explicit rmc (std::string_view nmea_sentence) noexcept (false);

  // clang-format off
  rmc (utc time, coordinate lat, coordinate lon, float speed, date d,
       std::uint8_t checksum_, char fix,
       std::optional<float> course = std::nullopt)
      : time_ (time), latitude_ (lat), longitude_ (lon),
        speed_over_ground_knot_ (speed), course_over_ground_deg_ (course),
        date_ (d), checksum (checksum_), fix_status_ (fix){}
  // clang-format on

  [[nodiscard]] utc
  time () const noexcept
  {
    return time_;
  }
  [[nodiscard]] double speed_mps () const noexcept;
  // True course in degrees, clockwise from north. Receivers leave it empty
  // when they are (nearly) stationary.
  [[nodiscard]] std::optional<double> course_deg () const noexcept;

  friend bool operator== (const rmc &rmc1, const rmc &rmc2);
  friend class utm;
};

#endif // !RMC_HPP
