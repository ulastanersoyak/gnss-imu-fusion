#include "time_and_location/utc.hpp"

#include <iomanip>

[[nodiscard]] double
utc::operator- (const utc &other) const noexcept
{
  constexpr double seconds_per_day = 86400.0;
  const auto to_seconds = [] (const utc &u)
    {
      return (u.hour * 3600.0) + (u.minute * 60.0) + u.second
             + (u.millisecond / 1000.0);
    };

  double diff = to_seconds (*this) - to_seconds (other);
  if (diff >= seconds_per_day / 2)
    {
      diff -= seconds_per_day;
    }
  else if (diff < -seconds_per_day / 2)
    {
      diff += seconds_per_day;
    }
  return diff;
}

std::ostream &
operator<< (std::ostream &os, const utc &utc)
{
  const auto fill = os.fill ('0');
  os << std::setw (2) << static_cast<int> (utc.hour) << ':' << std::setw (2)
     << static_cast<int> (utc.minute) << ':' << std::setw (2)
     << static_cast<int> (utc.second) << '.' << std::setw (3)
     << utc.millisecond;
  os.fill (fill);
  return os;
}
