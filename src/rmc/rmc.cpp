#include "rmc/rmc.hpp"
#include "time_and_location/parser.hpp"

#include <cctype>
#include <charconv>
#include <cmath>
#include <string>

namespace
{
constexpr double knot_to_mps = 1852.0 / 3600.0;

float
parse_float (std::string_view token)
{
  float value{ 0 };
  const char *end = token.data () + token.size ();
  const auto [ptr, ec] = std::from_chars (token.data (), end, value);
  if (ec != std::errc{} || ptr != end)
    {
      throw std::invalid_argument ("not a number: " + std::string (token));
    }
  return value;
}
} // namespace

[[nodiscard]] std::vector<std::string_view>
nmea_fields (std::string_view sentence)
{
  while (!sentence.empty ()
         && std::isspace (static_cast<unsigned char> (sentence.back ())))
    {
      sentence.remove_suffix (1);
    }

  const auto start = sentence.find ('$');
  if (start == std::string_view::npos)
    {
      throw std::invalid_argument ("no '$' in nmea sentence");
    }
  sentence.remove_prefix (start);

  const auto star = sentence.rfind ('*');
  if (star == std::string_view::npos || star + 3 != sentence.size ())
    {
      throw std::invalid_argument ("missing nmea checksum");
    }

  const auto body = sentence.substr (1, star - 1);
  std::uint8_t computed{ 0 };
  for (const char c : body)
    {
      computed ^= static_cast<std::uint8_t> (c);
    }

  std::uint8_t expected{ 0 };
  const auto hex = sentence.substr (star + 1);
  const auto [ptr, ec]
      = std::from_chars (hex.data (), hex.data () + hex.size (), expected, 16);
  if (ec != std::errc{} || ptr != hex.data () + hex.size ())
    {
      throw std::invalid_argument ("malformed nmea checksum");
    }
  if (computed != expected)
    {
      throw std::invalid_argument ("nmea checksum mismatch");
    }

  std::vector<std::string_view> fields;
  std::size_t pos = 0;
  while (true)
    {
      const auto comma = body.find (',', pos);
      fields.push_back (body.substr (pos, comma - pos));
      if (comma == std::string_view::npos)
        {
          break;
        }
      pos = comma + 1;
    }
  return fields;
}

rmc::rmc (std::string_view nmea_sentence) noexcept (false)
{
  // clang-format off
  // $GNRMC,072224.00,A,3958.59177,N,03244.28699,E,0.384,237.98,260724,,,A,V*01
  //   0        1     2      3     4      5      6   7     8      9   10 11 12 13
  // clang-format on
  const auto fields = nmea_fields (nmea_sentence);
  if (fields.size () < 12 || fields[0].size () != 5
      || fields[0].substr (2) != "RMC")
    {
      throw std::invalid_argument ("not an rmc sentence");
    }

  // Field 12 (NMEA 2.3+) is the mode indicator, 'N' means data not valid.
  if (fields[2] != "A" || (fields.size () > 12 && fields[12] == "N"))
    {
      throw no_fix_error ("rmc sentence without a valid fix");
    }

  this->time_ = parse_utc (fields[1]);
  this->latitude_
      = parse_coordinate (std::string (fields[3]).append (fields[4]));
  this->longitude_
      = parse_coordinate (std::string (fields[5]).append (fields[6]));
  this->speed_over_ground_knot_
      = fields[7].empty () ? 0.0F : parse_float (fields[7]);
  if (!fields[8].empty ())
    {
      this->course_over_ground_deg_ = parse_float (fields[8]);
    }
  this->date_ = parse_date (fields[9]);
  const auto star = nmea_sentence.rfind ('*');
  this->checksum = static_cast<std::uint8_t> (std::stoi (
      std::string (nmea_sentence.substr (star + 1, 2)), nullptr, 16));
  this->fix_status_ = fields[2][0];
}

[[nodiscard]] double
rmc::speed_mps () const noexcept
{
  return speed_over_ground_knot_ * knot_to_mps;
}

[[nodiscard]] std::optional<double>
rmc::course_deg () const noexcept
{
  if (course_over_ground_deg_)
    {
      return *course_over_ground_deg_;
    }
  return std::nullopt;
}

bool
operator== (const rmc &rmc1, const rmc &rmc2)
{
  return (rmc1.time_ == rmc2.time_) && (rmc1.latitude_ == rmc2.latitude_)
         && (rmc1.longitude_ == rmc2.longitude_) && (rmc1.date_ == rmc2.date_)
         && (rmc1.checksum == rmc2.checksum)
         && (rmc1.fix_status_ == rmc2.fix_status_);
}
