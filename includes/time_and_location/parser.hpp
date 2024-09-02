#ifndef PARSER_HPP
#define PARSER_HPP

#include "time_and_location/coordinate.hpp"
#include "time_and_location/date.hpp"
#include "time_and_location/utc.hpp"

#include <string_view>
#include <variant>

// All parsers throw std::invalid_argument on malformed input.

// "hhmmss" or "hhmmss.ss"
[[nodiscard]] utc parse_utc (std::string_view token);

// "ddmmyy"
[[nodiscard]] date parse_date (std::string_view token);

// A 6 character token is a date, anything else is a utc time.
[[nodiscard]] std::variant<utc, date>
parse_date_and_utc (std::string_view token);

// "ddmm.mmmm,N", "dddmm.mmmm,E" or the same without the comma.
[[nodiscard]] coordinate parse_coordinate (std::string_view token);

#endif // !PARSER_HPP
