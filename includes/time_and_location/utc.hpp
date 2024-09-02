#ifndef UTC_HPP
#define UTC_HPP

#include <compare>
#include <cstdint>

#include <iostream>

class utc
{
  std::uint8_t hour{ 0 };
  std::uint8_t minute{ 0 };
  std::uint8_t second{ 0 };
  std::uint16_t millisecond{ 0 };

public:
  utc () = default;
  // clang-format off
  constexpr utc (std::uint8_t hour_, std::uint8_t minute_, std::uint8_t second_,
                 std::uint16_t millisecond_ = 0)
      : hour{ hour_ }, minute{ minute_ }, second{ second_ },
        millisecond{ millisecond_ } {}
  // clang-format on

  [[nodiscard]] auto operator<=> (const utc &other) const noexcept = default;

  // Signed difference (this - other) in seconds. Wraps across midnight, so
  // the result is always in [-43200, 43200).
  [[nodiscard]] double operator- (const utc &other) const noexcept;

  friend std::ostream &operator<< (std::ostream &os, const utc &utc);
};

#endif // !UTC_HPP
