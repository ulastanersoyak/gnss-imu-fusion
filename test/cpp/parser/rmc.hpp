#ifndef RMC_TEST_HPP
#define RMC_TEST_HPP

#include <array>
#include <stdexcept>
#include <string>

#include "check.hpp"
#include "rmc/rmc.hpp"

void
rmc_construction_test ()
{
  std::array<std::string, 3> mmea_msgs{
    "$GNRMC,072244.00,A,3958.59794,N,03244.29486,E,0.113,,260724,,,A,V*14",
    "$GNRMC,072318.00,A,3958.59448,N,03244.28742,E,0.115,,260724,,,A,V*12",
    "$GNRMC,072247.00,A,3958.59798,N,03244.29466,E,0.087,,260724,,,A,V*19",
  };
  // clang-format off
std::array<rmc, 3> rmc_msgs{{
    { {7, 22, 44}, {39, 58.59794, 'N'}, {32, 44.29486, 'E'}, 0.113F, {26, 7, 24}, 20, 'A' },
    { {7, 23, 18}, {39, 58.59448, 'N'}, {32, 44.28742, 'E'}, 0.115F, {26, 7, 24}, 18, 'A' },
    { {7, 22, 47}, {39, 58.59798, 'N'}, {32, 44.29466, 'E'}, 0.087F, {26, 7, 24}, 25, 'A' }
  }};
  // clang-format on
  //
  for (std::uint8_t i = 0; i < 3; i++)
    {
      if (rmc_msgs.at (i) != rmc{ mmea_msgs.at (i) })
        {
          throw std::runtime_error ("rmc_construction_test failed");
        }
    }
}

void
rmc_sentence_test ()
{
  // Moving: the course field is filled in.
  const rmc moving{ "message715 := "
                    "$GNRMC,072224.00,A,3958.59177,N,03244.28699,E,0.384,"
                    "237.98,260724,,,A,V*01\r" };
  check (moving.course_deg ().has_value (), "course parsed");
  check_near (*moving.course_deg (), 237.98, 1e-4, "course value");
  check_near (moving.speed_mps (), 0.384 * 1852.0 / 3600.0, 1e-6, "speed");
  check (moving.time () == utc{ 7, 22, 24 }, "rmc time");

  check_throws<no_fix_error> (
      [] { const rmc r{ "$GNRMC,,V,,,,,,,,,,N,V*37" }; },
      "sentence without fix");
  check_throws<std::invalid_argument> (
      []
        {
          const rmc r{ "$GNRMC,072224.00,A,3958.59177,N,03244.28699,E,0.384,"
                       "237.98,260724,,,A,V*02" };
        },
      "checksum mismatch");
  check_throws<std::invalid_argument> (
      [] { const rmc r{ "$GNRMC,072224.00,A,3958.59177,N*" }; },
      "truncated sentence");
}

#endif // !RMC_TEST_HPP
