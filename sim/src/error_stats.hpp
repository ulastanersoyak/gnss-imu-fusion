#ifndef ERROR_STATS_HPP
#define ERROR_STATS_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

// Running statistics of a positive error (metres).
class error_stats
{
  std::vector<double> errors_{};
  double sum_{ 0 };
  double sum_sq_{ 0 };
  double max_{ 0 };

public:
  void
  add (double error)
  {
    errors_.push_back (error);
    sum_ += error;
    sum_sq_ += error * error;
    max_ = std::max (max_, error);
  }

  [[nodiscard]] std::size_t
  count () const noexcept
  {
    return errors_.size ();
  }

  [[nodiscard]] double
  last () const noexcept
  {
    return errors_.empty () ? 0.0 : errors_.back ();
  }

  [[nodiscard]] double
  mean () const noexcept
  {
    return errors_.empty () ? 0.0 : sum_ / static_cast<double> (count ());
  }

  [[nodiscard]] double
  rms () const noexcept
  {
    return errors_.empty ()
               ? 0.0
               : std::sqrt (sum_sq_ / static_cast<double> (count ()));
  }

  [[nodiscard]] double
  max () const noexcept
  {
    return max_;
  }

  // p in [0, 1]
  [[nodiscard]] double
  percentile (double p) const
  {
    if (errors_.empty ())
      {
        return 0.0;
      }
    auto sorted = errors_;
    const auto index = static_cast<std::size_t> (
        p * static_cast<double> (sorted.size () - 1));
    std::nth_element (sorted.begin (),
                      sorted.begin () + static_cast<std::ptrdiff_t> (index),
                      sorted.end ());
    return sorted[index];
  }
};

#endif // !ERROR_STATS_HPP
