#ifndef CHECK_HPP
#define CHECK_HPP

#include <cmath>
#include <stdexcept>
#include <string>

inline void
check (bool condition, const std::string &message)
{
  if (!condition)
    {
      throw std::runtime_error (message);
    }
}

inline void
check_near (double actual, double expected, double tolerance,
            const std::string &message)
{
  if (!(std::abs (actual - expected) <= tolerance))
    {
      throw std::runtime_error (message + ": expected "
                                + std::to_string (expected) + ", got "
                                + std::to_string (actual));
    }
}

template <typename Exception, typename Fn>
void
check_throws (Fn &&fn, const std::string &message)
{
  try
    {
      fn ();
    }
  catch (const Exception &)
    {
      return;
    }
  throw std::runtime_error (message + ": expected an exception");
}

#endif // !CHECK_HPP
