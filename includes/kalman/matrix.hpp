#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <utility>

// Minimal fixed size, row major matrix for small Kalman filters.
template <std::size_t R, std::size_t C> class matrix
{
  std::array<double, R * C> data_{};

public:
  constexpr matrix () = default;

  constexpr matrix (std::initializer_list<double> values)
  {
    if (values.size () != R * C)
      {
        throw std::invalid_argument ("matrix initializer size mismatch");
      }
    std::size_t i = 0;
    for (const double v : values)
      {
        data_[i++] = v;
      }
  }

  static constexpr matrix
  identity ()
  requires (R == C)
  {
    matrix m;
    for (std::size_t i = 0; i < R; ++i)
      {
        m (i, i) = 1.0;
      }
    return m;
  }

  constexpr double &
  operator() (std::size_t r, std::size_t c)
  {
    return data_[r * C + c];
  }

  constexpr double
  operator() (std::size_t r, std::size_t c) const
  {
    return data_[r * C + c];
  }

  [[nodiscard]] constexpr matrix<C, R>
  transpose () const
  {
    matrix<C, R> t;
    for (std::size_t r = 0; r < R; ++r)
      {
        for (std::size_t c = 0; c < C; ++c)
          {
            t (c, r) = (*this) (r, c);
          }
      }
    return t;
  }

  constexpr matrix &
  operator+= (const matrix &other)
  {
    for (std::size_t i = 0; i < R * C; ++i)
      {
        data_[i] += other.data_[i];
      }
    return *this;
  }

  constexpr matrix &
  operator-= (const matrix &other)
  {
    for (std::size_t i = 0; i < R * C; ++i)
      {
        data_[i] -= other.data_[i];
      }
    return *this;
  }
};

template <std::size_t N> using vector = matrix<N, 1>;

template <std::size_t R, std::size_t C>
constexpr matrix<R, C>
operator+ (matrix<R, C> lhs, const matrix<R, C> &rhs)
{
  return lhs += rhs;
}

template <std::size_t R, std::size_t C>
constexpr matrix<R, C>
operator- (matrix<R, C> lhs, const matrix<R, C> &rhs)
{
  return lhs -= rhs;
}

template <std::size_t R, std::size_t K, std::size_t C>
constexpr matrix<R, C>
operator* (const matrix<R, K> &lhs, const matrix<K, C> &rhs)
{
  matrix<R, C> out;
  for (std::size_t r = 0; r < R; ++r)
    {
      for (std::size_t c = 0; c < C; ++c)
        {
          double sum = 0.0;
          for (std::size_t k = 0; k < K; ++k)
            {
              sum += lhs (r, k) * rhs (k, c);
            }
          out (r, c) = sum;
        }
    }
  return out;
}

// Gauss-Jordan elimination with partial pivoting. Throws std::domain_error
// for (numerically) singular matrices.
template <std::size_t N>
matrix<N, N>
inverse (matrix<N, N> a)
{
  auto inv = matrix<N, N>::identity ();
  for (std::size_t col = 0; col < N; ++col)
    {
      std::size_t pivot = col;
      for (std::size_t r = col + 1; r < N; ++r)
        {
          if (std::abs (a (r, col)) > std::abs (a (pivot, col)))
            {
              pivot = r;
            }
        }
      if (std::abs (a (pivot, col)) < 1e-12)
        {
          throw std::domain_error ("singular matrix");
        }
      if (pivot != col)
        {
          for (std::size_t c = 0; c < N; ++c)
            {
              std::swap (a (col, c), a (pivot, c));
              std::swap (inv (col, c), inv (pivot, c));
            }
        }

      const double diagonal = a (col, col);
      for (std::size_t c = 0; c < N; ++c)
        {
          a (col, c) /= diagonal;
          inv (col, c) /= diagonal;
        }

      for (std::size_t r = 0; r < N; ++r)
        {
          if (r == col)
            {
              continue;
            }
          const double factor = a (r, col);
          for (std::size_t c = 0; c < N; ++c)
            {
              a (r, c) -= factor * a (col, c);
              inv (r, c) -= factor * inv (col, c);
            }
        }
    }
  return inv;
}

#endif // !MATRIX_HPP
