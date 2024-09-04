#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "kalman/matrix.hpp"

#include <cstddef>

// Linear Kalman filter with N states.
template <std::size_t N> class kalman_filter
{
  vector<N> x_{};
  matrix<N, N> P_{};

public:
  kalman_filter () = default;
  kalman_filter (const vector<N> &x0, const matrix<N, N> &P0)
      : x_ (x0), P_ (P0)
  {
  }

  [[nodiscard]] const vector<N> &
  state () const noexcept
  {
    return x_;
  }

  [[nodiscard]] const matrix<N, N> &
  covariance () const noexcept
  {
    return P_;
  }

  // x = F x + u, P = F P F' + Q. u is the (already integrated) effect of any
  // known control input such as measured acceleration.
  void
  predict (const matrix<N, N> &F, const matrix<N, N> &Q,
           const vector<N> &u = {})
  {
    x_ = F * x_ + u;
    P_ = F * P_ * F.transpose () + Q;
  }

  // Normalised innovation squared y' S^-1 y of measurement z. Follows a
  // chi-square distribution with M degrees of freedom when the filter is
  // consistent, so it can be used to gate outliers before update().
  template <std::size_t M>
  [[nodiscard]] double
  nis (const vector<M> &z, const matrix<M, N> &H, const matrix<M, M> &R) const
  {
    const vector<M> y = z - H * x_;
    const matrix<M, M> S = H * P_ * H.transpose () + R;
    return (y.transpose () * inverse (S) * y) (0, 0);
  }

  template <std::size_t M>
  void
  update (const vector<M> &z, const matrix<M, N> &H, const matrix<M, M> &R)
  {
    const vector<M> y = z - H * x_;
    const matrix<M, M> S = H * P_ * H.transpose () + R;
    const matrix<N, M> K = P_ * H.transpose () * inverse (S);
    x_ += K * y;

    // Joseph form keeps P symmetric and positive definite.
    const matrix<N, N> I_KH = matrix<N, N>::identity () - K * H;
    P_ = I_KH * P_ * I_KH.transpose () + K * R * K.transpose ();
  }
};

#endif // !KALMAN_FILTER_HPP
