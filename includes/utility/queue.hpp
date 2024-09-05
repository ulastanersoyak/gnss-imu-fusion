#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

template <typename contained> class queue
{
  std::queue<contained> q{};
  mutable std::mutex mtx{};
  std::condition_variable cv{};

public:
  void
  push (contained cont)
  {
    {
      std::lock_guard<std::mutex> lock (this->mtx);
      q.push (std::move (cont));
    }
    cv.notify_one ();
  }

  contained
  pop ()
  {
    std::unique_lock<std::mutex> lock (this->mtx);
    this->cv.wait (lock, [this] { return !q.empty (); });
    contained item = std::move (q.front ());
    q.pop ();
    return item;
  }

  // Like pop(), but gives up after timeout.
  template <typename Rep, typename Period>
  std::optional<contained>
  pop_for (std::chrono::duration<Rep, Period> timeout)
  {
    std::unique_lock<std::mutex> lock (this->mtx);
    if (!this->cv.wait_for (lock, timeout, [this] { return !q.empty (); }))
      {
        return std::nullopt;
      }
    contained item = std::move (q.front ());
    q.pop ();
    return item;
  }

  [[nodiscard]] std::size_t
  size () const
  {
    std::lock_guard<std::mutex> lock (this->mtx);
    return q.size ();
  }
};

#endif // !QUEUE_HPP
