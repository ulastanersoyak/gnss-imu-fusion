// Connects the Gazebo simulation to the position estimator:
//
//   /vehicle/imu     (gz.msgs.IMU, 100 Hz)   -> Xbus    -> add_imu_sample
//   /vehicle/navsat  (gz.msgs.NavSat, 1 Hz)  -> gps_fix -> add_gps_fix
//   /model/vehicle/pose (gz.msgs.Pose_V)     -> ground truth for scoring
//
// After every IMU sample the filter predicts where the vehicle will be
// --horizon seconds later. That prediction is drawn as a translucent ghost
// and, once the simulation reaches the predicted time, compared against the
// true pose. Accuracy is printed to the terminal and drawn above the ghost.

#include "error_stats.hpp"
#include "local_frame.hpp"

#include "kalman/position_estimator.hpp"
#include "kalman/sensor_event.hpp"
#include "utility/queue.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <variant>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void
handle_signal (int /*signal*/)
{
  stop_requested = 1;
}

template <typename... Ts> struct overloaded : Ts...
{
  using Ts::operator()...;
};

constexpr double pi = std::numbers::pi;

struct options
{
  double horizon_s{ 1.0 };
  double gps_noise_m{ 1.5 };
  double gps_drift_m{ 0.0 };
  double gps_drift_tau_s{ 60.0 };
  double speed_noise_mps{ 0.05 };
  double origin_lat{ 39.97656 };
  double origin_lon{ 32.73812 };
  double origin_alt{ 894.0 };
  double duration_s{ 0.0 }; // 0 = run until Ctrl-C
  double radius_m{ 12.0 };
  unsigned seed{ 1 };
  bool manual{ false };
  bool follow{ true };
};

void
print_usage (const char *name)
{
  std::cout
      << "usage: " << name << " [options]\n"
      << "  --horizon S        predict S seconds ahead (default 1.0)\n"
      << "  --gps-noise M      white GPS position noise, metres (1.5)\n"
      << "  --gps-drift M      slowly wandering GPS error, metres (0)\n"
      << "  --duration S       stop after S simulated seconds (run forever)\n"
      << "  --radius M         figure-eight loop radius (12)\n"
      << "  --seed N           noise seed (1)\n"
      << "  --manual           do not drive, use the GUI teleop instead\n"
      << "  --no-follow        do not make the GUI camera follow the "
         "vehicle\n";
}

std::optional<options>
parse_options (int argc, char *argv[])
{
  options opt;
  for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      const auto value = [&] () -> double
        {
          if (i + 1 >= argc)
            {
              throw std::invalid_argument ("missing value for " + arg);
            }
          return std::stod (argv[++i]);
        };
      if (arg == "--horizon")
        opt.horizon_s = value ();
      else if (arg == "--gps-noise")
        opt.gps_noise_m = value ();
      else if (arg == "--gps-drift")
        opt.gps_drift_m = value ();
      else if (arg == "--duration")
        opt.duration_s = value ();
      else if (arg == "--radius")
        opt.radius_m = value ();
      else if (arg == "--seed")
        opt.seed = static_cast<unsigned> (value ());
      else if (arg == "--manual")
        opt.manual = true;
      else if (arg == "--no-follow")
        opt.follow = false;
      else
        return std::nullopt;
    }
  if (opt.horizon_s <= 0)
    {
      throw std::invalid_argument ("--horizon must be positive");
    }
  return opt;
}

double
stamp_seconds (const gz::msgs::Header &header)
{
  return static_cast<double> (header.stamp ().sec ())
         + (header.stamp ().nsec () * 1e-9);
}

utc
to_utc (double sim_seconds)
{
  const auto ms_total = static_cast<std::uint64_t> (sim_seconds * 1000.0);
  const auto ms_of_day = ms_total % 86400000ULL;
  return utc{ static_cast<std::uint8_t> (ms_of_day / 3600000),
              static_cast<std::uint8_t> ((ms_of_day / 60000) % 60),
              static_cast<std::uint8_t> ((ms_of_day / 1000) % 60),
              static_cast<std::uint16_t> (ms_of_day % 1000) };
}

double
wrap_angle (double a)
{
  while (a > pi)
    a -= 2 * pi;
  while (a < -pi)
    a += 2 * pi;
  return a;
}

struct xy
{
  double x{ 0 };
  double y{ 0 };
};

double
distance (const xy &a, const xy &b)
{
  return std::hypot (a.x - b.x, a.y - b.y);
}

// ---- events from the gz-transport callback threads ------------------------

struct imu_event
{
  double t;
  Xbus sample;
};

struct navsat_event
{
  double t;
  double lat;
  double lon;
  double alt;
  double v_east;
  double v_north;
};

struct truth_event
{
  double t;
  xy position;
  double yaw;
};

using sim_event = std::variant<imu_event, navsat_event, truth_event>;

// ---- the bridge -----------------------------------------------------------

class positioning_sim
{
  struct prediction
  {
    double target_t;
    xy filter;
    std::optional<xy> baseline; // raw GPS extrapolation, for comparison
  };

  options opt_;
  gz::transport::Node node_{};
  gz::transport::Node::Publisher cmd_pub_{};
  gz::transport::Node::Publisher accuracy_pub_{};
  gz::transport::Node::Publisher overlay_pub_{};
  queue<sim_event> events_{};
  local_frame frame_;
  position_estimator estimator_{};

  std::mt19937 rng_;
  std::normal_distribution<double> unit_noise_{ 0.0, 1.0 };
  xy gps_drift_{};
  double last_gps_t_{ -1 };

  std::deque<truth_event> truth_{};
  std::deque<prediction> pending_{};
  // Positions to score against the truth at the same instant once the
  // (slightly later) truth sample has arrived.
  struct instant_check
  {
    double t;
    xy position;
    error_stats *stats;
  };
  std::deque<instant_check> pending_checks_{};
  std::optional<navsat_event> last_raw_fix_{};
  std::optional<xy> last_raw_fix_xy_{};

  error_stats filter_error_{};
  error_stats baseline_error_{};
  error_stats now_error_{};
  error_stats raw_gps_error_{};
  std::size_t gps_fixes_{ 0 };
  std::size_t gps_rejected_{ 0 };

  double sim_t_{ 0 };
  double last_print_t_{ 0 };
  double last_ghost_t_{ -1 };
  double last_overlay_t_{ -1 };
  double last_cmd_t_{ -1 };
  double last_follow_t_{ -100 };

  // figure-eight driving
  double turn_direction_{ 1 };
  double turned_{ 0 };
  std::optional<double> last_yaw_{};

  std::optional<prediction> last_evaluated_{};
  std::optional<xy> last_evaluated_truth_{};

public:
  explicit positioning_sim (const options &opt)
      : opt_ (opt), frame_ (opt.origin_lat, opt.origin_lon, opt.origin_alt),
        rng_ (opt.seed)
  {
  }

  bool
  start ()
  {
    cmd_pub_ = node_.Advertise<gz::msgs::Twist> ("/model/vehicle/cmd_vel");
    accuracy_pub_
        = node_.Advertise<gz::msgs::StringMsg> ("/positioning/accuracy");
    overlay_pub_
        = node_.Advertise<gz::msgs::StringMsg> ("/positioning/overlay");

    const bool ok
        = node_.Subscribe ("/vehicle/imu", &positioning_sim::on_imu, this)
          && node_.Subscribe ("/vehicle/navsat", &positioning_sim::on_navsat,
                              this)
          && node_.Subscribe ("/model/vehicle/pose", &positioning_sim::on_pose,
                              this);
    if (!ok)
      {
        std::cerr << "failed to subscribe to the gazebo topics\n";
      }
    return ok;
  }

  void
  run ()
  {
    std::cout << "waiting for the simulation (is `gz sim` running?)...\n";
    bool announced = false;
    while (stop_requested == 0)
      {
        auto event = events_.pop_for (std::chrono::milliseconds (200));
        if (!event)
          {
            continue;
          }
        if (!announced)
          {
            announced = true;
            std::cout << "connected, predicting " << opt_.horizon_s
                      << " s ahead\n";
          }
        std::visit (
            overloaded{
                [this] (const imu_event &e) { handle (e); },
                [this] (const navsat_event &e) { handle (e); },
                [this] (const truth_event &e) { handle (e); },
            },
            *event);

        if (sim_t_ - last_print_t_ >= 1.0)
          {
            last_print_t_ = sim_t_;
            print_status ();
          }
        if (sim_t_ - last_overlay_t_ >= 0.2)
          {
            last_overlay_t_ = sim_t_;
            publish_overlay ();
          }
        if (opt_.duration_s > 0 && sim_t_ >= opt_.duration_s)
          {
            break;
          }
      }
    stop_vehicle ();
    print_summary ();
  }

private:
  // -- gz-transport callbacks (other threads): only convert and enqueue --

  void
  on_imu (const gz::msgs::IMU &msg)
  {
    const double t = stamp_seconds (msg.header ());
    Xbus sample{};
    const auto &q = msg.orientation ();
    sample.quat = { q.w (), q.x (), q.y (), q.z () };
    sample.quaternionAvailable = true;
    sample.quatFrame = xbus_frame::ENU;
    sample.convertQuatToEuler ();
    const auto &a = msg.linear_acceleration ();
    sample.acc = { a.x (), a.y (), a.z () };
    sample.accAvailable = true;
    const auto &w = msg.angular_velocity ();
    sample.rot = { w.x (), w.y (), w.z () };
    sample.rotAvailable = true;
    sample.sampleTimeFine = static_cast<std::uint32_t> (
        std::llround (t * Xbus::sampleTimeFineHz));
    sample.sampleTimeFineAvailable = true;
    events_.push (imu_event{ t, sample });
  }

  void
  on_navsat (const gz::msgs::NavSat &msg)
  {
    events_.push (navsat_event{ stamp_seconds (msg.header ()),
                                msg.latitude_deg (), msg.longitude_deg (),
                                msg.altitude (), msg.velocity_east (),
                                msg.velocity_north () });
  }

  void
  on_pose (const gz::msgs::Pose_V &msg)
  {
    for (const auto &pose : msg.pose ())
      {
        const auto &name = pose.name ();
        if (name != "vehicle")
          {
            continue;
          }
        const auto &q = pose.orientation ();
        const double yaw
            = std::atan2 (2 * (q.w () * q.z () + q.x () * q.y ()),
                          1 - 2 * (q.y () * q.y () + q.z () * q.z ()));
        const double t = pose.has_header () ? stamp_seconds (pose.header ())
                                            : stamp_seconds (msg.header ());
        events_.push (truth_event{
            t, { pose.position ().x (), pose.position ().y () }, yaw });
        return;
      }
  }

  // -- main thread -----------------------------------------------------

  [[nodiscard]] xy
  to_local (const utm &position) const
  {
    const auto [lat, lon] = position.to_latlon ();
    const auto enu = frame_.to_enu (lat, lon, opt_.origin_alt);
    return { enu[0], enu[1] };
  }

  void
  handle (const imu_event &e)
  {
    sim_t_ = std::max (sim_t_, e.t);
    if (!estimator_.add_imu_sample (e.sample))
      {
        return;
      }

    const auto now = estimator_.estimate ();
    const auto ahead = estimator_.predict_ahead (opt_.horizon_s);
    if (!now || !ahead)
      {
        return;
      }
    const xy now_xy = to_local (now->position);
    const xy ahead_xy = to_local (ahead->position);

    pending_checks_.push_back ({ e.t, now_xy, &now_error_ });

    prediction p{ e.t + opt_.horizon_s, ahead_xy, std::nullopt };
    if (last_raw_fix_ && last_raw_fix_xy_)
      {
        const double dt = p.target_t - last_raw_fix_->t;
        p.baseline = xy{ last_raw_fix_xy_->x + last_raw_fix_->v_east * dt,
                         last_raw_fix_xy_->y + last_raw_fix_->v_north * dt };
      }
    pending_.push_back (p);

    if (e.t - last_ghost_t_ >= 1.0 / 30.0)
      {
        last_ghost_t_ = e.t;
        draw_ghost (now_xy, ahead_xy);
      }
  }

  void
  handle (const navsat_event &e)
  {
    sim_t_ = std::max (sim_t_, e.t);
    const double dt = last_gps_t_ < 0 ? 0 : e.t - last_gps_t_;
    last_gps_t_ = e.t;

    // Receiver model: white noise plus an optional slowly wandering
    // (first order Gauss-Markov) error.
    if (opt_.gps_drift_m > 0 && dt > 0)
      {
        const double phi = std::exp (-dt / opt_.gps_drift_tau_s);
        const double s = opt_.gps_drift_m * std::sqrt (1 - phi * phi);
        gps_drift_.x = phi * gps_drift_.x + s * unit_noise_ (rng_);
        gps_drift_.y = phi * gps_drift_.y + s * unit_noise_ (rng_);
      }
    const double noise_e
        = gps_drift_.x + opt_.gps_noise_m * unit_noise_ (rng_);
    const double noise_n
        = gps_drift_.y + opt_.gps_noise_m * unit_noise_ (rng_);
    const double v_east = e.v_east + opt_.speed_noise_mps * unit_noise_ (rng_);
    const double v_north
        = e.v_north + opt_.speed_noise_mps * unit_noise_ (rng_);
    const double speed = std::hypot (v_east, v_north);

    gps_fix fix{};
    try
      {
        fix.position = utm (e.lat, e.lon, to_utc (e.t));
      }
    catch (const std::exception &ex)
      {
        std::cerr << "navsat: " << ex.what () << "\n";
        return;
      }
    fix.position.update_easting (noise_e);
    fix.position.update_northing (noise_n);
    fix.speed_mps = speed;
    if (speed >= 0.3) // receivers leave the course empty when slow
      {
        double course = std::atan2 (v_east, v_north) * 180 / pi;
        fix.course_deg = course < 0 ? course + 360 : course;
      }
    fix.hdop = std::max (opt_.gps_noise_m, 0.5) / 2.0; // sigma = hdop * 2 m

    const xy raw = to_local (fix.position);
    last_raw_fix_ = navsat_event{ e.t, e.lat, e.lon, e.alt, v_east, v_north };
    last_raw_fix_xy_ = raw;
    pending_checks_.push_back ({ e.t, raw, &raw_gps_error_ });

    ++gps_fixes_;
    const auto result = estimator_.add_gps_fix (fix);
    if (result == position_estimator::fix_result::rejected
        || result == position_estimator::fix_result::reset)
      {
        ++gps_rejected_;
      }
    draw_gps_fix (raw);
  }

  void
  handle (const truth_event &e)
  {
    sim_t_ = std::max (sim_t_, e.t);
    truth_.push_back (e);
    while (!truth_.empty () && truth_.front ().t < e.t - opt_.horizon_s - 5)
      {
        truth_.pop_front ();
      }

    evaluate_predictions ();
    drive (e);
    follow_vehicle ();
  }

  // Linear interpolation of the true position at time t.
  [[nodiscard]] std::optional<xy>
  truth_at (double t) const
  {
    if (truth_.size () < 2 || t < truth_.front ().t || t > truth_.back ().t)
      {
        return std::nullopt;
      }
    for (std::size_t i = 1; i < truth_.size (); ++i)
      {
        const auto &b = truth_[i];
        if (b.t < t)
          {
            continue;
          }
        const auto &a = truth_[i - 1];
        const double span = b.t - a.t;
        const double w = span > 0 ? (t - a.t) / span : 0;
        return xy{ a.position.x + w * (b.position.x - a.position.x),
                   a.position.y + w * (b.position.y - a.position.y) };
      }
    return std::nullopt;
  }

  void
  evaluate_predictions ()
  {
    while (!pending_checks_.empty () && !truth_.empty ()
           && pending_checks_.front ().t <= truth_.back ().t)
      {
        const auto check = pending_checks_.front ();
        pending_checks_.pop_front ();
        if (const auto truth = truth_at (check.t))
          {
            check.stats->add (distance (check.position, *truth));
          }
      }

    while (!pending_.empty () && !truth_.empty ()
           && pending_.front ().target_t <= truth_.back ().t)
      {
        const auto p = pending_.front ();
        pending_.pop_front ();
        const auto truth = truth_at (p.target_t);
        if (!truth)
          {
            continue; // too old, history already dropped
          }
        filter_error_.add (distance (p.filter, *truth));
        if (p.baseline)
          {
            baseline_error_.add (distance (*p.baseline, *truth));
          }
        last_evaluated_ = p;
        last_evaluated_truth_ = truth;
      }
  }

  // Figure eight: full circles alternating left and right, with the speed
  // varying between 1 and 4 m/s so the filter sees acceleration too.
  void
  drive (const truth_event &e)
  {
    if (last_yaw_)
      {
        turned_ += wrap_angle (e.yaw - *last_yaw_);
      }
    last_yaw_ = e.yaw;

    if (opt_.manual || e.t - last_cmd_t_ < 0.05)
      {
        return;
      }
    last_cmd_t_ = e.t;

    gz::msgs::Twist cmd;
    if (e.t > 3.0) // stand still for the first fix
      {
        const double speed = 2.5 + 1.5 * std::sin (2 * pi * e.t / 23.0);
        if (std::abs (turned_) >= 2 * pi)
          {
            turn_direction_ = -turn_direction_;
            turned_ = 0;
          }
        cmd.mutable_linear ()->set_x (speed);
        cmd.mutable_angular ()->set_z (turn_direction_ * speed
                                       / opt_.radius_m);
      }
    cmd_pub_.Publish (cmd);
  }

  void
  stop_vehicle ()
  {
    if (!opt_.manual)
      {
        cmd_pub_.Publish (gz::msgs::Twist{});
      }
  }

  // The GUI usually starts after us and may drop an early request, so keep
  // asking during the first half minute.
  void
  follow_vehicle ()
  {
    if (!opt_.follow || sim_t_ > 30 || sim_t_ - last_follow_t_ < 3)
      {
        return;
      }
    last_follow_t_ = sim_t_;

    gz::msgs::Vector3d offset;
    offset.set_x (-9);
    offset.set_y (-4);
    offset.set_z (6);
    gz::msgs::StringMsg target;
    target.set_data ("vehicle");
    std::function<void (const gz::msgs::Boolean &, const bool)> ignore
        = [] (const gz::msgs::Boolean &, const bool) {};
    node_.Request ("/gui/follow/offset", offset, ignore);
    node_.Request ("/gui/follow", target, ignore);
  }

  // -- drawing -----------------------------------------------------------

  static gz::msgs::Marker
  marker (const std::string &ns, std::uint64_t id, gz::msgs::Marker::Type type)
  {
    gz::msgs::Marker m;
    m.set_ns (ns);
    m.set_id (id);
    m.set_action (gz::msgs::Marker::ADD_MODIFY);
    m.set_type (type);
    m.set_visibility (gz::msgs::Marker::GUI);
    return m;
  }

  static void
  set_color (gz::msgs::Marker &m, double r, double g, double b, double a)
  {
    for (auto *c : { m.mutable_material ()->mutable_ambient (),
                     m.mutable_material ()->mutable_diffuse () })
      {
        c->set_r (static_cast<float> (r));
        c->set_g (static_cast<float> (g));
        c->set_b (static_cast<float> (b));
        c->set_a (static_cast<float> (a));
      }
  }

  static void
  set_pose (gz::msgs::Marker &m, double x, double y, double z, double yaw)
  {
    m.mutable_pose ()->mutable_position ()->set_x (x);
    m.mutable_pose ()->mutable_position ()->set_y (y);
    m.mutable_pose ()->mutable_position ()->set_z (z);
    m.mutable_pose ()->mutable_orientation ()->set_w (std::cos (yaw / 2));
    m.mutable_pose ()->mutable_orientation ()->set_z (std::sin (yaw / 2));
  }

  static void
  add_point (gz::msgs::Marker &m, double x, double y, double z)
  {
    auto *p = m.add_point ();
    p->set_x (x);
    p->set_y (y);
    p->set_z (z);
  }

  void
  send (const gz::msgs::Marker &m)
  {
    node_.Request ("/marker", m);
  }

  void
  draw_ghost (const xy &now, const xy &ahead)
  {
    // Ghost: same size as the chassis, coloured by the latest error.
    const double err = filter_error_.last ();
    const double badness = std::clamp ((err - 0.25) / 1.5, 0.0, 1.0);
    const double yaw = distance (now, ahead) > 0.05
                           ? std::atan2 (ahead.y - now.y, ahead.x - now.x)
                           : last_yaw_.value_or (0);

    auto ghost = marker ("ghost", 1, gz::msgs::Marker::BOX);
    set_pose (ghost, ahead.x, ahead.y, 0.4, yaw);
    ghost.mutable_scale ()->set_x (1.6);
    ghost.mutable_scale ()->set_y (1.0);
    ghost.mutable_scale ()->set_z (0.4);
    set_color (ghost, 0.2 + 0.8 * badness, 1.0 - 0.7 * badness, 0.2, 0.55);
    auto *glow = ghost.mutable_material ()->mutable_emissive ();
    glow->set_r (static_cast<float> (0.1 + 0.6 * badness));
    glow->set_g (static_cast<float> (0.6 - 0.5 * badness));
    glow->set_b (0.1F);
    glow->set_a (0.55F);
    send (ghost);

    auto link = marker ("ghost", 2, gz::msgs::Marker::LINE_STRIP);
    add_point (link, now.x, now.y, 0.9);
    add_point (link, ahead.x, ahead.y, 0.9);
    set_color (link, 1, 1, 1, 0.8);
    send (link);

    // Where an earlier ghost said we would be now, and where we really are.
    if (last_evaluated_ && last_evaluated_truth_)
      {
        auto past = marker ("check", 1, gz::msgs::Marker::SPHERE);
        set_pose (past, last_evaluated_->filter.x, last_evaluated_->filter.y,
                  0.9, 0);
        past.mutable_scale ()->set_x (0.25);
        past.mutable_scale ()->set_y (0.25);
        past.mutable_scale ()->set_z (0.25);
        set_color (past, 1, 0.5, 0, 0.9);
        send (past);

        auto error_line = marker ("check", 2, gz::msgs::Marker::LINE_LIST);
        add_point (error_line, last_evaluated_->filter.x,
                   last_evaluated_->filter.y, 0.9);
        add_point (error_line, last_evaluated_truth_->x,
                   last_evaluated_truth_->y, 0.9);
        set_color (error_line, 1, 0.1, 0.1, 1);
        send (error_line);
      }
  }

  void
  draw_gps_fix (const xy &raw)
  {
    // The last 20 raw GPS fixes as small dots.
    auto dot = marker ("gps", gps_fixes_ % 20, gz::msgs::Marker::SPHERE);
    set_pose (dot, raw.x, raw.y, 0.1, 0);
    dot.mutable_scale ()->set_x (0.3);
    dot.mutable_scale ()->set_y (0.3);
    dot.mutable_scale ()->set_z (0.3);
    set_color (dot, 0.9, 0.9, 0.1, 0.9);
    send (dot);
  }

  // -- reporting ---------------------------------------------------------

  void
  print_status ()
  {
    std::ostringstream line;
    line << std::fixed << std::setprecision (2) << "t=" << std::setw (6)
         << sim_t_ << "s | ghost +" << opt_.horizon_s << "s err: last "
         << filter_error_.last () << " m, mean " << filter_error_.mean ()
         << ", rms " << filter_error_.rms () << ", p95 "
         << filter_error_.percentile (0.95) << ", max " << filter_error_.max ()
         << " | raw GPS extrapolated: mean " << baseline_error_.mean ()
         << " m | fixes " << gps_fixes_ << " (rejected " << gps_rejected_
         << ")";
    std::cout << line.str () << std::endl;

    gz::msgs::StringMsg msg;
    msg.set_data (line.str ());
    accuracy_pub_.Publish (msg);
  }

  // Rich text for the AccuracyOverlay GUI plugin.
  void
  publish_overlay ()
  {
    const double err = filter_error_.last ();
    const char *colour
        = err < 0.5 ? "#7cfc7c" : (err < 1.5 ? "#ffd966" : "#ff6b6b");
    std::ostringstream o;
    o << std::fixed << std::setprecision (2)
      << "<b>Ghost = Kalman prediction +" << opt_.horizon_s << " s</b><br>"
      << "error now: <font color=\"" << colour << "\"><b>" << err
      << " m</b></font><br>"
      << "mean " << filter_error_.mean () << " m | rms "
      << filter_error_.rms () << " m | p95 " << filter_error_.percentile (0.95)
      << " m | max " << filter_error_.max () << " m<br>"
      << "<font color=\"#aaaaaa\">raw GPS extrapolated +" << opt_.horizon_s
      << " s: mean " << baseline_error_.mean () << " m<br>"
      << "current position: Kalman " << now_error_.mean () << " m vs raw GPS "
      << raw_gps_error_.mean () << " m<br>"
      << "GPS fixes " << gps_fixes_ << " (rejected " << gps_rejected_
      << ") | sim time " << std::setprecision (1) << sim_t_ << " s</font>";

    gz::msgs::StringMsg msg;
    msg.set_data (o.str ());
    overlay_pub_.Publish (msg);
  }

  void
  print_summary () const
  {
    const auto row = [] (const char *name, const error_stats &s)
      {
        std::cout << "  " << std::left << std::setw (38) << name << std::right
                  << std::fixed << std::setprecision (3) << std::setw (8)
                  << s.mean () << std::setw (8) << s.rms () << std::setw (8)
                  << s.percentile (0.5) << std::setw (8) << s.percentile (0.95)
                  << std::setw (8) << s.max () << std::setw (9) << s.count ()
                  << "\n";
      };

    std::cout << "\n=== accuracy over " << std::fixed << std::setprecision (1)
              << sim_t_ << " s of simulation (metres) ===\n"
              << "  " << std::left << std::setw (38) << "" << std::right
              << std::setw (8) << "mean" << std::setw (8) << "rms"
              << std::setw (8) << "p50" << std::setw (8) << "p95"
              << std::setw (8) << "max" << std::setw (9) << "samples"
              << "\n";
    std::ostringstream ghost;
    ghost << "ghost: Kalman prediction +" << std::setprecision (2)
          << opt_.horizon_s << " s";
    std::ostringstream baseline;
    baseline << "baseline: raw GPS extrapolated +" << std::setprecision (2)
             << opt_.horizon_s << " s";
    row (ghost.str ().c_str (), filter_error_);
    row (baseline.str ().c_str (), baseline_error_);
    row ("Kalman estimate of the current pos.", now_error_);
    row ("raw GPS fix", raw_gps_error_);
    std::cout << "  gps fixes: " << gps_fixes_
              << ", rejected by the filter: " << gps_rejected_ << "\n";
  }
};
} // namespace

int
main (int argc, char *argv[])
{
  std::optional<options> opt;
  try
    {
      opt = parse_options (argc, argv);
    }
  catch (const std::exception &e)
    {
      std::cerr << e.what () << "\n";
    }
  if (!opt)
    {
      print_usage (argv[0]);
      return 2;
    }

  std::signal (SIGINT, handle_signal);
  std::signal (SIGTERM, handle_signal);

  positioning_sim sim (*opt);
  if (!sim.start ())
    {
      return 1;
    }
  sim.run ();
  return 0;
}
