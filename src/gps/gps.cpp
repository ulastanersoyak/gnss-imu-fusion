#include "gps/gps.hpp"
#include "rmc/rmc.hpp"
#include "utm/utm.hpp"

#include <array>
#include <charconv>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

gps::gps (std::string_view port_name, std::uint32_t baud_rate,
          boost::asio::serial_port_base::parity::type parity,
          boost::asio::serial_port_base::stop_bits::type stop_bits,
          boost::asio::serial_port_base::character_size char_size)
    : port_{ std::make_unique<serial_port> (port_name, baud_rate, parity,
                                            stop_bits, char_size) }
{
}

gps::gps (const std::filesystem::path &replay_file) : replay_{ replay_file }
{
  if (!replay_.is_open ())
    {
      throw std::runtime_error ("cannot open replay file "
                                + replay_file.string ());
    }
}

std::optional<std::string>
gps::next_line (const std::stop_token &stop)
{
  if (!port_)
    {
      std::string line;
      if (std::getline (replay_, line))
        {
          return line;
        }
      return std::nullopt;
    }

  constexpr std::size_t max_line_length = 1024;
  std::array<std::uint8_t, 256> buffer{};
  while (!stop.stop_requested ())
    {
      const auto newline = pending_.find ('\n');
      if (newline != std::string::npos)
        {
          std::string line = pending_.substr (0, newline);
          pending_.erase (0, newline + 1);
          return line;
        }
      if (pending_.size () > max_line_length)
        {
          pending_.clear (); // garbage on the line, resync on next '\n'
        }

      const auto n
          = port_->read_some (buffer, std::chrono::milliseconds (100));
      for (std::size_t i = 0; i < n; ++i)
        {
          pending_.push_back (static_cast<char> (buffer[i]));
        }
    }
  return std::nullopt;
}

[[nodiscard]] std::optional<gps_fix>
gps::process_sentence (std::string_view sentence)
{
  const auto dollar = sentence.find ('$');
  if (dollar == std::string_view::npos || sentence.size () < dollar + 6)
    {
      return std::nullopt;
    }
  const auto type = sentence.substr (dollar + 3, 3);

  if (type == "GGA")
    {
      try
        {
          // $GNGGA,072223.00,3958.59211,N,03244.28744,E,1,05,2.30,...
          const auto fields = nmea_fields (sentence);
          if (fields.size () > 8 && !fields[8].empty ())
            {
              double hdop{ 0 };
              const auto f = fields[8];
              const auto [ptr, ec]
                  = std::from_chars (f.data (), f.data () + f.size (), hdop);
              if (ec == std::errc{} && ptr == f.data () + f.size ())
                {
                  hdop_ = hdop;
                }
            }
        }
      catch (const std::invalid_argument &)
        {
          // corrupted sentence, keep the previous hdop
        }
      return std::nullopt;
    }

  if (type != "RMC")
    {
      return std::nullopt;
    }

  try
    {
      const rmc sentence_ (sentence);
      return gps_fix{ .position = utm (sentence_),
                      .speed_mps = sentence_.speed_mps (),
                      .course_deg = sentence_.course_deg (),
                      .hdop = hdop_ };
    }
  catch (const no_fix_error &)
    {
      return std::nullopt;
    }
  catch (const std::exception &e)
    {
      std::cerr << "gps: dropping sentence (" << e.what () << ")\n";
      return std::nullopt;
    }
}

void
gps::work (const std::stop_token &stop, queue<sensor_event> &events)
{
  while (!stop.stop_requested ())
    {
      std::optional<std::string> line;
      try
        {
          line = next_line (stop);
        }
      catch (const std::exception &e)
        {
          std::cerr << "gps: read error (" << e.what () << ")\n";
          std::this_thread::sleep_for (std::chrono::seconds (1));
          continue;
        }

      if (!line)
        {
          if (!port_)
            {
              events.push (end_of_stream{});
              return;
            }
          continue;
        }

      if (auto fix = process_sentence (*line))
        {
          events.push (std::move (*fix));
        }
    }
}
