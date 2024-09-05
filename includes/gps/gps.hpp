#ifndef GPS_HPP
#define GPS_HPP

#include "kalman/sensor_event.hpp"
#include "utility/queue.hpp"
#include "utility/serial_port.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>

class gps
{
  std::unique_ptr<serial_port> port_{};
  std::ifstream replay_{};
  std::string pending_{};
  std::optional<double> hdop_{};

  // Next line from the port or the replay file. nullopt when stopped or when
  // the replay file is exhausted.
  std::optional<std::string> next_line (const std::stop_token &stop);

public:
  // Live receiver. Throws boost::system::system_error if the port cannot be
  // opened.
  gps (std::string_view port_name, std::uint32_t baud_rate,
       boost::asio::serial_port_base::parity::type parity,
       boost::asio::serial_port_base::stop_bits::type stop_bits,
       boost::asio::serial_port_base::character_size char_size);

  // Replays a recorded NMEA log, one sentence per line (anything before '$'
  // is ignored). Throws std::runtime_error if the file cannot be opened.
  explicit gps (const std::filesystem::path &replay_file);

  // Feeds one NMEA sentence. Returns a fix for valid RMC sentences, GGA
  // sentences only update the HDOP attached to later fixes.
  [[nodiscard]] std::optional<gps_fix>
  process_sentence (std::string_view sentence);

  // Pushes fixes until stopped. A replay pushes end_of_stream when done.
  void work (const std::stop_token &stop, queue<sensor_event> &events);
};

#endif // GPS_HPP
