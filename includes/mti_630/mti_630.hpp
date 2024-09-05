#ifndef SERIALHANDLER_HPP
#define SERIALHANDLER_HPP

#include "kalman/sensor_event.hpp"
#include "utility/queue.hpp"
#include "utility/serial_port.hpp"

#include <stop_token>
#include <string_view>

class mti_630
{
  serial_port port_;

public:
  // Throws boost::system::system_error if the port cannot be opened.
  mti_630 (std::string_view port_name, std::uint32_t baud_rate,
           boost::asio::serial_port_base::parity::type parity,
           boost::asio::serial_port_base::stop_bits::type stop_bits,
           boost::asio::serial_port_base::character_size char_size);

  // Pushes every valid MTData2 message as an Xbus event until stopped.
  void work (const std::stop_token &stop, queue<sensor_event> &events);
};

#endif // SERIALHANDLER_HPP
