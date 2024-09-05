#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>

#include <chrono>
#include <cstdint>
#include <span>
#include <string_view>

// Blocking serial port with read timeouts, so reader threads can notice a
// stop request instead of blocking forever.
class serial_port
{
  boost::asio::io_context context_{};
  boost::asio::serial_port port_;

public:
  // Throws boost::system::system_error if the port cannot be opened.
  serial_port (std::string_view port_name, std::uint32_t baud_rate,
               boost::asio::serial_port_base::parity::type parity,
               boost::asio::serial_port_base::stop_bits::type stop_bits,
               boost::asio::serial_port_base::character_size char_size);

  serial_port (serial_port &&other) = delete;
  serial_port (const serial_port &) = delete;
  serial_port &operator= (serial_port &&other) = delete;
  serial_port &operator= (const serial_port &) = delete;
  ~serial_port ();

  // Reads up to buffer.size() bytes. Returns 0 on timeout, throws
  // boost::system::system_error on I/O errors.
  std::size_t read_some (std::span<std::uint8_t> buffer,
                         std::chrono::milliseconds timeout);
};

#endif // !SERIAL_PORT_HPP
