#include "utility/serial_port.hpp"

#include <boost/asio/buffer.hpp>
#include <boost/system/system_error.hpp>
#include <string>

serial_port::serial_port (
    std::string_view port_name, std::uint32_t baud_rate,
    boost::asio::serial_port_base::parity::type parity,
    boost::asio::serial_port_base::stop_bits::type stop_bits,
    boost::asio::serial_port_base::character_size char_size)
    : port_{ context_ }
{
  port_.open (std::string (port_name));
  port_.set_option (boost::asio::serial_port_base::baud_rate (baud_rate));
  port_.set_option (boost::asio::serial_port_base::parity (parity));
  port_.set_option (boost::asio::serial_port_base::stop_bits (stop_bits));
  port_.set_option (char_size);
}

serial_port::~serial_port ()
{
  if (port_.is_open ())
    {
      boost::system::error_code ignored;
      port_.close (ignored);
    }
}

std::size_t
serial_port::read_some (std::span<std::uint8_t> buffer,
                        std::chrono::milliseconds timeout)
{
  boost::system::error_code result{};
  std::size_t transferred{ 0 };
  port_.async_read_some (
      boost::asio::buffer (buffer.data (), buffer.size ()),
      [&] (const boost::system::error_code &ec, std::size_t n)
        {
          result = ec;
          transferred = n;
        });

  context_.restart ();
  context_.run_for (timeout);
  if (!context_.stopped ())
    {
      // Timed out: cancel and wait for the handler to observe it.
      port_.cancel ();
      context_.restart ();
      context_.run ();
    }

  if (result == boost::asio::error::operation_aborted)
    {
      return transferred;
    }
  if (result)
    {
      throw boost::system::system_error (result);
    }
  return transferred;
}
