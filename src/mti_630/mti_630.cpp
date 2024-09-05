#include "mti_630/mti_630.hpp"
#include "mti_630/data_packet_parser.hpp"
#include "mti_630/xbus_packet.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <thread>

mti_630::mti_630 (std::string_view port_name, std::uint32_t baud_rate,
                  boost::asio::serial_port_base::parity::type parity,
                  boost::asio::serial_port_base::stop_bits::type stop_bits,
                  boost::asio::serial_port_base::character_size char_size)
    : port_{ port_name, baud_rate, parity, stop_bits, char_size }
{
}

void
mti_630::work (const std::stop_token &stop, queue<sensor_event> &events)
{
  XbusPacket packet{};
  std::array<std::uint8_t, 512> buffer{};
  std::size_t bad_checksums{ 0 };

  while (!stop.stop_requested ())
    {
      std::size_t n{ 0 };
      try
        {
          n = port_.read_some (buffer, std::chrono::milliseconds (100));
        }
      catch (const std::exception &e)
        {
          std::cerr << "mti_630: read error (" << e.what () << ")\n";
          std::this_thread::sleep_for (std::chrono::seconds (1));
          continue;
        }

      for (std::size_t i = 0; i < n; ++i)
        {
          packet.feedByte (buffer[i]);
          while (packet.isPacketComplete ())
            {
              if (packet.validateChecksum ())
                {
                  Xbus xbusData{};
                  if (DataPacketParser::parseDataPacket (packet.getRawData (),
                                                         xbusData))
                    {
                      events.push (xbusData);
                    }
                  packet.reset ();
                }
              else
                {
                  if (bad_checksums++ % 100 == 0)
                    {
                      std::cerr << "mti_630: invalid checksum ("
                                << bad_checksums << " so far)\n";
                    }
                  packet.resync ();
                }
            }
        }
    }
}
