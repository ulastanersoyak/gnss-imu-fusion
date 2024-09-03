#ifndef XBUSPACKET_HPP
#define XBUSPACKET_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

class XbusPacket
{
public:
  XbusPacket ();

  // Add a byte to the internal buffer and update state accordingly. Feeding
  // a byte after a complete packet starts a new one.
  void feedByte (uint8_t byte);

  // Check if a complete and valid packet has been received.
  bool isPacketComplete () const;

  // Check if the packet's checksum is valid.
  bool validateChecksum () const;

  // Reset the state of the packet parser.
  void reset ();

  // Drop the current preamble and re-scan the buffered bytes for the next
  // one. Use after a checksum failure so a real packet hidden inside the bad
  // one is not lost.
  void resync ();

  // Get the data bytes of the packet if it's complete and valid.
  std::vector<uint8_t> getData () const;
  const std::vector<uint8_t> &
  getRawData () const
  {
    return m_buffer;
  }

private:
  static constexpr uint8_t PREAMBLE = 0xFA;
  static constexpr uint8_t BUS_ID = 0xFF;
  static constexpr std::size_t MAX_DATA_LENGTH = 2048;

  uint8_t computeChecksum (const std::vector<uint8_t> &packet) const;

  std::vector<uint8_t> m_buffer{};
  bool m_extendedLength{ false };
  std::size_t m_expectedLength{ 0 };
  bool m_lengthValid{ false };
};

#endif
