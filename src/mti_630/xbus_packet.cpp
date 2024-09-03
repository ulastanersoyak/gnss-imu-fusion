#include "mti_630/xbus_packet.hpp"

XbusPacket::XbusPacket () : m_extendedLength (false), m_lengthValid (false) {}

void
XbusPacket::feedByte (uint8_t byte)
{
  if (isPacketComplete ())
    {
      reset ();
    }

  if (m_buffer.empty () && byte != PREAMBLE)
    {
      // Wait for preamble
      return;
    }

  if (m_buffer.size () == 1 && byte != BUS_ID)
    {
      // False preamble, the byte may itself start a packet.
      reset ();
      if (byte == PREAMBLE)
        {
          m_buffer.push_back (byte);
        }
      return;
    }

  m_buffer.push_back (byte);

  if (m_buffer.size () == 4)
    {
      if (byte == 0xFF)
        {
          m_extendedLength = true;
        }
      else
        {
          m_expectedLength = byte;
          m_lengthValid = true;
        }
    }
  else if (m_extendedLength && m_buffer.size () == 6)
    {
      m_expectedLength
          = (static_cast<std::size_t> (m_buffer[4]) << 8) | m_buffer[5];
      m_lengthValid = true;
      if (m_expectedLength > MAX_DATA_LENGTH)
        {
          resync ();
        }
    }
}

bool
XbusPacket::isPacketComplete () const
{
  if (!m_lengthValid)
    return false;

  std::size_t totalLength
      = m_extendedLength ? 7 + m_expectedLength : 5 + m_expectedLength;
  return m_buffer.size () == totalLength;
}

void
XbusPacket::reset ()
{
  m_buffer.clear ();
  m_extendedLength = false;
  m_expectedLength = 0;
  m_lengthValid = false; // Reset the validity of the length
}

void
XbusPacket::resync ()
{
  const std::vector<uint8_t> pending (
      m_buffer.begin () + (m_buffer.empty () ? 0 : 1), m_buffer.end ());
  reset ();
  for (const auto byte : pending)
    {
      feedByte (byte);
      if (isPacketComplete ())
        {
          return;
        }
    }
}

bool
XbusPacket::validateChecksum () const
{
  if (!isPacketComplete ())
    return false;

  uint8_t checksum = computeChecksum (m_buffer);
  return checksum == 0;
}

std::vector<uint8_t>
XbusPacket::getData () const
{
  if (!isPacketComplete () || !validateChecksum ())
    return {};

  const auto startIdx = static_cast<std::ptrdiff_t> (m_extendedLength ? 6 : 4);
  const auto endIdx
      = startIdx + static_cast<std::ptrdiff_t> (m_expectedLength);

  return std::vector<uint8_t> (m_buffer.begin () + startIdx,
                               m_buffer.begin () + endIdx);
}

uint8_t
XbusPacket::computeChecksum (const std::vector<uint8_t> &packet) const
{
  uint8_t sum = 0;
  for (std::size_t i = 1; i < packet.size (); ++i)
    { // Exclude the preamble
      sum = static_cast<uint8_t> (sum + packet[i]);
    }

  return sum;
}
