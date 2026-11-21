/* The framing itself; ubx.h explains why it is one decoder.
 *
 * The two sync states are the only place rubbish can be discarded cheaply, and
 * the test there is deliberately narrow: NMEA text is off at this receiver, but a
 * boot burst or a u-center session can still put anything on the line, and the
 * state has to be able to sit through it without inventing frames. A repeated
 * 0xB5 in the second sync state stays in it, because `B5 B5 62` is a real frame
 * with a byte of noise in front.
 */
#include "ubx.h"

namespace ubx {

void Decoder::reset()
{
    m_state    = State::Idle;
    m_got      = 0;
    m_frame    = Frame{};
}

void Decoder::feed(std::span<const uint8_t> bytes)
{
    for (const uint8_t byte : bytes) {
        feed(byte);
    }
}

void Decoder::feed(uint8_t byte)
{
    switch (m_state) {
    case State::Idle:
        if (byte == kSync1) {
            m_state = State::Sync2;
        }
        return;

    case State::Sync2:
        if (byte == kSync2) {
            m_state = State::Class;
        } else if (byte != kSync1) {
            m_state = State::Idle;
        }
        return;

    case State::Class:
        m_frame.cls = byte;
        m_ckA       = byte;
        m_ckB       = byte;
        m_state     = State::Id;
        return;

    case State::Id:
        m_frame.id = byte;
        m_ckA      = static_cast<uint8_t>(m_ckA + byte);
        m_ckB      = static_cast<uint8_t>(m_ckB + m_ckA);
        m_state    = State::LenLo;
        return;

    case State::LenLo:
        m_frame.len = byte;
        m_ckA       = static_cast<uint8_t>(m_ckA + byte);
        m_ckB       = static_cast<uint8_t>(m_ckB + m_ckA);
        m_state     = State::LenHi;
        return;

    case State::LenHi: {
        const uint8_t lenLo = static_cast<uint8_t>(m_frame.len);
        m_frame.len |= static_cast<uint16_t>(byte) << 8;
        m_ckA = static_cast<uint8_t>(m_ckA + byte);
        m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
        /* A length that cannot be right is refused here, not walked. Walking it
         * trusts sixteen bits nothing has checked yet: a frame cut short on the
         * line runs straight into the next one's sync pair, B5 62 reads as a
         * length of 25269, and the decoder then sat through 25 KB of healthy
         * stream waiting for a checksum -- five minutes of "silent receiver",
         * a baud scan and a reset of a module that had never stopped talking.
         * The two bytes go back through the hunt, because they may be exactly
         * that sync pair. */
        const Def *def = definition(m_frame.cls, m_frame.id);
        if (m_frame.len > kMaxPayload ||
            (def != nullptr && (m_frame.len < def->minLen || m_frame.len > def->maxLen))) {
            ++m_badLength;
            m_frame.payload = {};
            warn(Drop::Length);
            reset();
            feed(lenLo);
            feed(byte);
            return;
        }
        m_got   = 0;
        m_state = m_frame.len == 0 ? State::CkA : State::Payload;
        return;
    }

    case State::Payload:
        m_payload[m_got++] = byte;
        m_ckA = static_cast<uint8_t>(m_ckA + byte);
        m_ckB = static_cast<uint8_t>(m_ckB + m_ckA);
        if (m_got >= m_frame.len) {
            m_state = State::CkA;
        }
        return;

    case State::CkA:
        if (byte != m_ckA) {
            /* Counted as a checksum drop rather than a silent restart: the
             * first byte already disagreed, and leaving it uncounted made "the
             * line is corrupt" look like "the receiver is quiet". */
            ++m_badChecksum;
            m_frame.payload = std::span<const uint8_t>(m_payload, m_frame.len);
            warn(Drop::Checksum);
            reset();
            return;
        }
        m_state = State::CkB;
        return;

    case State::CkB:
        /* The only gate that matters. A frame that fails it is dropped and the
         * state goes back to hunting sync, because after a bad byte the length
         * that was read may not have been a length at all. */
        if (byte != m_ckB) {
            ++m_badChecksum;
            if (m_listener != nullptr) {
                m_frame.payload = std::span<const uint8_t>(m_payload, m_frame.len);
                m_listener->onDrop(Drop::Checksum, m_frame);
            }
            reset();
            return;
        }
        emit();
        reset();
        return;
    }
}

void Decoder::emit()
{
    /* The length was held to the buffer and to the definition when it was read,
     * so what reaches here is a frame. */
    m_frame.payload = std::span<const uint8_t>(m_payload, m_frame.len);
    ++m_frames;
    if (m_listener != nullptr) {
        m_listener->onFrame(m_frame);
    }
}

void Decoder::warn(Drop why)
{
    if (m_listener != nullptr) {
        m_listener->onDrop(why, m_frame);
    }
}

}  // namespace ubx
