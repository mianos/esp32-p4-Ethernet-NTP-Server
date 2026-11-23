/* UBX framing, used for the receiver's stream and for both directions of the
 * u-center bridge. Frames failing the checksum are dropped. Messages listed in
 * kDefs are also dropped when their length is outside the listed range, so
 * their handlers need no length checks. A message with no rule still cannot be
 * read past its own payload: the readers below return zero there. Lengths are
 * from the u-blox 8 / M8 protocol description. */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "app/text.h"

namespace ubx {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
/* Largest payload accepted: NAV-SAT with 42 satellites. */
constexpr std::size_t kMaxPayload = 512;

struct Frame {
    uint8_t                  cls = 0;
    uint8_t                  id  = 0;
    uint16_t                 len = 0;
    std::span<const uint8_t> payload;

    /* Whether `n` bytes at `at` are inside the payload. */
    bool has(std::size_t at, std::size_t n) const
    {
        return at <= payload.size() && n <= payload.size() - at;
    }

    /* Little-endian field reads. The kDefs gate holds these in range for every
     * message that has a handler; this check keeps a message with no length
     * rule from reading whatever follows its payload as a field, which it
     * otherwise would. It reads zero instead. */
    uint8_t u8(std::size_t at) const
    {
        return has(at, sizeof(uint8_t)) ? text::getLe<uint8_t>(payload.data() + at) : 0;
    }
    uint16_t u2(std::size_t at) const
    {
        return has(at, sizeof(uint16_t)) ? text::getLe<uint16_t>(payload.data() + at) : 0;
    }
    uint32_t u4(std::size_t at) const
    {
        return has(at, sizeof(uint32_t)) ? text::getLe<uint32_t>(payload.data() + at) : 0;
    }
    int32_t i4(std::size_t at) const
    {
        return has(at, sizeof(int32_t)) ? text::getLe<int32_t>(payload.data() + at) : 0;
    }
    /* For MON-VER's NUL-terminated strings. */
    std::span<const uint8_t> from(std::size_t at) const
    {
        return has(at, 0) ? payload.subspan(at) : std::span<const uint8_t>{};
    }
};

struct Def {
    uint8_t     cls;
    uint8_t     id;
    uint16_t    minLen;
    uint16_t    maxLen;
    const char *name;
};

/* Length-checked messages. Others are delivered without a length check. */
inline constexpr Def kDefs[] = {
    {0x01, 0x07, 92, 92, "NAV-PVT"},
    {0x01, 0x26, 24, 24, "NAV-TIMELS"},
    {0x01, 0x27, 16, 16, "NAV-TIM-TP"},
    {0x01, 0x35, 8, 512, "NAV-SAT"}, /* 8 + 12n, n <= 42 */
    {0x0D, 0x01, 16, 16, "TIM-TP"},
    {0x0D, 0x04, 28, 28, "TIM-SVIN"},
    {0x0D, 0x12, 56, 56, "TIM-TOS"},
    {0x05, 0x00, 2, 2, "ACK-ACK"},
    {0x05, 0x01, 2, 2, "ACK-NAK"},
    {0x06, 0x00, 20, 20, "CFG-PRT"},
    {0x06, 0x01, 3, 8, "CFG-MSG"},
    {0x06, 0x31, 32, 32, "CFG-TP5"},
    {0x06, 0x3D, 28, 28, "CFG-TMODE2"},
    {0x0A, 0x04, 30, 512, "MON-VER"}, /* 30 + NUL-terminated extensions */
    {0x0A, 0x09, 60, 60, "MON-HW"},
    {0x0A, 0x36, 8, 512, "MON-COMMS"}, /* 8 + 40n */
};

/* nullptr when the message has no length rule. */
inline const Def *definition(uint8_t cls, uint8_t id)
{
    for (const Def &d : kDefs) {
        if (d.cls == cls && d.id == id) {
            return &d;
        }
    }
    return nullptr;
}

/* Why a frame did not reach a listener; each is counted. */
enum class Drop : uint8_t { Checksum, Length };

class Listener {
public:
    virtual ~Listener()                               = default;
    virtual void onFrame(const Frame &frame)          = 0;
    virtual void onDrop(Drop why, const Frame &frame) = 0;
};

class Decoder {
public:
    void listen(Listener *listener) { m_listener = listener; }

    /* One byte in, at most one frame out; frames may be split anywhere. */
    void feed(uint8_t byte);

    void feed(std::span<const uint8_t> bytes);

    /* Bytes of a half-assembled frame, 0 when idle. */
    std::size_t pending() const { return m_state == State::Idle ? 0 : m_frame.len + 6; }

    uint32_t frames() const { return m_frames; }
    uint32_t droppedChecksum() const { return m_badChecksum; }
    uint32_t droppedLength() const { return m_badLength; }

    /* Discard any partial frame, e.g. after a baud change. */
    void reset();

private:
    enum class State : uint8_t { Idle, Sync2, Class, Id, LenLo, LenHi, Payload, CkA, CkB };

    void emit();
    void warn(Drop why);

    Listener   *m_listener = nullptr;
    State       m_state    = State::Idle;
    Frame       m_frame{};
    uint8_t     m_payload[kMaxPayload] = {};
    std::size_t m_got                  = 0;
    uint8_t     m_ckA                  = 0;
    uint8_t     m_ckB                  = 0;
    uint32_t    m_frames               = 0;
    uint32_t    m_badChecksum          = 0;
    uint32_t    m_badLength            = 0;
};

}  // namespace ubx
