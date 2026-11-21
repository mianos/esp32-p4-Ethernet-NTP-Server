/* UBX framing: one decoder, two readers.
 *
 * There are two byte streams in this tree and both are UBX: the receiver's own
 * output, and whatever a u-center session on tcp/3333 sends or is answered. The
 * receiver's side has always been framed properly -- byte state machine,
 * Fletcher checksum accumulated as the frame arrives, resync on a repeated
 * sync byte, a length latch for payloads too big for the buffer. The bridge side
 * was three sliding-window `B5 62 xx yy` scans with no checksum and no length
 * check, and they got it wrong: a CFG-PRT split across a `recv()`, or a
 * `B5 62 06 00` occurring inside a payload, produced either a missed frame or a
 * port and a baud read out of whatever bytes happened to follow.
 *
 * So the framing lives here, once, and both readers feed the same object. The
 * bridge inherits the checksum gate and the straddling for nothing.
 *
 * The checksum is the whole reason a frame is worth believing: without it a byte
 * of 0xB5 lost at the wrong moment slides every field by one and the firmware
 * anchors the clock to a second that never happened. A frame that fails it is
 * dropped, never repaired.
 *
 * Definitions are a table, not memory. Each entry names the message, the lengths
 * it may claim, and the handler; a frame outside its own range is dropped before
 * a handler sees it, so "is this long enough for the field I am about to read"
 * is answered once, here, instead of by a guard at the top of every handler that
 * someone has to remember to keep in step with the offsets below it.
 *
 * Field offsets and lengths are transcribed from the u-blox 8 / M8 receiver
 * description (the TIM 1.10 dialect for NAV-TIMELS and TIM-SVIN). Where a
 * message is variable the range is 8 + the per-repeat size, so a receiver that
 * reports more satellites than the buffer holds is caught here rather than
 * walking off it.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "text.h"

namespace ubx {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
/* Largest payload accepted. NAV-SAT with 42 satellites is 512; the buffer is
 * sized from this so the two cannot drift. */
constexpr std::size_t kMaxPayload = 512;

struct Frame {
    uint8_t  cls = 0;
    uint8_t  id  = 0;
    uint16_t len = 0;
    std::span<const uint8_t> payload;

    /* Field reads. The caller has already been guaranteed Frame::len is inside
     * the definition's range, so `at` is known to be in the payload by
     * construction -- these do not re-check, they name the type and the
     * little-endianness that the protocol floor assumes. */
    uint8_t  u8(std::size_t at) const { return text::getLe<uint8_t>(payload.data() + at); }
    uint16_t u2(std::size_t at) const { return text::getLe<uint16_t>(payload.data() + at); }
    uint32_t u4(std::size_t at) const { return text::getLe<uint32_t>(payload.data() + at); }
    int32_t  i4(std::size_t at) const { return text::getLe<int32_t>(payload.data() + at); }
    /* A NUL-terminated string field, which MON-VER is made of. */
    std::span<const uint8_t> from(std::size_t at) const { return payload.subspan(at); }
};

struct Def {
    uint8_t     cls;
    uint8_t     id;
    uint16_t    minLen;
    uint16_t    maxLen;
    const char *name;
};

/* Every message the firmware decodes, plus the two the bridge inspects. Messages
 * outside the table are still delivered -- MON-* and CFG-* arrive from u-center
 * that this firmware has no interest in -- with no length gate. */
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

/* The definition for a class and id, or nullptr if this firmware has no opinion
 * about the message's length. */
inline const Def *definition(uint8_t cls, uint8_t id)
{
    for (const Def &d : kDefs) {
        if (d.cls == cls && d.id == id) {
            return &d;
        }
    }
    return nullptr;
}

/* Why a frame did not reach a listener. Counted, because "the receiver went
 * quiet" and "every frame is failing its checksum" look identical from the
 * outside and mean different things. */
enum class Drop : uint8_t { Checksum, Length };

class Listener {
public:
    virtual ~Listener() = default;
    virtual void onFrame(const Frame &frame) = 0;
    virtual void onDrop(Drop why, const Frame &frame) = 0;
};

class Decoder {
public:
    void listen(Listener *listener) { m_listener = listener; }

    /* One byte in, at most one frame out. Everything the decoder needs between
     * calls is a member, so a frame split anywhere -- between the sync bytes,
     * across the length, halfway through the payload -- is assembled the same as
     * one that arrives whole. */
    void feed(uint8_t byte);

    /* A run of bytes. */
    void feed(std::span<const uint8_t> bytes);

    /* Bytes currently half-assembled, so a caller can tell "mid-frame" from
     * "idle". */
    std::size_t pending() const { return m_state == State::Idle ? 0 : m_frame.len + 6; }

    uint32_t frames() const { return m_frames; }
    uint32_t droppedChecksum() const { return m_badChecksum; }
    uint32_t droppedLength() const { return m_badLength; }

    /* Back to hunting for sync. For a caller that knows the stream broke under
     * the decoder -- a rate change -- so half a frame at the old rate is not
     * finished with bytes at the new one. */
    void reset();

private:
    enum class State : uint8_t { Idle, Sync2, Class, Id, LenLo, LenHi, Payload, CkA, CkB };

    void emit();
    void warn(Drop why);

    Listener   *m_listener  = nullptr;
    State       m_state     = State::Idle;
    Frame       m_frame{};
    uint8_t     m_payload[kMaxPayload] = {};
    std::size_t m_got       = 0;
    uint8_t     m_ckA       = 0;
    uint8_t     m_ckB       = 0;
    uint32_t    m_frames    = 0;
    uint32_t    m_badChecksum = 0;
    uint32_t    m_badLength   = 0;
};

}  // namespace ubx
