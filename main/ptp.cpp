#include "ptp.h"

#include "soc/emac_dma_struct.h"

#include "eth_link.h"
#include "gps_time.h"

#include "esp_attr.h"
#include "esp_eth_mac_esp.h"
#include "hal/emac_hal.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <sys/time.h>

#include <cstdlib>
#include <cstring>

#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr const char *TAG = "ptp";

/* Enough for the NTP excursions of a burst plus a PTP exchange's worth; the
 * responder takes its entry immediately after recvfrom(), so entries are only
 * briefly live. */
constexpr size_t kRing = 16;
constexpr int64_t kStampAgeNs = 500000000LL; /* an entry older than this is noise */

struct RxStamp {
    uint8_t  addr[16];
    uint8_t  addrLen;
    uint8_t  mac[6];
    uint16_t port;
    bool     used;
    int64_t  ns;
};

esp_eth_handle_t s_eth   = nullptr;
esp_eth_mac_t   *s_mac   = nullptr;
esp_netif_t     *s_netif = nullptr;
bool             s_attached = false;
bool             s_enabled  = false;

} // namespace

namespace ptp {
volatile bool gHardwareOn = false;
}

namespace {

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
RxStamp      s_ring[kRing];
size_t       s_next = 0;
ptp::Stats    s_stats{};
uint8_t       s_lastFail = 0;

void fail(uint8_t reason)
{
    s_lastFail = reason;
    if (s_stats.txMissing == 0) {
        ESP_LOGW(TAG, "sendUdp refused: reason %u", static_cast<unsigned>(reason));
    }
}

uint16_t be16(const uint8_t *p)
{
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

void recordRx(const uint8_t *mac, const uint8_t *addr, uint8_t addrLen, uint16_t port, int64_t ns)
{
    portENTER_CRITICAL(&s_mux);
    RxStamp &slot = s_ring[s_next];
    s_next        = (s_next + 1) % kRing;
    std::memcpy(slot.addr, addr, addrLen);
    std::memcpy(slot.mac, mac, 6);
    slot.addrLen = addrLen;
    slot.port    = port;
    slot.used    = true;
    slot.ns      = ns;
    portEXIT_CRITICAL(&s_mux);

    ++s_stats.rxStamps;
    s_stats.lastNs = ns;
    if (s_stats.rxStamps <= 20) {
        ESP_LOGW(TAG, "record port %u from %u.%u.%u.%u", static_cast<unsigned>(port), addr[0],
                 addr[1], addr[2], addr[3]);
    }
}

/* A UDP datagram bound for port 123, in either family. Returns the source once
 * the frame has been checked far enough to have one. */
void maybeRecordUdp(const uint8_t *frame, uint32_t len, int64_t ns)
{
    if (len < 34) {
        return;
    }
    const uint16_t type = be16(frame + 12);
    if (type == 0x0800) {
        if (len < 14 + 20 + 8) {
            return;
        }
        const uint8_t ihl = static_cast<uint8_t>((frame[14] & 0x0f) * 4);
        if (ihl < 20 || len < static_cast<uint32_t>(14 + ihl + 8)) {
            return;
        }
        const uint8_t *udp = frame + 14 + ihl;
        if (frame[23] != 17 || be16(udp + 2) != 123) {
            return;
        }
        recordRx(frame + 6, frame + 26, 4, be16(udp), ns);
    } else if (type == 0x86dd) {
        if (len < 14 + 40 + 8 || frame[20] != 17) {
            return;
        }
        const uint8_t *udp = frame + 54;
        if (be16(udp + 2) != 123) {
            return;
        }
        recordRx(frame + 6, frame + 22, 16, be16(udp), ns);
    }
}


/* ------------------------------------------------------------------ PTP ---
 *
 * The message layer. Two-step only: a Sync goes out with the twoStepFlag set and
 * a Follow_Up carries the stamp the MAC gave that Sync, so nothing here has to
 * predict a transmit time. That is the one thing this unit cannot do (one-step
 * needs the timestamp inserted into the outgoing frame by hardware).
 *
 * The stamps are TAI, which is what PTP counts: the counter runs on the UTC the
 * rest of the box uses, so every stamp has the UTC offset added and the same
 * offset is announced. A host that keeps its clock in UTC therefore sees 37 s
 * against its own clock and the right answer against another PTP node.
 */

constexpr uint8_t kMsgSync     = 0x00;
constexpr uint8_t kMsgDelayReq = 0x01;
constexpr uint8_t kMsgFollowUp = 0x08;
constexpr uint8_t kMsgDelayResp = 0x09;
constexpr uint8_t kMsgAnnounce = 0x0B;
constexpr uint8_t kPtpVersion  = 2;

constexpr size_t kHeader = 34;
constexpr size_t kStamp  = 10; /* 48-bit seconds, 32-bit nanoseconds */
constexpr size_t kSyncLen = kHeader + kStamp;
constexpr size_t kFollowUpLen = kHeader + kStamp;
constexpr size_t kDelayRespLen = kHeader + kStamp + 10;
constexpr size_t kAnnounceLen  = kHeader + 30;

constexpr uint8_t kMulticast[6] = {0x01, 0x1b, 0x19, 0x00, 0x00, 0x00};
/* flagField is a 16-bit big-endian bit field: bit 1 is twoStep, bit 7 is
 * currentUtcOffsetValid, and ptpTimescale/timeTraceable are bits 8 and 9, i.e.
 * the second octet. Putting twoStep in the second octet instead is what makes
 * a two-step master look like a one-step one and a slave wait for a Follow_Up
 * that never matches. */
/* flagField octet 0: bit 0 alternateMaster, bit 1 twoStep, bit 2 unicast.
 * Octet 1: bit 0 leap61, bit 1 leap59, bit 2 currentUtcOffsetValid, bit 3
 * ptpTimescale, bit 4 timeTraceable, bit 5 frequencyTraceable. Getting the
 * octet wrong is silent: the announce still parses, it just says the time is
 * ARB and a slave then leaves the 37 s of TAI in its offset. */
constexpr uint8_t kFlagTwoStep   = 0x02; /* octet 0 */
constexpr uint8_t kFlagUtcValid  = 0x04; /* octet 1 */
constexpr uint8_t kFlagTimescale = 0x08; /* octet 1 */
constexpr uint8_t kFlagTraceable = 0x30; /* octet 1, time and frequency */

constexpr int kTaskStack = 4096;

/* Whether a transmit stamp is asked of the hardware, and only for the frames
 * whose stamp is used: an NTP reply's arrival comes from the RX side and its
 * transmit field has to be written before the frame leaves anyway, so it gains
 * nothing from the read-back. The read-back is the path that can leave a
 * descriptor holding a pending timestamp, so the exposure is kept to the one
 * frame a second that needs it. The Follow_Up falls back to the counter read
 * just before the send, which is what any non-stamped master does. */
constexpr bool kTxStamp = true;

volatile uint8_t gMode      = 0;
uint8_t          gClock[8]  = {};
uint16_t         gPort      = 1;
uint8_t          gDomain    = 0;
uint8_t          gPriority1 = 128;
uint16_t         gSeq[16]   = {};
TaskHandle_t     gTask      = nullptr;
SemaphoreHandle_t s_txMux   = nullptr;

struct DelayReq {
    uint8_t  mac[6];
    uint8_t  clock[8];
    uint16_t port;
    uint16_t seq;
    int64_t  rxNs;
};
constexpr size_t kDelayRing = 8;

/* Slave side: the exchange in flight and what the last one measured. Single
 * peer, single exchange at a time -- a LAN has one master and the servo is
 * reported, not applied, so a pipelined design buys nothing. */
struct Exchange {
    bool     syncSeen;
    int64_t  t2;      /* arrival of the Sync */
    uint16_t syncSeq;
    uint8_t  clock[8]; /* the master's port identity */
    uint16_t port;
    int64_t  t3;       /* egress of our Delay_Req */
    uint16_t delaySeq;
    int64_t  t1; /* the master's origin, from the Follow_Up */
    int64_t  offsetNs;
    int64_t  delayNs;
    uint32_t measured;
    uint32_t timeouts;
};
Exchange gSlave = {};
uint32_t gLastReportS = 0;
DelayReq  gDelay[kDelayRing];
size_t    gDelayNext = 0;
portMUX_TYPE gDelayMux = portMUX_INITIALIZER_UNLOCKED;

void put16(uint8_t *p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xff);
}

uint16_t get16(const uint8_t *p)
{
    return be16(p);
}

/* 48-bit seconds then 32-bit nanoseconds, both big endian. */
void putStamp(uint8_t *p, int64_t ns)
{
    uint64_t seconds = static_cast<uint64_t>(ns / 1000000000LL);
    for (int i = 0; i < 6; ++i) {
        p[5 - i] = static_cast<uint8_t>(seconds >> (8 * i));
    }
    /* The fraction starts after the six second bytes: p[3-i] would overwrite
     * four of them, which reads back as a timestamp in the wrong century. */
    uint32_t frac = static_cast<uint32_t>(ns % 1000000000LL);
    for (int i = 0; i < 4; ++i) {
        p[9 - i] = static_cast<uint8_t>(frac >> (8 * i));
    }
}

/* TAI, from the counter and the receiver's leap offset. NAV-TIMELS carries
 * GPS-UTC; TAI is GPS plus 19 s. */
uint8_t utcOffset()
{
    /* GPS-UTC is 18 s; TAI is GPS plus 19. The receiver's own figure is used
     * when it has one, and the known constant when NAV-TIMELS is silent --
     * announcing 19 would put a slave 18 s out. */
    const int leap   = static_cast<int>(GpsTime::instance().status().leapSeconds);
    const int offset = (leap > 0 ? leap : 18) + 19;
    return static_cast<uint8_t>(offset > 127 ? 127 : offset);
}

int64_t taiNs()
{
    return ptp::nowNs() + static_cast<int64_t>(utcOffset()) * 1000000000LL;
}

size_t putHeader(uint8_t *out, uint8_t type, size_t length, uint16_t seq, uint8_t flags0,
                 uint8_t flags1)
{
    /* Transport specific in the high nibble of the first byte and the message
     * type in the low one; versionPTP is the low nibble of the *second* byte.
     * Putting the version in the first byte is the mistake that reads as PTPv0
     * and makes ptp4l discard the frame without saying so. */
    out[0] = static_cast<uint8_t>(type & 0x0f);
    out[1] = kPtpVersion;
    put16(out + 2, static_cast<uint16_t>(length));
    out[4] = gDomain;
    out[6] = flags0;
    out[7] = flags1;
    /* correctionField (8) stays zero: two-step puts the precise origin in the
     * Follow_Up, and a Delay_Resp's correction is the responder's processing
     * time, which this does not measure. */
    std::memcpy(out + 20, gClock, 8);
    put16(out + 28, gPort);
    put16(out + 30, seq);
    /* control is deprecated but set for older readers; logMessageInterval is
     * 2^n seconds, so 0 is once a second and 127 (which reads as 2^127) is not
     * a legal interval at all. The Sync and Follow_Up go out as n = 0 too,
     * since a 1 Hz timepulse has nothing to offer a faster rate. */
    out[32] = 0x00;
    out[33] = 0x00;
    return kHeader;
}

/* One PTP frame out, ethertype 0x88F7, to `dest` (multicast or the peer). Returns
 * the egress stamp in the counter's base. */
esp_err_t sendPtp(const uint8_t dest[6], const uint8_t *body, size_t length, int64_t *stampNs)
{
    static uint8_t frame[kHeader + 30 + 14];
    std::memset(frame, 0, sizeof(frame));
    uint8_t mac[6] = {};
    if (esp_eth_ioctl(s_eth, ETH_CMD_G_MAC_ADDR, mac) != ESP_OK) {
        return ESP_FAIL;
    }
    std::memcpy(frame, dest, 6);
    std::memcpy(frame + 6, mac, 6);
    frame[12] = 0x88;
    frame[13] = 0xf7;
    std::memcpy(frame + 14, body, length);
    /* Ethernet needs 60 bytes before the FCS and a Sync is 58. The padding is
     * below the PTP layer, so messageLength stays as built. */
    const size_t frameLen = 14 + length < 60 ? 60 : 14 + length;
    return ptp::transmit(frame, frameLen, stampNs);
}

void announcedQuality(uint8_t &clockClass, uint8_t &accuracy, uint8_t &timeSource)
{
    const GpsTime::Status  st    = GpsTime::instance().status();
    const GpsTime::ReplyFacts facts = GpsTime::instance().replyFacts();
    if (st.locked && !facts.demoted) {
        clockClass = 6;
        accuracy   = 0x25; /* 10 us: the stamps are interrupt- and software-class */
        timeSource = 0x20;
    } else if (st.locked) {
        clockClass = 7; /* holdover, inside the window */
        accuracy   = 0x27; /* 100 us */
        timeSource = 0x20;
    } else {
        clockClass = 52;
        accuracy   = 0x2a; /* 2.5 ms */
        timeSource = 0x00;
    }
}

void sendAnnounce()
{
    uint8_t body[kAnnounceLen] = {};
    const uint8_t flags0 = 0;
    uint8_t       quality = 0;
    uint8_t       accuracy = 0;
    uint8_t       source = 0;
    announcedQuality(quality, accuracy, source);
    putHeader(body, kMsgAnnounce, kAnnounceLen, gSeq[kMsgAnnounce]++, flags0,
              static_cast<uint8_t>(kFlagUtcValid | kFlagTimescale | kFlagTraceable));
    putStamp(body + kHeader, taiNs());
    put16(body + kHeader + 10, utcOffset());
    body[kHeader + 13] = gPriority1;
    body[kHeader + 14] = quality;
    body[kHeader + 15] = accuracy;
    put16(body + kHeader + 16, 0x8000); /* offsetScaledLogVariance: unknown */
    body[kHeader + 18] = 255;            /* priority2 */
    std::memcpy(body + kHeader + 19, gClock, 8);
    put16(body + kHeader + 27, 0); /* stepsRemoved */
    body[kHeader + 29] = source;
    (void)sendPtp(kMulticast, body, kAnnounceLen, nullptr);
    ++s_stats.announceSent;
}

/* Sync, then the Follow_Up that carries its stamp. The pair is the measurement;
 * a slave may not use the Sync's own timestamp. */
void sendSync()
{
    const uint16_t seq = gSeq[kMsgSync]++;
    uint8_t        sync[kSyncLen] = {};
    putHeader(sync, kMsgSync, kSyncLen, seq, kFlagTwoStep, 0);
    /* The counter is read before the frame: without a hardware stamp that is
     * the best the Follow_Up can carry, and it is within the send cost of the
     * truth. With one, the measured value replaces it. */
    int64_t syncNs = 0;
    if (kTxStamp) {
        if (sendPtp(kMulticast, sync, kSyncLen, &syncNs) != ESP_OK || syncNs == 0) {
            return;
        }
    } else {
        syncNs = ptp::nowNs();
        if (sendPtp(kMulticast, sync, kSyncLen, nullptr) != ESP_OK) {
            return;
        }
    }
    uint8_t follow[kFollowUpLen] = {};
    putHeader(follow, kMsgFollowUp, kFollowUpLen, seq, kFlagTwoStep, 0);
    /* The counter is UTC-based; the wire is TAI. */
    putStamp(follow + kHeader, syncNs + static_cast<int64_t>(utcOffset()) * 1000000000LL);
    (void)sendPtp(kMulticast, follow, kFollowUpLen, nullptr);
    ++s_stats.syncSent;
}

/* One queued request, oldest first; false when the queue is empty. The task
 * drains the whole queue each tick, so a client that bursts (or several) is
 * answered up to the ring's depth instead of one per second. */
bool takeDelayReq(DelayReq &req)
{
    bool have = false;
    portENTER_CRITICAL(&gDelayMux);
    for (size_t i = 0; i < kDelayRing; ++i) {
        if (gDelay[i].seq != 0xffff) {
            req           = gDelay[i];
            have          = true;
            gDelay[i].seq = 0xffff;
            break;
        }
    }
    portEXIT_CRITICAL(&gDelayMux);
    return have;
}

void answerDelayReq(const DelayReq &req)
{
    uint8_t resp[kDelayRespLen] = {};
    putHeader(resp, kMsgDelayResp, kDelayRespLen, req.seq, 0, 0);
    putStamp(resp + kHeader, req.rxNs + static_cast<int64_t>(utcOffset()) * 1000000000LL);
    std::memcpy(resp + kHeader + kStamp, req.clock, 8);
    put16(resp + kHeader + kStamp + 8, req.port);
    (void)sendPtp(req.mac, resp, kDelayRespLen, nullptr);
    ++s_stats.delayResp;
}

void slaveTick();

void ptpTask(void *)
{
    /* The start hangs off the address, which is applied a moment before the
     * driver starts the MAC. Wait for a transmit to be accepted rather than
     * firing the first announces into a downed link. */
    for (int i = 0; i < 50 && gMode != 0; ++i) {
        uint8_t probe[kHeader] = {};
        putHeader(probe, kMsgAnnounce, kHeader, 0, 0, 0);
        if (sendPtp(kMulticast, probe, kHeader, nullptr) == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    TickType_t last = xTaskGetTickCount();
    while (gMode != 0) {
        /* Only a master announces: a slave that kept announcing would be
         * elected by the master it is trying to follow. */
        if (gMode == static_cast<uint8_t>(ptp::Mode::Master)) {
            sendAnnounce();
            sendSync();
            DelayReq queued;
            for (size_t i = 0; i < kDelayRing && takeDelayReq(queued); ++i) {
                answerDelayReq(queued);
            }
        } else {
            slaveTick();
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
    gTask = nullptr;
    vTaskDelete(nullptr);
}

/* ---- slave ---- */

void sendDelayReq(const uint8_t mac[6], const uint8_t clock[8], uint16_t port)
{
    (void)clock;
    (void)port;
    uint8_t body[kHeader + kStamp] = {};
    const uint16_t seq = gSeq[kMsgDelayReq]++;
    putHeader(body, kMsgDelayReq, kHeader + kStamp, seq, 0, 0);
    putStamp(body + kHeader, taiNs());
    int64_t egress = 0;
    if (sendPtp(mac, body, kHeader + kStamp, &egress) == ESP_OK && egress != 0) {
        gSlave.t3       = egress;
        gSlave.delaySeq = seq;
    }
}

void onSync(const uint8_t *frame, int64_t rxNs, uint16_t seq)
{
    gSlave.syncSeen = true;
    gSlave.t2       = rxNs;
    gSlave.syncSeq  = seq;
    std::memcpy(gSlave.clock, frame + 14 + 20, 8);
    gSlave.port     = get16(frame + 14 + 28);
}

void onFollowUp(const uint8_t *frame, uint16_t seq)
{
    if (!gSlave.syncSeen || seq != gSlave.syncSeq || gSlave.t2 == 0) {
        return;
    }
    gSlave.syncSeen = false;
    const uint8_t *stamp = frame + 14 + kHeader;
    uint64_t seconds = 0;
    for (int i = 0; i < 6; ++i) {
        seconds = (seconds << 8) | stamp[i];
    }
    uint32_t fraction = 0;
    for (int i = 0; i < 4; ++i) {
        fraction = (fraction << 8) | stamp[6 + i];
    }
    gSlave.t1 = static_cast<int64_t>(seconds) * 1000000000LL + fraction;
    /* The Delay_Req goes out at once, so the exchange is one round trip and the
     * pair is still fresh when the answer comes back. */
    sendDelayReq(frame + 6, gSlave.clock, gSlave.port);
}

void onDelayResp(const uint8_t *frame, int64_t rxNs, uint16_t seq)
{
    if (seq != gSlave.delaySeq || gSlave.t3 == 0 || gSlave.t1 == 0 || gSlave.t2 == 0) {
        return;
    }
    /* offset = ((t2 - t1) - (t4 - t3)) / 2, all four on the counter's base. */
    const int64_t t4     = rxNs;
    const int64_t offset = ((gSlave.t2 - gSlave.t1) - (t4 - gSlave.t3)) / 2;
    gSlave.offsetNs      = offset;
    gSlave.delayNs       = ((t4 - gSlave.t3) + (gSlave.t2 - gSlave.t1)) / 2;
    gSlave.t3            = 0;
    gSlave.t1            = 0;
    gSlave.t2            = 0;
    ++gSlave.measured;
    ++s_stats.measured;
    s_stats.lastPeerNs    = rxNs;
    s_stats.slaveOffsetNs = offset;
    (void)frame;
}

void slaveTick()
{
    const uint32_t nowS = static_cast<uint32_t>(esp_timer_get_time() / 1000000);
    if (gSlave.measured != 0 && nowS - gLastReportS >= 5) {
        gLastReportS = nowS;
        ESP_LOGI(TAG, "slave: offset %+lld ns, delay %lld ns, %u measured",
                 static_cast<long long>(gSlave.offsetNs), static_cast<long long>(gSlave.delayNs),
                 static_cast<unsigned>(gSlave.measured));
    }
}

/* A Delay_Req from a peer: keep where it came from, its portIdentity and the
 * arrival stamp, and answer from the task so the RX path stays short. */
void noteDelayReq(const uint8_t *frame, uint32_t len, int64_t ns)
{
    if (len < 14 + kHeader + kStamp || gMode != static_cast<uint8_t>(ptp::Mode::Master)) {
        return;
    }
    portENTER_CRITICAL(&gDelayMux);
    DelayReq &slot = gDelay[gDelayNext];
    gDelayNext     = (gDelayNext + 1) % kDelayRing;
    std::memcpy(slot.mac, frame + 6, 6);
    std::memcpy(slot.clock, frame + 14 + 20, 8);
    slot.port = get16(frame + 14 + 28);
    slot.seq  = get16(frame + 14 + 30);
    slot.rxNs = ns;
    portEXIT_CRITICAL(&gDelayMux);
}

void notePeer(const uint8_t *frame)
{
    std::memset(s_stats.lastPeer, 0, sizeof(s_stats.lastPeer));
    std::memcpy(s_stats.lastPeer, frame + 6, 4);
    s_stats.lastPeer[4]  = frame[10];
    s_stats.lastPeer[5]  = frame[11];
    s_stats.lastPeerLen  = 4; /* an IPv4-shaped summary; the MAC is what matters */
}

esp_err_t inputHandler(esp_eth_handle_t, uint8_t *buffer, uint32_t length, void *priv, void *info)
{
    const auto *stamp = static_cast<const eth_mac_time_t *>(info);
    const bool  valid = stamp != nullptr && stamp->seconds != 0;
    const int64_t ns  = valid ? static_cast<int64_t>(stamp->seconds) * 1000000000LL +
                                   static_cast<int64_t>(stamp->nanoseconds)
                              : 0;

    if (length >= 14 && be16(buffer + 12) == 0x88F7) {
        ++s_stats.frames;
        if (valid) {
            s_stats.lastNs = ns;
        }
        if (length >= 14 + 34) {
            /* Delay_Reqs are the only frames answered from here, and only as a
             * master; everything else is for the slave side. Parsed in place and
             * the buffer handed back at once. */
            notePeer(buffer);
            if (gMode == static_cast<uint8_t>(ptp::Mode::Master) && buffer[14] == kMsgDelayReq) {
                noteDelayReq(buffer, length, ns);
            } else if (gMode == static_cast<uint8_t>(ptp::Mode::Slave)) {
                switch (buffer[14]) {
                case kMsgSync:
                    onSync(buffer, ns, get16(buffer + 14 + 30));
                    break;
                case kMsgFollowUp:
                    onFollowUp(buffer, get16(buffer + 14 + 30));
                    break;
                case kMsgDelayResp:
                    onDelayResp(buffer, ns, get16(buffer + 14 + 30));
                    break;
                default:
                    break;
                }
            }
        }
        free(buffer);
        return ESP_OK;
    }
    if (valid) {
        maybeRecordUdp(buffer, length, ns);
    }
    /* priv is the netif: esp_eth_update_input_path_info() overwrites the glue's
     * own priv slot with it, so it cannot be anything else. */
    return esp_netif_receive(static_cast<esp_netif_t *>(priv), buffer, length, NULL);
}

} // namespace

namespace ptp {

esp_err_t attach(esp_eth_handle_t eth, esp_eth_mac_t *mac, esp_netif_t *netif)
{
    if (s_attached) {
        return ESP_OK;
    }
    if (eth == nullptr || mac == nullptr || netif == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    s_eth   = eth;
    s_mac   = mac;
    s_netif = netif;
    /* Clock identity from the MAC: the EUI-64 the 802.1AS form makes of it (the
     * U/L bit set, ff:fe in the middle). Stable for the life of the board, which
     * is all a port identity needs. */
    uint8_t ourMac[6] = {};
    if (esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, ourMac) == ESP_OK) {
        gClock[0] = static_cast<uint8_t>(ourMac[0] | 0x02);
        gClock[1] = ourMac[1];
        gClock[2] = ourMac[2];
        gClock[3] = 0xff;
        gClock[4] = 0xfe;
        gClock[5] = ourMac[3];
        gClock[6] = ourMac[4];
        gClock[7] = ourMac[5];
    }
    const esp_err_t err = esp_eth_update_input_path_info(eth, inputHandler, netif);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "input path: %s", esp_err_to_name(err));
        return err;
    }
    s_attached = true;
    return ESP_OK;
}

esp_err_t enable(bool restart)
{
    if (s_mac == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_enabled && !restart) {
        return ESP_OK;
    }

    /* A second ptp_enable() on an already running block leaves the timestamp
     * unit unable to produce anything -- the frames still go out, without
     * stamps, and the responder stops being able to send a Sync at all. A
     * restart disables it first and puts the counter back on the clock
     * afterwards, since ptp_start() zeroes it. */
    if (restart && s_enabled && s_mac != nullptr) {
        (void)esp_eth_mac_ptp_disable(s_mac);
    }

    eth_mac_ptp_config_t config = {};
    /* The crystal, at 40 MHz: 25 ns is the finest the counter can be stepped and
     * is also what its increment is set to. The 80 MHz PLL is the alternative
     * and buys 12.5 ns for a clock that has to be kept running. */
    config.clk_src            = EMAC_PTP_CLK_SRC_XTAL;
    config.clk_src_period_ns  = 25.0f;
    /* 40 and not the 25 ns the source ticks at: the driver derives the addend
     * from 2^32 * period / accuracy, and at 25/25 that is exactly 2^32, which
     * truncates to zero in a uint32. 40 ns keeps it well inside and still
     * resolves far finer than the microseconds this clock has to serve. */
    config.required_accuracy_ns = 40.0f;
    config.roll_type          = ETH_PTP_DIGITAL_ROLLOVER;

    esp_err_t err = esp_eth_mac_ptp_enable(s_mac, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "phc enable: %s", esp_err_to_name(err));
        return err;
    }
    /* Receive stamps on every frame, not only PTP's: the responder needs the
     * arrival stamp of a UDP/123 datagram long before there is a PTP message to
     * serve. The transmit side is the opposite -- see transmit(). */
    err = esp_eth_mac_enable_ts4all(s_mac, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ts4all: %s", esp_err_to_name(err));
        return err;
    }
    if (s_txMux == nullptr) {
        s_txMux = xSemaphoreCreateMutex();
        if (s_txMux == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }
    uint32_t stampBit = EMAC_HAL_TDES0_TX_TS_ENABLE;
    esp_eth_ioctl(s_eth, static_cast<esp_eth_io_cmd_t>(ETH_MAC_ESP_CMD_CLEAR_TDES0_CFG_BITS), &stampBit);
    if (restart) {
        (void)stepToRealtime();
    }

    /* Start it at CLOCK_REALTIME so a stamp is an epoch instant and can be read
     * by eye. Before the clock is set this is 1970, which the anchor corrects
     * from the first fix. */
    struct timeval tv = {};
    gettimeofday(&tv, nullptr);
    eth_mac_time_t start = {};
    start.seconds        = static_cast<uint32_t>(tv.tv_sec);
    start.nanoseconds    = static_cast<uint32_t>(tv.tv_usec) * 1000;
    err = esp_eth_mac_set_ptp_time(s_mac, &start);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "phc set: %s", esp_err_to_name(err));
    }

    s_enabled     = true;
    ptp::gHardwareOn = true;
    ESP_LOGI(TAG, "phc running, 25 ns steps, started from %u.%09u",
             static_cast<unsigned>(start.seconds), static_cast<unsigned>(start.nanoseconds));
    return ESP_OK;
}

bool enabled()
{
    return s_enabled;
}

esp_err_t stepToRealtime()
{
    if (!s_enabled || s_mac == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    struct timeval tv = {};
    gettimeofday(&tv, nullptr);
    eth_mac_time_t now = {};
    now.seconds        = static_cast<uint32_t>(tv.tv_sec);
    now.nanoseconds    = static_cast<uint32_t>(tv.tv_usec) * 1000;
    return esp_eth_mac_set_ptp_time(s_mac, &now);
}

/* The MAC stops transmitting now and then -- ARP included, so the box goes
 * dark while it still reports a link -- and nothing in the driver clears it. A
 * restart of the MAC does, so the failure is watched for here and escalated
 * once it is clearly not a one-off, with a floor so a stuck restart cannot
 * spin. The errno's own log line is the evidence trail. */
uint32_t gTxFails       = 0;
int64_t  gLastTxFailUs  = 0;
int64_t  gLastRestartUs = 0;
bool     gRestarting    = false;

void noteTxResult(esp_err_t err)
{
    if (err == ESP_ERR_INVALID_STATE) {
        return; /* link not up yet: not a fault */
    }
    const int64_t now = esp_timer_get_time();
    /* Anything sparse enough to be individual losses is left alone; a burst is
     * the stall. */
    if (now - gLastTxFailUs > 2000000) {
        gTxFails = 0;
    }
    gLastTxFailUs = now;
    if (++gTxFails < 8 || gRestarting || now - gLastRestartUs < 30000000) {
        return;
    }
    gRestarting    = true;
    gLastRestartUs = now;
    /* What the DMA itself says: a fatal bus error or an underflow halts the
     * transmit channel and nothing in the driver restarts it. The status word
     * is the difference between "the ring is full of stuck descriptors" and
     * "the channel stopped". */
    const uint32_t dmaStatus = EMAC_DMA.dmastatus.val;
    const uint32_t opMode    = EMAC_DMA.dmaoperation_mode.val;
    const uint32_t current   = EMAC_DMA.dmatxcurrdesc;
    /* The transmit descriptor the DMA is sitting on, sampled twice: a pointer
     * that does not move while frames are being queued is a stopped channel,
     * and one that moves is the ring full of descriptors nobody reclaimed. */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGW(TAG, "transmit stalled (%u in a row): dma status 0x%08x opmode 0x%08x "
                  "curtxdesc %08x -> %08x, restarting the MAC",
             gTxFails, static_cast<unsigned>(dmaStatus), static_cast<unsigned>(opMode),
             static_cast<unsigned>(current), static_cast<unsigned>(EMAC_DMA.dmatxcurrdesc));
    (void)EthernetLink::instance().restart("tx stalled");
    gTxFails    = 0;
    gRestarting = false;
}

/* The transmit path stamps a frame only when the descriptor flag is set for it,
 * and the driver sets that flag globally for every frame once timestamping is
 * on. That is the problem this works around: the MAC writes the timestamp back
 * into the descriptor after the frame has gone, and a descriptor that has been
 * handed to the next frame by then gets it in the words the DMA is reading --
 * `insufficient TX buffer size`, then nothing transmits at all. So the flag is
 * cleared at start-up and raised only around the frames whose stamp is read
 * back, under a mutex because two tasks send. */
esp_err_t transmit(const void *frame, size_t len, int64_t *stampNs)
{
    if (stampNs != nullptr) {
        *stampNs = 0;
    }
    if (s_eth == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!kTxStamp || stampNs == nullptr) {
        const esp_err_t plain = esp_eth_transmit_ctrl_vargs(s_eth, nullptr, 2, frame, len);
        if (plain != ESP_OK && plain != ESP_ERR_INVALID_STATE) {
            noteTxResult(plain);
        }
        return plain;
    }
    if (s_txMux == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_txMux, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uint32_t stampBit = EMAC_HAL_TDES0_TX_TS_ENABLE;
    esp_eth_ioctl(s_eth, static_cast<esp_eth_io_cmd_t>(ETH_MAC_ESP_CMD_SET_TDES0_CFG_BITS), &stampBit);
    eth_mac_time_t stamp = {};
    const esp_err_t err  = esp_eth_transmit_ctrl_vargs(s_eth, &stamp, 2, frame, len);
    esp_eth_ioctl(s_eth, static_cast<esp_eth_io_cmd_t>(ETH_MAC_ESP_CMD_CLEAR_TDES0_CFG_BITS), &stampBit);
    xSemaphoreGive(s_txMux);
    if (err != ESP_OK) {
        noteTxResult(err);
        /* A frame offered while the driver still says the link is down is not a
         * fault: the task starts on the address, which is up a moment before
         * the driver's own link flag is. Only the rest are counted. */
        if (err == ESP_ERR_INVALID_STATE) {
            return err;
        }
        if ((++s_stats.txErrors % 100) == 1) {
            ESP_LOGW(TAG, "transmit: %s", esp_err_to_name(err));
        }
        return err;
    }
    if (stamp.seconds == 0) {
        ++s_stats.txMissing;
        return ESP_OK;
    }
    /* A stamp that is a second away from the counter was written by an older
     * frame and describes nothing; the caller falls back to software. */
    const int64_t got  = static_cast<int64_t>(stamp.seconds) * 1000000000LL +
                         static_cast<int64_t>(stamp.nanoseconds);
    const int64_t diff = got - nowNs();
    if (diff > 1000000000LL || diff < -1000000000LL) {
        ++s_stats.txMissing;
        return ESP_OK;
    }
    ++s_stats.txStamps;
    s_stats.lastNs = got;
    if (stampNs != nullptr) {
        *stampNs = s_stats.lastNs;
    }
    return ESP_OK;
}

uint32_t sum16(const uint8_t *p, size_t n, uint32_t sum)
{
    while (n > 1) {
        sum += static_cast<uint32_t>((p[0] << 8) | p[1]);
        p += 2;
        n -= 2;
    }
    if (n != 0) {
        sum += static_cast<uint32_t>(p[0] << 8);
    }
    return sum;
}

uint16_t fold(uint32_t sum)
{
    while ((sum >> 16) != 0) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return static_cast<uint16_t>(~sum);
}

constexpr size_t kMaxUdpPayload = 64;

} // namespace

namespace ptp {

bool sendUdp(const uint8_t *dstAddr, size_t dstAddrLen, uint16_t dstPort, const uint8_t dstMac[6],
             const uint8_t *payload, size_t len, int64_t *stampNs)
{
    if (!s_enabled || s_netif == nullptr || s_eth == nullptr || dstAddr == nullptr ||
        dstMac == nullptr || payload == nullptr || len > kMaxUdpPayload) {
        fail(1);
        return false;
    }
    uint8_t ourMac[6] = {};
    if (esp_eth_ioctl(s_eth, ETH_CMD_G_MAC_ADDR, ourMac) != ESP_OK) {
        fail(2);
        return false;
    }

    constexpr size_t kEthLen = 14;
    /* Static, not on the stack: the responder task has a small stack and this
     * is one sender. */
    static uint8_t frame[kEthLen + 40 + 8 + kMaxUdpPayload];
    std::memset(frame, 0, sizeof(frame));
    std::memcpy(frame, dstMac, 6);
    std::memcpy(frame + 6, ourMac, 6);

    size_t   udpOff = 0;
    uint16_t udpLen = static_cast<uint16_t>(8 + len);
    if (dstAddrLen == 4) {
        esp_netif_ip_info_t info = {};
        if (esp_netif_get_ip_info(s_netif, &info) != ESP_OK || info.ip.addr == 0) {
            fail(3);
            return false;
        }
        const uint32_t src = info.ip.addr;
        frame[12]          = 0x08;
        frame[13]          = 0x00;
        uint8_t *ip        = frame + kEthLen;
        ip[0]              = 0x45;
        ip[2]              = static_cast<uint8_t>((20 + udpLen) >> 8);
        ip[3]              = static_cast<uint8_t>((20 + udpLen) & 0xff);
        ip[8]              = 64;
        ip[9]              = 17;
        std::memcpy(ip + 12, &src, 4);
        std::memcpy(ip + 16, dstAddr, 4);
        const uint16_t ipSum = fold(sum16(ip, 20, 0));
        ip[10]               = static_cast<uint8_t>(ipSum >> 8);
        ip[11]               = static_cast<uint8_t>(ipSum & 0xff);
        udpOff               = kEthLen + 20;
    } else if (dstAddrLen == 16) {
        /* LWIP_IPV6_NUM_ADDRESSES is three in this build, so the array covers
         * the netif and the returned count is bounded by it. */
        esp_ip6_addr_t addresses[3] = {};
        const int      count        = esp_netif_get_all_ip6(s_netif, addresses);
        const uint8_t *src          = nullptr;
        for (int i = 0; i < count; ++i) {
            const uint8_t *candidate = reinterpret_cast<const uint8_t *>(&addresses[i].addr);
            const bool linklocal = candidate[0] == 0xfe && (candidate[1] & 0xc0) == 0x80;
            const bool dstLinklocal = dstAddr[0] == 0xfe && (dstAddr[1] & 0xc0) == 0x80;
            if (linklocal == dstLinklocal) {
                src = candidate;
                break;
            }
        }
        if (src == nullptr) {
            fail(4);
            return false;
        }
        frame[12] = 0x86;
        frame[13] = 0xdd;
        uint8_t *ip = frame + kEthLen;
        ip[0]       = 0x60;
        ip[4]       = static_cast<uint8_t>(udpLen >> 8);
        ip[5]       = static_cast<uint8_t>(udpLen & 0xff);
        ip[6]       = 17;
        ip[7]       = 64;
        std::memcpy(ip + 8, src, 16);
        std::memcpy(ip + 24, dstAddr, 16);
        udpOff = kEthLen + 40;
    } else {
        fail(5);
        return false;
    }

    uint8_t *udp = frame + udpOff;
    udp[0]       = 0;
    udp[1]       = 123;
    udp[2]       = static_cast<uint8_t>(dstPort >> 8);
    udp[3]       = static_cast<uint8_t>(dstPort & 0xff);
    udp[4]       = static_cast<uint8_t>(udpLen >> 8);
    udp[5]       = static_cast<uint8_t>(udpLen & 0xff);
    std::memcpy(udp + 8, payload, len);

    /* IPv4 does not require a UDP checksum and a zero means "not computed";
     * IPv6 does, over a pseudo-header. */
    if (dstAddrLen == 16) {
        uint32_t sum = 0;
        sum          = sum16(frame + kEthLen + 8, 32, sum);
        sum += udpLen;
        sum += 17;
        sum = sum16(udp, udpLen, sum);
        const uint16_t c = fold(sum);
        udp[6]           = static_cast<uint8_t>(c >> 8);
        udp[7]           = static_cast<uint8_t>(c & 0xff);
    }

    const size_t total = udpOff + udpLen;
    /* No transmit stamp: the reply's arrival is the one that matters, and its
     * transmit field has to be written before the frame goes. */
    const esp_err_t err = transmit(frame, total, nullptr);
    if (s_stats.txStamps == 0) {
        ESP_LOGI(TAG, "first raw send: %u bytes to %u.%u.%u.%u:%u, %s",
                 static_cast<unsigned>(total), dstAddr[0], dstAddr[1], dstAddr[2], dstAddr[3],
                 static_cast<unsigned>(dstPort), esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        fail(6);
        ESP_LOGW(TAG, "transmit: %s", esp_err_to_name(err));
        return false;
    }
    /* Sent, stamped or not: the caller must not fall back to sendto() and put
     * a second copy on the wire. */
    return true;
}

bool takeRx(const uint8_t *addr, size_t addrLen, uint16_t port, Rx *out)
{
    if (addr == nullptr || out == nullptr || addrLen > 16) {
        return false;
    }
    const int64_t now   = nowNs();
    bool          found = false;
    int64_t       age   = 0;
    /* No logging inside this section: the responder's log hook sends over
     * lwIP, and this runs with interrupts off. The first miss is reported
     * below, once the lock is released. */
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < kRing; ++i) {
        RxStamp &slot = s_ring[i];
        if (!slot.used || slot.port != port || slot.addrLen != addrLen ||
            std::memcmp(slot.addr, addr, addrLen) != 0) {
            continue;
        }
        age = now - slot.ns;
        if (age < 0 || age > kStampAgeNs) {
            slot.used = false;
            continue;
        }
        out->ns = slot.ns;
        std::memcpy(out->mac, slot.mac, 6);
        slot.used = false;
        found     = true;
        break;
    }
    portEXIT_CRITICAL(&s_mux);

    if (found) {
        ++s_stats.hits;
    } else if (s_stats.hits == 0) {
        ESP_LOGW(TAG, "takeRx miss port %u addrLen %u (best age %lld ns; %u recorded)",
                 static_cast<unsigned>(port), static_cast<unsigned>(addrLen),
                 static_cast<long long>(age), static_cast<unsigned>(s_stats.rxStamps));
    }
    return found;
}

Stats stats()
{
    return s_stats;
}

esp_err_t start(Mode value, uint8_t domain, uint8_t priority1)
{
    if (value == Mode::Off) {
        stop();
        return ESP_OK;
    }
    if (s_eth == nullptr || s_mac == nullptr || !s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    stop();
    gDomain    = static_cast<uint8_t>(domain > 127 ? 127 : domain);
    gPriority1 = priority1;
    for (size_t i = 0; i < kDelayRing; ++i) {
        gDelay[i].seq = 0xffff;
    }
    gMode = static_cast<uint8_t>(value);
    if (xTaskCreate(&ptpTask, "ptp", kTaskStack, nullptr, 6, &gTask) != pdPASS) {
        gMode = 0;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "ptp %s, domain %u, priority1 %u, two-step",
             value == Mode::Master ? "master" : "slave", static_cast<unsigned>(gDomain),
             static_cast<unsigned>(gPriority1));
    return ESP_OK;
}

void stop()
{
    if (gMode != 0) {
        gMode = 0;
        /* The task deletes itself on its next tick; wait for it to be gone so a
         * restart cannot leave two senders. */
        for (int i = 0; i < 30 && gTask != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

Mode mode()
{
    return static_cast<Mode>(gMode);
}

const char *modeText(Mode value)
{
    switch (value) {
    case Mode::Master: return "master";
    case Mode::Slave:  return "slave";
    default:           return "off";
    }
}

} // namespace ptp
