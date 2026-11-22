/* Compiles this file's ESP_LOGD lines in; they stay hidden until raised with
 * the console's `log_level debug <tag>`. */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "ptp/ptp.h"

#include "soc/emac_dma_struct.h"

#include "net/eth_link.h"
#include "app/cores.h"
#include "gps/gps_time.h"

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

/* Entries live only until the responder's recvfrom(), so a short ring holds a
 * burst. */
constexpr size_t  kRing       = 16;
constexpr int64_t kStampAgeNs = 500000000LL; /* an entry older than this is noise */

struct RxStamp {
    uint8_t  addr[16];
    uint8_t  addrLen;
    uint8_t  mac[6];
    uint16_t port;
    bool     used;
    int64_t  ns;
};

esp_eth_handle_t s_eth      = nullptr;
esp_eth_mac_t   *s_mac      = nullptr;
esp_netif_t     *s_netif    = nullptr;
bool             s_attached = false;
bool             s_enabled  = false;
volatile bool    gSteered   = false;

}  // namespace

namespace ptp {
volatile bool gHardwareOn = false;
}

namespace {

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
RxStamp      s_ring[kRing];
size_t       s_next = 0;
ptp::Stats   s_stats{};
uint8_t      s_lastFail = 0;

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
}

/* Record the stamp of a UDP datagram to port 123, IPv4 or IPv6. */
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
 * Two-step only: the MAC cannot insert a stamp into an outgoing frame. The
 * counter is UTC, so the TAI-UTC offset is added to every stamp and announced. */

constexpr uint8_t kMsgSync      = 0x00;
constexpr uint8_t kMsgDelayReq  = 0x01;
constexpr uint8_t kMsgFollowUp  = 0x08;
constexpr uint8_t kMsgDelayResp = 0x09;
constexpr uint8_t kMsgAnnounce  = 0x0B;
constexpr uint8_t kPtpVersion   = 2;

constexpr size_t kHeader       = 34;
constexpr size_t kStamp        = 10; /* 48-bit seconds, 32-bit nanoseconds */
constexpr size_t kSyncLen      = kHeader + kStamp;
constexpr size_t kFollowUpLen  = kHeader + kStamp;
constexpr size_t kDelayRespLen = kHeader + kStamp + 10;
constexpr size_t kAnnounceLen  = kHeader + 30;

constexpr uint8_t kMulticast[6] = {0x01, 0x1b, 0x19, 0x00, 0x00, 0x00};
/* flagField octet 0: bit 0 alternateMaster, bit 1 twoStep, bit 2 unicast.
 * Octet 1: bit 0 leap61, bit 1 leap59, bit 2 currentUtcOffsetValid, bit 3
 * ptpTimescale, bit 4 timeTraceable, bit 5 frequencyTraceable. */
constexpr uint8_t kFlagTwoStep   = 0x02; /* octet 0 */
constexpr uint8_t kFlagUtcValid  = 0x04; /* octet 1 */
constexpr uint8_t kFlagTimescale = 0x08; /* octet 1 */
constexpr uint8_t kFlagTraceable = 0x30; /* octet 1, time and frequency */

constexpr int kTaskStack = 4096;

/* Hardware transmit stamps, requested per frame by the callers that use them
 * (the Sync, the slave's Delay_Req, and NTP replies to possibly interleaved
 * clients). Without one the Follow_Up carries the counter read before sending. */
constexpr bool kTxStamp = true;

volatile uint8_t  gMode      = 0;
uint8_t           gClock[8]  = {};
uint16_t          gPort      = 1;
uint8_t           gDomain    = 0;
uint8_t           gPriority1 = 128;
uint16_t          gSeq[16]   = {};
TaskHandle_t      gTask      = nullptr;
SemaphoreHandle_t s_txMux    = nullptr;

struct DelayReq {
    uint8_t  mac[6];
    uint8_t  clock[8];
    uint16_t port;
    uint16_t seq;
    int64_t  rxNs;
};
constexpr size_t kDelayRing = 8;

/* Slave side: one exchange with one master at a time. The offset is reported,
 * not applied. */
struct Exchange {
    bool     syncSeen;
    int64_t  t2; /* arrival of the Sync */
    uint16_t syncSeq;
    uint8_t  clock[8]; /* the master's port identity */
    uint16_t port;
    int64_t  t3; /* egress of our Delay_Req */
    uint16_t delaySeq;
    int64_t  t1; /* the master's origin, from the Follow_Up */
    int64_t  offsetNs;
    int64_t  delayNs;
    uint32_t measured;
    uint32_t timeouts;
};
Exchange     gSlave       = {};
uint32_t     gLastReportS = 0;
DelayReq     gDelay[kDelayRing];
size_t       gDelayNext = 0;
portMUX_TYPE gDelayMux  = portMUX_INITIALIZER_UNLOCKED;

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
    uint32_t frac = static_cast<uint32_t>(ns % 1000000000LL);
    for (int i = 0; i < 4; ++i) {
        p[9 - i] = static_cast<uint8_t>(frac >> (8 * i));
    }
}

/* TAI-UTC: NAV-TIMELS gives GPS-UTC, and TAI = GPS + 19 s. 18 s is assumed
 * until the receiver reports it. */
uint8_t utcOffset()
{
    /* leapSeconds(), not status(), which is too big for this task's stack. */
    const int leap   = static_cast<int>(GpsTime::instance().leapSeconds());
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
    /* Octet 0: transportSpecific (high nibble), messageType (low). Octet 1:
     * versionPTP. */
    out[0] = static_cast<uint8_t>(type & 0x0f);
    out[1] = kPtpVersion;
    put16(out + 2, static_cast<uint16_t>(length));
    out[4] = gDomain;
    out[6] = flags0;
    out[7] = flags1;
    /* correctionField (8) is left zero. */
    std::memcpy(out + 20, gClock, 8);
    put16(out + 28, gPort);
    put16(out + 30, seq);
    /* controlField (deprecated) is left 0 for every message type.
     * logMessageInterval 0 is 2^0 = 1 s. */
    out[32] = 0x00;
    out[33] = 0x00;
    return kHeader;
}

/* One PTP frame (ethertype 0x88F7) to `dest`, with its egress stamp if asked. */
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
    /* Pad to Ethernet's 60-byte minimum; messageLength is unchanged. */
    const size_t frameLen = 14 + length < 60 ? 60 : 14 + length;
    return ptp::transmit(frame, frameLen, stampNs);
}

void announcedQuality(uint8_t &clockClass, uint8_t &accuracy, uint8_t &timeSource)
{
    /* replyFacts(): locked includes the dispersion gate; demoted means locked
     * but over it. */
    const GpsTime::ReplyFacts facts = GpsTime::instance().replyFacts();
    if (facts.locked) {
        clockClass = 6;
        accuracy   = 0x25; /* 10 us */
        timeSource = 0x20;
    } else if (facts.demoted) {
        clockClass = 7;    /* demoted by the dispersion gate */
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
    uint8_t       body[kAnnounceLen] = {};
    const uint8_t flags0             = 0;
    uint8_t       quality            = 0;
    uint8_t       accuracy           = 0;
    uint8_t       source             = 0;
    announcedQuality(quality, accuracy, source);
    putHeader(body, kMsgAnnounce, kAnnounceLen, gSeq[kMsgAnnounce]++, flags0,
              static_cast<uint8_t>(kFlagUtcValid | kFlagTimescale | kFlagTraceable));
    putStamp(body + kHeader, taiNs());
    put16(body + kHeader + 10, utcOffset());
    body[kHeader + 13] = gPriority1;
    body[kHeader + 14] = quality;
    body[kHeader + 15] = accuracy;
    put16(body + kHeader + 16, 0x8000); /* offsetScaledLogVariance: unknown */
    body[kHeader + 18] = 255;           /* priority2 */
    std::memcpy(body + kHeader + 19, gClock, 8);
    put16(body + kHeader + 27, 0); /* stepsRemoved */
    body[kHeader + 29] = source;
    (void)sendPtp(kMulticast, body, kAnnounceLen, nullptr);
    ++s_stats.announceSent;
}

/* Sync, then the Follow_Up carrying its egress stamp. */
void sendSync()
{
    const uint16_t seq            = gSeq[kMsgSync]++;
    uint8_t        sync[kSyncLen] = {};
    putHeader(sync, kMsgSync, kSyncLen, seq, kFlagTwoStep, 0);
    /* The fallback if the Sync gets no hardware stamp. */
    const int64_t beforeNs = ptp::nowNs();
    int64_t       syncNs   = 0;
    if (sendPtp(kMulticast, sync, kSyncLen, kTxStamp ? &syncNs : nullptr) != ESP_OK) {
        return;
    }
    /* A Sync sent without a stamp still gets a Follow_Up. */
    if (syncNs == 0) {
        syncNs = beforeNs;
    }
    uint8_t follow[kFollowUpLen] = {};
    putHeader(follow, kMsgFollowUp, kFollowUpLen, seq, kFlagTwoStep, 0);
    putStamp(follow + kHeader, syncNs + static_cast<int64_t>(utcOffset()) * 1000000000LL);
    (void)sendPtp(kMulticast, follow, kFollowUpLen, nullptr);
    ++s_stats.syncSent;
}

/* One queued Delay_Req, or false when the queue is empty. */
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
    /* The task starts when the address is set, slightly before the MAC is up;
     * wait until a transmit is accepted. */
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
    uint8_t        body[kHeader + kStamp] = {};
    const uint16_t seq                    = gSeq[kMsgDelayReq]++;
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
    gSlave.port = get16(frame + 14 + 28);
}

void onFollowUp(const uint8_t *frame, uint16_t seq)
{
    if (!gSlave.syncSeen || seq != gSlave.syncSeq || gSlave.t2 == 0) {
        return;
    }
    gSlave.syncSeen        = false;
    const uint8_t *stamp   = frame + 14 + kHeader;
    uint64_t       seconds = 0;
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
    /* offset = ((t2 - t1) - (t4 - t3)) / 2, with t4 the receiveTimestamp in
     * this Delay_Resp. The master's TAI stamps are converted to UTC first. */
    const uint8_t *stamp   = frame + 14 + kHeader;
    uint64_t       seconds = 0;
    for (int i = 0; i < 6; ++i) {
        seconds = (seconds << 8) | stamp[i];
    }
    uint32_t fraction = 0;
    for (int i = 0; i < 4; ++i) {
        fraction = (fraction << 8) | stamp[6 + i];
    }
    const int64_t taiToUtc = static_cast<int64_t>(utcOffset()) * 1000000000LL;
    const int64_t t4       = static_cast<int64_t>(seconds) * 1000000000LL + fraction - taiToUtc;
    const int64_t t1       = gSlave.t1 - taiToUtc;
    const int64_t offset   = ((gSlave.t2 - t1) - (t4 - gSlave.t3)) / 2;
    gSlave.offsetNs        = offset;
    gSlave.delayNs         = ((t4 - gSlave.t3) + (gSlave.t2 - t1)) / 2;
    gSlave.t3              = 0;
    gSlave.t1              = 0;
    gSlave.t2              = 0;
    ++gSlave.measured;
    ++s_stats.measured;
    s_stats.lastPeerNs    = rxNs;
    s_stats.slaveOffsetNs = offset;
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

/* Queue a Delay_Req for the task to answer, keeping the RX path short. */
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
    s_stats.lastPeer[4] = frame[10];
    s_stats.lastPeer[5] = frame[11];
    s_stats.lastPeerLen = 4; /* the MAC in four "octets" plus two */
}

esp_err_t inputHandler(esp_eth_handle_t, uint8_t *buffer, uint32_t length, void *priv, void *info)
{
    const auto   *stamp = static_cast<const eth_mac_time_t *>(info);
    const bool    valid = stamp != nullptr && stamp->seconds != 0;
    const int64_t ns    = valid ? static_cast<int64_t>(stamp->seconds) * 1000000000LL +
                                   static_cast<int64_t>(stamp->nanoseconds)
                                : 0;

    if (length >= 14 && be16(buffer + 12) == 0x88F7) {
        ++s_stats.frames;
        if (valid) {
            s_stats.lastNs = ns;
        }
        if (length >= 14 + 34) {
            /* Parsed in place; the buffer is freed below. */
            notePeer(buffer);
            if (gMode == static_cast<uint8_t>(ptp::Mode::Master) && buffer[14] == kMsgDelayReq) {
                noteDelayReq(buffer, length, ns);
            } else if (gMode == static_cast<uint8_t>(ptp::Mode::Slave)) {
                switch (buffer[14]) {
                case kMsgSync: onSync(buffer, ns, get16(buffer + 14 + 30)); break;
                /* These two also read a stamp after the header. */
                case kMsgFollowUp:
                    if (length >= 14 + kHeader + kStamp) {
                        onFollowUp(buffer, get16(buffer + 14 + 30));
                    }
                    break;
                case kMsgDelayResp:
                    if (length >= 14 + kHeader + kStamp) {
                        onDelayResp(buffer, ns, get16(buffer + 14 + 30));
                    }
                    break;
                default: break;
                }
            }
        }
        free(buffer);
        return ESP_OK;
    }
    if (valid) {
        maybeRecordUdp(buffer, length, ns);
    }
    /* priv is the netif passed to esp_eth_update_input_path_info(). */
    return esp_netif_receive(static_cast<esp_netif_t *>(priv), buffer, length, NULL);
}

}  // namespace

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
    /* Clock identity: EUI-64 from the MAC (U/L bit set, ff:fe inserted). */
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

    /* Enabling a running block again stops it stamping, so disable first. */
    if (restart && s_enabled && s_mac != nullptr) {
        gSteered = false;
        (void)esp_eth_mac_ptp_disable(s_mac);
    }

    eth_mac_ptp_config_t config = {};
    /* 40 MHz crystal. */
    config.clk_src           = EMAC_PTP_CLK_SRC_XTAL;
    config.clk_src_period_ns = 25.0f;
    /* Also the sub-second increment, so the counter moves in 40 ns steps. The
     * addend is 2^32 * period / accuracy, and 25/25 would overflow it to 0. */
    config.required_accuracy_ns = 40.0f;
    config.roll_type            = ETH_PTP_DIGITAL_ROLLOVER;

    esp_err_t err = esp_eth_mac_ptp_enable(s_mac, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "phc enable: %s", esp_err_to_name(err));
        return err;
    }
    /* Stamp every received frame, for NTP; transmit stamps are per frame (see
     * transmit()). */
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
    esp_eth_ioctl(s_eth, static_cast<esp_eth_io_cmd_t>(ETH_MAC_ESP_CMD_CLEAR_TDES0_CFG_BITS),
                  &stampBit);
    if (restart) {
        (void)stepToRealtime();
    }

    /* From CLOCK_REALTIME, which may still be 1970; the clock step on the
     * first fix moves the counter too. */
    struct timeval tv = {};
    gettimeofday(&tv, nullptr);
    eth_mac_time_t start = {};
    start.seconds        = static_cast<uint32_t>(tv.tv_sec);
    start.nanoseconds    = static_cast<uint32_t>(tv.tv_usec) * 1000;
    err                  = esp_eth_mac_set_ptp_time(s_mac, &start);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "phc set: %s", esp_err_to_name(err));
    }

    s_enabled        = true;
    ptp::gHardwareOn = true;
    if (restart) {
        GpsTime::instance().dropPhcAnchor();
    }
    ESP_LOGI(TAG, "phc running, 40 ns steps, started from %u.%09u",
             static_cast<unsigned>(start.seconds), static_cast<unsigned>(start.nanoseconds));
    return ESP_OK;
}

bool enabled()
{
    return s_enabled;
}

esp_err_t setRatePpb(int32_t ppb)
{
    if (!s_enabled || s_mac == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t err = esp_eth_mac_adj_ptp_time(s_mac, ppb);
    if (err == ESP_OK) {
        gSteered = true;
    }
    return err;
}

bool steered()
{
    return gSteered;
}

esp_err_t stepBy(const int64_t deltaNs)
{
    if (!s_enabled || s_mac == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t  target = nowNs() + deltaNs;
    eth_mac_time_t to     = {};
    to.seconds            = static_cast<uint32_t>(target / 1000000000LL);
    to.nanoseconds        = static_cast<uint32_t>(target % 1000000000LL);
    return esp_eth_mac_set_ptp_time(s_mac, &to);
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

/* The MAC occasionally stops transmitting with the link still up, and only a
 * MAC restart clears it. A burst of failures triggers one, at most every 30 s. */
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
    if (now - gLastTxFailUs > 2000000) {
        gTxFails = 0;
    }
    gLastTxFailUs = now;
    if (++gTxFails < 8 || gRestarting || now - gLastRestartUs < 30000000) {
        return;
    }
    gRestarting    = true;
    gLastRestartUs = now;
    /* DMA status, to tell a halted channel from a full ring. */
    const uint32_t dmaStatus = EMAC_DMA.dmastatus.val;
    const uint32_t opMode    = EMAC_DMA.dmaoperation_mode.val;
    const uint32_t current   = EMAC_DMA.dmatxcurrdesc;
    /* Sampled again below: a descriptor pointer that does not move is a
     * stopped channel. */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGW(TAG,
             "transmit stalled (%u in a row): dma status 0x%08x opmode 0x%08x "
             "curtxdesc %08x -> %08x, restarting the MAC",
             gTxFails, static_cast<unsigned>(dmaStatus), static_cast<unsigned>(opMode),
             static_cast<unsigned>(current), static_cast<unsigned>(EMAC_DMA.dmatxcurrdesc));
    (void)EthernetLink::instance().restart("tx stalled");
    gTxFails    = 0;
    gRestarting = false;
}

/* The driver sets the transmit-stamp descriptor flag for every frame, and a
 * stamp written back into a descriptor already reused corrupted the ring
 * ("insufficient TX buffer size", then no transmit). So the flag is cleared at
 * start and set only around frames whose stamp is wanted, under a mutex. */
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
    esp_eth_ioctl(s_eth, static_cast<esp_eth_io_cmd_t>(ETH_MAC_ESP_CMD_SET_TDES0_CFG_BITS),
                  &stampBit);
    eth_mac_time_t  stamp = {};
    const esp_err_t err   = esp_eth_transmit_ctrl_vargs(s_eth, &stamp, 2, frame, len);
    esp_eth_ioctl(s_eth, static_cast<esp_eth_io_cmd_t>(ETH_MAC_ESP_CMD_CLEAR_TDES0_CFG_BITS),
                  &stampBit);
    xSemaphoreGive(s_txMux);
    if (err != ESP_OK) {
        noteTxResult(err);
        /* Link not up yet: not counted. */
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
    /* A stamp more than a second from now belongs to an older frame. */
    const int64_t got = static_cast<int64_t>(stamp.seconds) * 1000000000LL +
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

}  // namespace ptp

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
    /* Static: the responder's stack is small, and it is the only caller. */
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
        /* LWIP_IPV6_NUM_ADDRESSES is 3 in this build. */
        esp_ip6_addr_t addresses[3] = {};
        const int      count        = esp_netif_get_all_ip6(s_netif, addresses);
        const uint8_t *src          = nullptr;
        for (int i = 0; i < count; ++i) {
            const uint8_t *candidate    = reinterpret_cast<const uint8_t *>(&addresses[i].addr);
            const bool     linklocal    = candidate[0] == 0xfe && (candidate[1] & 0xc0) == 0x80;
            const bool     dstLinklocal = dstAddr[0] == 0xfe && (dstAddr[1] & 0xc0) == 0x80;
            if (linklocal == dstLinklocal) {
                src = candidate;
                break;
            }
        }
        if (src == nullptr) {
            fail(4);
            return false;
        }
        frame[12]   = 0x86;
        frame[13]   = 0xdd;
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

    /* IPv4 UDP checksum is optional (0); IPv6 requires it. */
    if (dstAddrLen == 16) {
        uint32_t sum = 0;
        sum          = sum16(frame + kEthLen + 8, 32, sum);
        sum += udpLen;
        sum += 17;
        sum = sum16(udp, udpLen, sum);
        /* A computed 0 is sent as 0xffff (RFC 8200 8.1). */
        const uint16_t folded = fold(sum);
        const uint16_t c      = folded == 0 ? 0xffff : folded;
        udp[6]                = static_cast<uint8_t>(c >> 8);
        udp[7]                = static_cast<uint8_t>(c & 0xff);
    }

    const size_t total = udpOff + udpLen;

    const esp_err_t err = transmit(frame, total, stampNs);

    static bool firstLogged = false;
    if (!firstLogged) {
        firstLogged = true;
        ESP_LOGD(TAG, "first raw send: %u bytes to %u.%u.%u.%u:%u, %s",
                 static_cast<unsigned>(total), dstAddr[0], dstAddr[1], dstAddr[2], dstAddr[3],
                 static_cast<unsigned>(dstPort), esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        fail(6);
        ESP_LOGW(TAG, "transmit: %s", esp_err_to_name(err));
        return false;
    }
    /* Sent, stamped or not: the caller must not send it again. */
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
    /* No logging under the lock: the log hook sends over lwIP. */
    portENTER_CRITICAL(&s_mux);
    /* Oldest match first, so queued requests from one sender keep their order. */
    RxStamp *best = nullptr;
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
        if (best == nullptr || slot.ns < best->ns) {
            best = &slot;
        }
    }
    if (best != nullptr) {
        out->ns = best->ns;
        std::memcpy(out->mac, best->mac, 6);
        best->used = false;
        found      = true;
    }
    portEXIT_CRITICAL(&s_mux);

    if (found) {
        ++s_stats.hits;
    } else if (s_stats.hits == 0) {
        ESP_LOGD(TAG, "takeRx miss port %u addrLen %u (best age %lld ns; %u recorded)",
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
    if (xTaskCreatePinnedToCore(&ptpTask, "ptp", kTaskStack, nullptr, 6, &gTask, cores::kWork) !=
        pdPASS) {
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
        /* Wait up to 1.5 s for the task, whose tick is 1 s, to exit. */
        for (int i = 0; i < 150 && gTask != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void setIdentity(uint8_t domain, uint8_t priority1)
{
    gDomain    = static_cast<uint8_t>(domain > 127 ? 127 : domain);
    gPriority1 = priority1;
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

}  // namespace ptp
