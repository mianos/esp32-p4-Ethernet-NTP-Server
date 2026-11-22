#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_eth_driver.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "soc/emac_ptp_struct.h"

/* The 1588 counter, and the frames that carry timestamps.
 *
 * Two things share it and they are not the same job: the NTP responder wants a
 * hardware stamp on the datagrams it answers, and the PTP responder will want
 * the whole frame. Both need the clock running and the receive path diverted,
 * so that part lives here and neither protocol owns it.
 *
 * The PHC is not a third timebase. It is started from CLOCK_REALTIME once, and
 * what it is for is being read by hardware at the ends of a frame; the servo
 * still owns the reference and will steer it once the NTP path trusts it.
 */
namespace ptp {

/* Install the receive path that splits 0x88F7 frames off and records the stamp
 * of a UDP/123 datagram before lwIP sees it. Called once, after the netif is
 * attached; idempotent. */
esp_err_t attach(esp_eth_handle_t eth, esp_eth_mac_t *mac, esp_netif_t *netif);

/* Start or restart the PHC, from CLOCK_REALTIME. Idempotent unless `restart`,
 * which a MAC stop/start needs because it clears the block. */
esp_err_t enable(bool restart = false);
bool      enabled();

/* Move the counter onto CLOCK_REALTIME. A boot sets it while the clock is still
 * 1970, so the counter is stepped with the clock -- PTP counts epoch seconds
 * and a stamp 56 years out is not a small error. */
/* Steer the counter's rate by `ppb` (positive speeds it up). The servo measures the
 * crystal; without this the counter the PTP stamps come from free-runs at that
 * error while everything else is corrected. */
esp_err_t setRatePpb(int32_t ppb);

/* Whether the counter is being steered, so a caller that also scales elapsed
 * time knows not to do it twice. Cleared when the counter is re-initialised. */
bool steered();

/* Move the counter by deltaNs, read and set back to back. The time between
 * the two is lost, so the caller measures where it landed at the next edge
 * rather than trusting it. */
esp_err_t stepBy(int64_t deltaNs);

esp_err_t stepToRealtime();

/* Set from a task, read from the PPS interrupt: a plain word, so the handler
 * needs no call into flash. */
extern volatile bool gHardwareOn;

inline IRAM_ATTR bool hardware()
{
    return gHardwareOn;
}

/* The counter, in nanoseconds. IRAM-safe: the registers are read directly, so
 * the PPS interrupt may call it while the flash cache is off for an OTA write.
 * Reading the nanoseconds latches the seconds, so the two cannot straddle a
 * second boundary. */
inline IRAM_ATTR int64_t nowNs()
{
    const uint32_t ns  = EMAC_PTP.sys_nanosec.ts_sub_seconds;
    const uint32_t sec = EMAC_PTP.sys_seconds.ts_second;
    return static_cast<int64_t>(sec) * 1000000000LL + ns;
}

/* One frame out, with the hardware transmit stamp if the PHC is running.
 * Returns the stamp in `stampNs`, or 0 when the frame went without one. */
esp_err_t transmit(const void *frame, size_t len, int64_t *stampNs);

/* An arrival stamp for a UDP/123 datagram, with the address it came from and
 * the MAC it came from -- the reply goes back to that MAC, so no ARP or NDP
 * lookup stands between the stamp and the wire. */
struct Rx {
    int64_t ns     = 0;
    uint8_t mac[6] = {};
};

/* Recorded in the input path, taken by the responder after recvfrom() so the
 * reply can carry the arrival as t2 rather than a time read after the
 * scheduler. */
bool takeRx(const uint8_t *addr, size_t addrLen, uint16_t port, Rx *out);

/* One UDP datagram out from the box's own address, as a raw frame. With a
 * non-null `stampNs` the frame is sent for a hardware transmit stamp, and
 * `*stampNs` is zero when the MAC did not give one. False when the frame could
 * not be built or sent, so the caller can fall back to sendto(). */
bool sendUdp(const uint8_t *dstAddr, size_t dstAddrLen, uint16_t dstPort, const uint8_t dstMac[6],
             const uint8_t *payload, size_t len, int64_t *stampNs);

enum class Mode : uint8_t { Off = 0, Master = 1, Slave = 2 };

/* Start or stop answering PTP. Domain 0..127, priority1 0..255. Master only for
 * now; the slave side shares the same frames and the same counter. */
esp_err_t start(Mode mode, uint8_t domain, uint8_t priority1);
void      stop();
/* Domain and priority1 for a task that is already running: both are read per
 * message, so a change reaches the next Announce without a restart. */
void        setIdentity(uint8_t domain, uint8_t priority1);
Mode        mode();
const char *modeText(Mode value);

struct Stats {
    uint32_t frames; /* 0x88F7 frames seen */
    uint32_t syncSent;
    uint32_t announceSent;
    uint32_t delayResp;
    int64_t  lastPeerNs;    /* when the last neighbour frame arrived */
    int64_t  slaveOffsetNs; /* last measured offset against a master */
    uint8_t  lastPeer[16];
    uint8_t  lastPeerLen;
    uint32_t rxStamps; /* UDP/123 arrival stamps recorded */
    uint32_t txStamps; /* frames sent with a hardware stamp */
    uint32_t txMissing;
    uint32_t txErrors;
    uint32_t hits;
    uint32_t measured;
    int64_t  lastNs; /* most recent stamp of either kind */
};
Stats stats();

}  // namespace ptp
