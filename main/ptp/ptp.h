#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_eth_driver.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "soc/emac_ptp_struct.h"

/* The EMAC 1588 counter (PHC), the receive hook that stamps UDP/123 and
 * diverts PTP frames, raw UDP and PTP transmit, and the PTP master/slave.
 *
 * The PHC is set from CLOCK_REALTIME, rate-steered by the GPS servo, stepped
 * onto the PPS edge, and is the base NTP time is served from. It counts UTC;
 * PTP messages carry TAI. */
namespace ptp {

/* Install the receive hook. Call after the netif is attached; idempotent. */
esp_err_t attach(esp_eth_handle_t eth, esp_eth_mac_t *mac, esp_netif_t *netif);

/* Start the PHC from CLOCK_REALTIME. `restart` is for after a MAC stop/start,
 * which clears the block. */
esp_err_t enable(bool restart = false);
bool      enabled();

/* Steer the counter's rate by `ppb` (positive speeds it up). */
esp_err_t setRatePpb(int32_t ppb);

/* A rate has been pushed since the counter was last initialised, so its
 * elapsed time needs no further rate correction. */
bool steered();

/* Move the counter by deltaNs. The read-to-set gap is lost, so the caller
 * checks where it landed at the next edge. */
esp_err_t stepBy(int64_t deltaNs);

/* Set the counter to CLOCK_REALTIME, e.g. after the system clock is stepped. */
esp_err_t stepToRealtime();

/* A plain word, so the PPS interrupt can read it with the flash cache off. */
extern volatile bool gHardwareOn;

inline IRAM_ATTR bool hardware()
{
    return gHardwareOn;
}

/* The counter in ns, read from the registers so it is safe with the cache off.
 * Seconds and nanoseconds are not latched together, so the nanoseconds are read
 * either side of the seconds and the read is repeated across a rollover. */
inline IRAM_ATTR int64_t nowNs()
{
    uint32_t ns;
    uint32_t sec;
    uint32_t nsAfter;
    do {
        ns      = EMAC_PTP.sys_nanosec.ts_sub_seconds;
        sec     = EMAC_PTP.sys_seconds.ts_second;
        nsAfter = EMAC_PTP.sys_nanosec.ts_sub_seconds;
    } while (nsAfter < ns);
    return static_cast<int64_t>(sec) * 1000000000LL + ns;
}

/* One frame out, with the hardware transmit stamp if the PHC is running.
 * Returns the stamp in `stampNs`, or 0 when the frame went without one. */
esp_err_t transmit(const void *frame, size_t len, int64_t *stampNs);

/* A UDP/123 arrival stamp and the sender's MAC, which the reply is sent to
 * without an ARP or NDP lookup. */
struct Rx {
    int64_t ns     = 0;
    uint8_t mac[6] = {};
};

/* Take the stamp the receive hook recorded for this sender. */
bool takeRx(const uint8_t *addr, size_t addrLen, uint16_t port, Rx *out);

/* A UDP/123 datagram as a raw frame. With `stampNs` the frame is hardware
 * stamped (0 if the MAC gave none). False if it could not be built or sent, so
 * the caller can use sendto(). */
bool sendUdp(const uint8_t *dstAddr, size_t dstAddrLen, uint16_t dstPort, const uint8_t dstMac[6],
             const uint8_t *payload, size_t len, int64_t *stampNs);

enum class Mode : uint8_t { Off = 0, Master = 1, Slave = 2 };

/* Start or stop PTP. Domain 0..127, priority1 0..255. A master announces and
 * syncs; a slave only measures its offset from a master and reports it. */
esp_err_t start(Mode mode, uint8_t domain, uint8_t priority1);
void      stop();
/* Takes effect from the next message, without a restart. */
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
