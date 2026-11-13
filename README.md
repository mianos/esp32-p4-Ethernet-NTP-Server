# esp32p4-ntp

An NTP server on the ESP32-P4: **Ethernet bring-up + DHCP + ping**, the device's
HTTP surface (`/healthz`, `/config`, `/config/reset`, `/ntp/sync`, `/gps/pps`)
using the same REST conventions as the other
[mianesp](https://github.com/mianos/mianesp) devices, **a stratum-1 reference of
its own** — a u-blox LEA-M8T on UART1, disciplined by its PPS line, with a frequency
servo that turns a three-second holdover into a ten-minute one (measured on this
board: 2 µs across 151 s of missing edges), and the upstream SNTP client kept as
the other source ([GPS](#gps-u-blox-lea-8--pps)) — and **a responder that serves
that clock to the LAN on UDP/123** — over IPv4 and IPv6, with a stratum that
admits what it does and does not know
([Serving NTP](#serving-ntp-udp123)). The HTTP surface and settings are
mianesp's own components (`webserver`, `jsonwrapper`, `nvsstoragemanager`,
`settingsbase`), not local copies of them.

Target board: **Waveshare ESP32-P4-ETH** (ESP32-P4NRW32, 32 MB flash,
IP101GRI 10/100 PHY, RJ45). It came up as an `/dev/cu.usbmodem*` node over the
USB-C cable (chip reported by esptool: ESP32-P4 rev v3.2).

The P4 has an on-chip 10/100 Ethernet MAC but **no PHY**, so the board's
IP101GRI is driven over RMII. Pins are hard-coded in `main/eth_link.cpp`:

| Signal      | GPIO | Notes                                  |
|-------------|------|----------------------------------------|
| MDC         | 31   | SMI clock                              |
| MDIO        | 52   | SMI data                               |
| PHY nRST    | 51   | active low                             |
| RMII REF_CLK| 50   | 50 MHz **input** from the PHY          |
| RMII TX_EN  | 49   |                                        |
| RMII TXD0   | 34   |                                        |
| RMII TXD1   | 35   |                                        |
| RMII RXD0   | 29   |                                        |
| RMII RXD1   | 30   |                                        |
| RMII CRS_DV | 28   |                                        |

PHY address 1 (IP101 default, confirmed: ID reads `0x02430c54`). The whole
table is verified working on this board — DHCP would not complete otherwise —
but if you ever swap boards, the `kEthPins` table at the top of
`main/eth_link.cpp` is the only thing to change.

## Status: working

Verified on this unit (2026-11-11): PHY `0x02430c54` at MDIO addr 1, link up,
DHCP lease `192.0.2.20/24` gw `192.0.2.1`, `ping` 4/4 replies at ~0.6 ms. HTTP
endpoints answered on the same lease; `/config` survives a reboot (NVS), and
`/config/reset` restores the compiled-in defaults. The clock steps from the
DHCP-provided NTP server (option 42) about 2.7 s after the lease, and `/healthz`
then reports `time_set: true` with a real date.

Updates go over the network once the two-slot partition table is in place: this
unit was moved `factory` → `ota_0` by USB, then `ota_0` → `ota_1` → `ota_0` by
`POST /firmware` over the RJ45, including the fix that turned a rejected
payload into a `400` rather than a `500`. Settings survived all of it.

IPv6 came up beside IPv4 on the same netif with no change to the HTTP handlers:
`fe80::200:5eff:fe00:5301` link-local plus the SLAAC global
`2001:db8:1:1:200:5eff:fe00:5301` advertised on the LAN, `ping6` to both, and
`/healthz` served over `http://[2001:db8:...]/healthz` as well as over IPv4 —
see [IPv6](#ipv6).

The responder serves NTP to both families from one dual-stack socket, and says
what it is: with the upstream clock fresh it answers `li=0 stratum=3
refid=192.0.2.1`, and macOS `sntp` takes it
(`+0.014526 +/- 0.002399 192.0.2.20`). Booted with `timeSource=off` it answers
`li=3 stratum=16 refid=INIT` and `sntp` refuses it with `Server not
synchronized` — which is the point ([Serving NTP](#serving-ntp-udp123)).

With the u-blox receiver wired (PPS on GPIO14, UART1 on 15/16) and
`timeSource=gps`, it is a stratum-1 server instead: `li=0 stratum=1
refid=GPS` over both families, locked 1.4 s after boot — before the lease
lands — and `/healthz` reported `gps_fix:3`, `gps_sats:5`,
`gps_pps_age_ms:301`, `gps_offset_us:29` while this was being written. Asked at
the same moment, macOS `sntp` put the board and the upstream server 2 ms apart
(`+0.136669 ± 0.001914` vs `+0.138697 ± 0.006242`): the two references agree to
within the LAN's own measurement noise, and the board's clock is the one that is
not drifting ([GPS](#gps-u-blox-lea-8--pps)).

The servo and the holdover it buys were measured rather than assumed
(2026-11-13). `esp_timer` on this unit runs 29.97 ppm slow; the fit says so in
about a minute, with 0.3 µs of residual. With the PPS suppressed for 151 s — the
same path a loose wire takes — the clock came back **2 µs** out, and the same test
before the rate was known came back 4.5 ms out. Suppressed for 701 s it came back
**4 µs** out, having demoted itself at ten minutes exactly as designed.
`POST /gps/pps` is that test, and
it stays in the firmware because a holdover nobody has exercised is a promise.
`NAV-TIMELS` is parsed too: this receiver reports a GPS-UTC offset of 18 s and no
pending leap, which is what every reply's leap indicator is built from.

## Build / flash

```sh
cd esp32p4-ntp
./build.sh flash      # build + flash over USB
./build.sh monitor    # build + flash + serial log (Ctrl-] to quit)
curl --data-binary @build/esp32p4_ntp.bin http://<ip>/firmware   # update over Ethernet
```

USB is only needed the first time (or to recover): it is the only transport the
ROM bootloader speaks, so the two-slot partition table has to be written with a
cable. After that, `POST /firmware` installs builds over the network — see
[Updating over Ethernet](#updating-over-ethernet-ota).

`build.sh` uses the ESP-IDF install `$IDF_PATH` points at (exported by
`idf.py export`, or set it yourself) and auto-detects the board's serial node
from `/dev/cu.usbmodem*`, so override with `PORT=...` if it picks the wrong one.
Use the `/dev/cu.*` node on macOS: `/dev/tty.*` intermittently reports `EBUSY`
here.

The lwIP SNTP/DHCP options, the IPv6/SLAAC options and the partition table
(`TWO_OTA_LARGE`: `nvs`, `otadata`, `phy_init`, `ota_0` 1700 K, `ota_1` 1700 K)
live in `sdkconfig.defaults`, which only seeds a *fresh* `sdkconfig`; an
existing one keeps its old values, so after pulling a change like this run
`rm sdkconfig` (or `idf.py menuconfig`) before building. `nvs` sits at `0x9000`
in both the old single-app table and this one, so settings survive the swap.


Console output goes to **UART0**, which on this board is wired to the USB-C
port's CH343 USB-UART bridge (VID `0x1A86`, PID `0x55D3`, enumerates as
`/dev/tty.usbmodem*`), i.e. the same cable used for flashing. The P4's
native USB-Serial/JTAG is not on that connector, which is why
`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` yields a completely silent log.

## What it does

1. `esp_eth_mac_new_esp32()` + RMII pin map → EMAC up.
2. `esp_eth_phy_new_generic()` → standard IEEE 802.3 PHY driver (IP101
   autonegotiates 10/100, full/half duplex).
3. `ESP_NETIF_DEFAULT_ETH()` → DHCP client starts automatically, and IPv6 comes
   up beside it: a `fe80::` link-local on every link-up, plus a global address
   from the router's advertisements (see [IPv6](#ipv6)).
4. lwIP answers **ICMP echo** once the netif has an address, over IPv4 and IPv6,
   so the board is pingable with no application code.
5. Diagnostics kept on purpose: at boot it scans the 32 MDIO addresses and
   prints the PHY ID, and while there is no IP it prints `BMSR` every 5 s
   (`link` / `autoneg_done`). If MDIO is miswired you get
   `phy: MDIO read failed` instead — a one-line answer to "are my pins right?".
6. Starts an HTTP server on port 80 with an address on the first DHCP lease
   (see [HTTP API](#http-api)). `esp_http_server` binds `in6addr_any` once lwIP
   has IPv6, so that one server is reachable over both families.
7. Starts a u-blox receiver's reference — UART1 plus the PPS interrupt — and,
   when `timeSource=gps`, serves its clock: a stratum-1 reference on the board
   itself ([GPS](#gps-u-blox-lea-8--pps)). With `timeSource=dhcp`/`static` the
   SNTP **client** (`NtpTime`) steps the clock instead, from the server DHCP
   option 42 named or one configured by hand, so the responder still has real
   time to serve ([Clock (test mode)](#clock-test-mode)).

The app code is C++ (`.cpp` + classes) on top of those components: `Settings`
is a `settingsbase`/`SettingsBase` subclass, the HTTP handlers are
`NtpWebServer : webserver`/`WebServer`, and `Settings`, `NtpTime`, `GpsTime`,
`NtpServer`, `NtpWebServer` and `EthernetLink` are singletons reached through
`instance()`, with httpd's, esp_event's and the GPIO interrupts' C callbacks as
static members that take the instance from the `arg` pointer. `webserver` is the
network-agnostic base — the WiFi endpoints (`/reset`, `/set_hostname`) live in
`wifiwebserver`, which this firmware does not pull, because the P4 has no radio.

`EthernetLink` is also where the C idioms go: the board's pads are one
`constexpr RmiiPins kEthPins` table copied in by `applyPinMap()`, the MAC and
PHY instances are `std::unique_ptr`s with `del()` deleters (released to the
driver once it owns them, so a failed bring-up has no cleanup branches), PHY
registers read as `std::optional<uint32_t>`, and the MAC arrives as a
`std::string`.

Each subsystem owns a `.[ch]pp` pair, so `main/main.cpp` holds `app_main()` and
nothing else — and `app_main()` is just boot order: NVS, settings, netif, clock,
link. The Ethernet link, which is the longest of those, is `eth_link.cpp`; the
boot sequence reads as five calls because of it.

## Expected log (captured from this board)

```
I (385) eth_demo: esp32p4-ntp: RMII + IP101, DHCP client, HTTP /healthz, NTP from GPS/PPS or upstream
I (395) settings: hostname=esp32p4-ntp tz=AEST-10AEDT,M10.1.0,M4.1.0/3 timeSource=gps ntpServer=(unset) serveNtp=on
I (395) gps_time: UART1 9600 8N1 (esp TX 15 -> module RX, esp RX 16 <- module TX), PPS on GPIO14
I (405) ntp_time: time source: gps (u-blox over UART, PPS on GPIO14)
I (585) gps_time: UBX frames arriving at 9600 baud
I (585) gps_time: module accepted the configuration (NAV-PVT + NAV-TIMELS at 1 Hz, NMEA off)
I (705) gps_time: 3D fix, 18 satellites, UTC 2026-11-13T12:17:19+1100 (nano 495480)
I (705) gps_time: clock set from the GPS (2026-11-13T12:17:19+1100): CLOCK_REALTIME now agrees with what the responder serves
I (805) gps_time: leap: GPS-UTC offset +18 s, nothing announced
I (955) eth_demo: MDIO addr 1: PHY ID 0x02430c54 (IP101 = 0x0243xxxx)
I (955) eth_demo: phy: bmsr=0x7849 link=0 autoneg_done=0 bmcr=0x3100
I (955) esp_eth.netif.netif_glue: 02:00:5e:00:53:01
I (955) esp_eth.netif.netif_glue: ethernet attached to netif
I (1405) gps_time: locked: stratum 1 from here on (PPS edges so far: 1, rate +0 ppb)
I (2965) eth_demo: Ethernet started
I (2965) eth_demo: link up   (MAC 02:00:5e:00:53:01)
I (2965) eth_demo: requesting DHCP lease ...
I (4395) eth_demo: IPv6 link-local: fe80::200:5eff:fe00:5301
I (4395) eth_demo: http over IPv6: http://[fe80::200:5eff:fe00:5301]/healthz
I (7395) eth_demo: IPv6 global: 2001:db8:1:1:200:5eff:fe00:5301
I (7395) eth_demo: http over IPv6: http://[2001:db8:1:1:200:5eff:fe00:5301]/healthz
I (7465) esp_netif_handlers: eth ip: 192.0.2.20, mask: 255.255.255.0, gw: 192.0.2.1
I (7465) eth_demo: DHCP lease: ip 192.0.2.20  mask 255.255.255.0  gw 192.0.2.1
I (7465) eth_demo: ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP
I (7475) WebServer: Starting server on port: 80
I (7485) eth_demo: http: http://192.0.2.20/healthz  http://192.0.2.20/config
I (7535) eth_demo: OTA image confirmed: rollback to the previous slot cancelled
I (7535) ntp_server: serving on udp/123 (dual-stack), reference: GPS/PPS, 3D fix, 18 satellites, stratum 1
```

The clock is real before the network is: the receiver has a fix at 0.7 s, the
leap notice 100 ms later, and the first PPS edge is paired with the fix in the
same breath, so by the time the lease arrives the responder's opening line already
says `stratum 1` — with `rate +0 ppb`, because a rate needs a minute of anchors
before it means anything ([The servo](#the-servo)). `nano` is the receiver's own
sub-second offset for that solution — a few hundred microseconds either side of
zero, which is the evidence that a NAV-PVT and a PPS edge describe the same
instant. On a boot with `timeSource=dhcp` instead, the GPS still runs and logs
the fix, it just does not steer anything, and the responder starts at `stratum 16`
until SNTP has set the clock.

The addresses, MAC and IPv6 prefix in this log are documentation values rather
than what this LAN handed out: `192.0.2.0/24` is TEST-NET-1 (RFC 5737),
`2001:db8::/32` is the IPv6 documentation prefix (RFC 3849), and the MAC is a
locally administered one. The link-local is that MAC's EUI-64, built the way the
board builds it, so the three stay consistent if you compare them by hand.

The two IPv6 lines arrive on their own schedule: the link-local at ~1.4 s after
`link up` (Duplicate Address Detection clearing `fe80::`), the global one a Router
Solicitation later — on this boot 3 s after the link-local and 70 ms before the
lease. `OTA image confirmed` only appears on the first boot of an image that
arrived over `POST /firmware`, which is what this one was.

Later boots show `settings: hostname=...` with no "no stored config" line, i.e.
the NVS blob was found and applied.

Notes: `0x02430c54` is the IP101 PHY ID (addr 1, as strapped on the board);
`bmsr=0x7849 link=0` before the cable is plugged in, `link=1` after. The P4 uses
its eFuse base MAC; the one shown here is the documentation MAC the IPv6 lines
are derived from.

## Test from your laptop

```sh
ping -c 4 <leased-ip>           # the address the lease above printed
```

Captured on this setup:

```
PING 192.0.2.20 (192.0.2.20): 56 data bytes
64 bytes from 192.0.2.20: icmp_seq=0 ttl=64 time=0.805 ms
64 bytes from 192.0.2.20: icmp_seq=1 ttl=64 time=0.799 ms
...
--- 192.0.2.20 ping statistics ---
4 packets transmitted, 4 packets received, 0.0% packet loss
```

A reply means the whole path (PHY, RMII clocking, DHCP, lwIP, ICMP) works.

## IPv6

IPv4 is still the default path — the DHCP lease, and the option-42 clock source
— and IPv6 runs beside it, not instead of it. Three config lines and two calls
are the whole implementation:

| Where | What |
|-------|------|
| `sdkconfig.defaults` | `CONFIG_LWIP_IPV6=y` (IDF's own default, spelled out because the rest depends on it), `CONFIG_LWIP_IPV6_AUTOCONFIG=y` for SLAAC, `CONFIG_LWIP_IPV6_NUM_ADDRESSES=3` |
| `eth_link.cpp`, `ETHERNET_EVENT_CONNECTED` | `esp_netif_create_ip6_linklocal()`. esp_netif does **not** create `fe80::` on its own, and without a source address lwIP has nothing to send the Router Solicitation from — so no RA comes back and there is no global address either. Called on every link-up, so a cable bounce re-arms it. |
| `eth_link.cpp`, `IP_EVENT_GOT_IP6` | logs each address as it lands, with the URL it makes reachable |

Nothing above that needed changing:

- `esp_http_server` binds `in6addr_any` whenever lwIP is built with IPv6, so
  `/healthz`, `/config`, `/ntp/sync` and `POST /firmware` answer over IPv6 the
  moment the netif has an address. Same handlers, same auth (none).
- The mianesp `webserver` component never formats an address of its own: the
  handlers get an `httpd_req_t`, and the base class logs the peer with
  `getpeername()` into a `sockaddr_in6`, unwrapping v4-mapped ones, so IPv6
  clients are named correctly with no change there either.
- lwIP answers ICMPv6 echo and NDP by itself, exactly like ICMPv4.

From another host on the same segment as the ping above:

```console
$ ping6 -c 3 -I en0 fe80::200:5eff:fe00:5301
16 bytes from fe80::200:5eff:fe00:5301%en0, icmp_seq=2 hlim=255 time=9.399 ms
--- fe80::200:5eff:fe00:5301 ping6 statistics ---
3 packets transmitted, 3 packets received, 0.0% packet loss

$ ping6 -c 3 2001:db8:1:1:200:5eff:fe00:5301
3 packets transmitted, 3 packets received, 0.0% packet loss
round-trip min/avg/max/std-dev = 0.827/1.139/1.474/0.265 ms

$ curl -g -s http://192.0.2.20/healthz | tr , '\n' | grep ipv6
 "ipv6_linklocal":"fe80::200:5eff:fe00:5301"
 "ipv6_global":"2001:db8:1:1:200:5eff:fe00:5301"

$ curl -g -s 'http://[2001:db8:1:1:200:5eff:fe00:5301]/healthz' | head -c 80
{"uptime":29,"time":"2026-11-11T17:16:11+1100","time_set":true,"version":"...

$ curl -g -s 'http://[fe80::200:5eff:fe00:5301%25en0]/healthz' | head -c 40
{"uptime":29,"time":"2026-11-11T17:16:11+1100
```

Two client-side details: `curl` needs `-g` (brackets are glob characters to it)
and the zone spelled `%25` rather than `%`; `ping6` takes `-I en0` instead. The
link-local address only works from that segment — a router does not forward it.

`/healthz` omits `ipv6_linklocal` and `ipv6_global` until each exists: the
link-local appears once Duplicate Address Detection clears it (about a second
after link-up), the global one only after a Router Advertisement. On a LAN with
no IPv6 router you get the link-local and nothing else, which is also the answer
to "why does ping6 work but curl to the global address not".

Deliberately not included:

- **DHCPv6.** SLAAC covers a board that wants one routable address and is what a
  normal LAN hands out; DHCPv6-only numbering would need
  `CONFIG_LWIP_IPV6_DHCP6=y` plus a hook to take the address it offers.
- **An IPv6 upstream for the clock.** lwIP's SNTP client resolves its server
  through `dns_gethostbyname()`, i.e. A records only, and RFC 2132 defines DHCP
  option 42 as a list of IPv4 addresses — so both `dhcp` and `static` clock modes
  stay on IPv4. `ntpServer` accepts and stores an IPv6 literal, but
  `ntp_time.cpp` logs `lwIP's SNTP client resolves A records only, so it cannot
  sync from it` rather than failing silently. (The *responder* has no such
  limitation: it answers IPv6 clients.)

The responder socket this list used to defer to now exists, and is bound exactly
as sketched here: `AF_INET6` with `IPV6_V6ONLY=0` on `::`, one socket for both
families — see [Serving NTP](#serving-ntp-udp123).

### Cost, and switching it off on a small board

Measured app sizes (`.bin` as flashed), same toolchain, IPv6 either in or out of
lwIP's build:

| Build | bytes | Δ vs IPv6 off |
|-------|------:|--------------:|
| ESP32-P4, IPv6 off | 495,328 | — |
| ESP32-P4, IPv6 + SLAAC | 533,552 | +38,224 (+7.7%) |
| ESP32-P4, IPv6 + SLAAC + NTP responder | 539,600 | +44,272 (+8.9%) |
| ESP32-P4, the same + GPS reference (this firmware) | 583,216 | +87,888 (+17.7%) |
| classic ESP32 (`modswitch`, mianesp stack), IPv6 off | 938,896 | — |
| classic ESP32, IPv6 on as IDF ships it | 969,440 | +30,544 (+3.3%) |
| classic ESP32, IPv6 + SLAAC | 969,456 | +30,560 (+3.3%) |

The GPS row is +43 KB, and almost none of it is `gps_time.cpp`, whose own code is
under 4 KB. `esp_driver_uart`'s `uart.c` alone is 31 KB at this toolchain's `-Og`,
and the GPIO ISR service adds the rest — driver code this firmware did not
previously link. That is 2.5% of a 1.7 MB app slot for a stratum-1 reference.

What that says:

- **The cost is lwIP's IPv6 stack, not this feature.** ~30–38 KB for IPv6 at all;
  SLAAC on top of it is **16 bytes** — a flag in the netif and a couple of
  branches that were compiled in either way.
- **The responder is 6,048 bytes** (533,552 → 539,600): the socket, the header
  builder, the rate limiter and the `/healthz` fields, with no new dependency.
- **Your ESP32 fleet is already paying it.** IDF 6 sets `CONFIG_LWIP_IPV6=y` by
  default, so those binaries have carried IPv6 since they moved to IDF 6 — they
  simply had nothing to give it an address. `modswitch` as shipped is the third
  row: 969,440 bytes with IPv6 compiled in and no SLAAC.
- **Switching it off is not a config line away.** mianesp's `webserver`
  component initialises a `sockaddr_in6` in `WebServer::callerAddress()`, and
  lwIP only defines that type when IPv6 is on: an IPv4-only build of this tree
  stops with `'sockaddr_in6' has initializer but incomplete type`. The numbers
  above for "IPv6 off" were produced with a six-line `#if CONFIG_LWIP_IPV6`
  fallback to `sockaddr_in` in a throwaway copy of that component — that is the
  change it would take to make IPv4-only builds possible again.
- The code in *this* repo is already guarded: `eth_link.cpp`, `net_state.cpp`
  and `web_server.cpp` put their IPv6 work behind `#if CONFIG_LWIP_IPV6`, so
  the same sources serve an IPv4-only board at zero cost once the component
  allows it.

Verdict: leave IPv6 on. 30 KB is 1.6% of a 1.9 MB OTA slot on a 4 MB module —
not worth a component patch and the loss of the IPv6 half of every endpoint —
and on the P4 (32 MB, ~0.5 MB app) it is noise.

## HTTP API

The endpoints follow the [mianesp](https://github.com/mianos/mianesp)
`components/webserver` conventions, so anything already talking to those
devices works here unchanged: success is a 2xx with the resulting state as the
body, failure is `{"error": "...", "statusCode": N}` with the matching status,
and unknown paths / wrong methods get the same JSON error shape instead of
httpd's built-in plain-text page.

| Method | Path            | Purpose                                          |
|--------|-----------------|--------------------------------------------------|
| GET    | `/healthz`      | uptime, clock, build, heap, link, address, clock source, servo |
| GET    | `/config`       | current settings                                 |
| POST   | `/config`       | apply any subset of the keys, persist, echo back |
| POST   | `/config/reset` | restore compiled-in defaults and persist         |
| POST   | `/ntp/sync`     | force an upstream poll, wait up to 5 s for it (with `timeSource=gps`: report the lock) |
| POST   | `/gps/pps`      | make the PPS interrupt ignore edges for `{"suppress_s":N}` seconds — the holdover test hook |
| GET    | `/firmware`     | build + slot info for the running image          |
| POST   | `/firmware`     | write the body into the inactive OTA slot, reboot into it |

`/healthz` on this unit:

```console
$ curl -s http://192.0.2.20/healthz
{"uptime":42,"time":"2026-11-11T18:07:30+1100","time_set":true,
 "version":"aa33def","idf_ver":"v6.0.1","partition":"ota_1","heap_free":524508,
 "hostname":"esp32p4-ntp","link_up":true,"ip":"192.0.2.20",
 "ipv6_linklocal":"fe80::200:5eff:fe00:5301",
 "ipv6_global":"2001:db8:1:1:200:5eff:fe00:5301",
 "ntp_source":"dhcp","ntp_server":"192.0.2.1",
 "ntp_reachable":true,"ntp_synced":true,"last_sync":1794380815,
 "serve_ntp":true,"ntp_stratum":3,"ntp_refid":"192.0.2.1",
 "ntp_leap":0,"ntp_root_delay_us":991.8,"ntp_root_dispersion_us":991.8,
 "ntp_requests":5}
```

The same JSON comes back over IPv6 — `/healthz` does not care which family the
request arrived on, and `ip`/`ipv6_*` describe the board, not the caller. See
[IPv6](#ipv6) for the two address fields and how to reach them.

`time_set` matters: there is no RTC, so the base `time` field is 1970 + uptime
until something disciplines it. `time_set` says whether the clock is plausible
(after 2020) rather than presenting 1970 as a fact, `ntp_synced` says whether
*this* client set it, `last_sync` is the epoch it last set, `ntp_server` is the
upstream actually in use (`null` until one is known) and `ntp_reachable` is the
RFC 5905 reachability register: non-zero means the server has answered. `ip` is
omitted until the DHCP lease lands, and `ipv6_linklocal` / `ipv6_global` are
omitted until each address exists.

With `timeSource=gps` the same object answers the same question about the other
source: `ntp_server` is `null` (nothing is polled, so there is no upstream),
`ntp_synced` is the receiver's lock, and `gps_fix` (u-blox fix type: 0 none,
2 2D, 3 3D), `gps_sats`, `gps_pps_age_ms` and `gps_offset_us` describe the
receiver itself — see [GPS](#gps-u-blox-lea-8--pps) for what those numbers mean.

`gps_tp5_ok` and `gps_timtp` are the receiver's answers to the two configuration
messages that decide whether any of the rest can be trusted: whether the 1 Hz
timepulse was asserted, and whether `NAV-TIM-TP` exists on this firmware
(`rejected`, `accepted` or `silent` — three different faults). `gps_module` is what
`MON-VER` said, and is absent because this unit does not answer it. When `TIM-TP`
does flow, `gps_tp_qerr_ns` is the quantisation the receiver admits to,
`gps_tp_jitter_ns` the spread the stamps actually show, and `gps_tp_on_second`
whether the pulse is on the UTC second.

`gps_ant`, `gps_ant_power`, `gps_noise_dbm`, `gps_agc`, `gps_jam_ind` and the
`gps_cn0_*` group describe the antenna and the front end behind it — see
[Antennas](#antennas-and-what-the-front-end-says-about-one) for which of those is
the one to compare two antennas with, and which is the tell for a preamp that is
not running.

The last group is the responder's: `serve_ntp` is whether it is listening,
`ntp_stratum` / `ntp_refid` / `ntp_leap` are what a reply would claim *right now*
(so they flip to `16` / `"INIT"` / `3` the moment the clock stops being fresh),
`ntp_root_delay_us` and `ntp_root_dispersion_us` are the two fields a client reads
to decide how much to trust it — the same numbers the wire carries, before 16.16
rounds them to 15.26 µs steps — and `ntp_requests` is how many requests it has
answered since boot. They are absent while it is not running. Reporting the root
figures here rather than leaving them to be inferred keeps the endpoint honest in
the same way the reply is: 2 µs of delay and 2.5 µs of dispersion from the GPS,
992 µs from an upstream nobody measured.

`/config` takes and returns a flat JSON object. Values are strings; keys that
are not registered are ignored, so an older/newer client will not break it:

| Key        | Default                          | Effect |
|------------|----------------------------------|--------|
| `hostname` | `esp32p4-ntp`                    | netif hostname, and the name the DHCP client sends as option 12. A change reaches the lease at the next renewal. |
| `tz`       | `AEST-10AEDT,M10.1.0,M4.1.0/3`   | POSIX TZ used by `localtime()` for `/healthz`. Presentation only — NTP always serves UTC. |
| `timeSource` | `dhcp`                         | `dhcp` takes the upstream NTP server from DHCP option 42, `static` polls `ntpServer`, `gps` disciplines the clock from the u-blox receiver over UART plus its PPS line ([GPS](#gps-u-blox-lea-8--pps)), `off` leaves the clock alone. Unknown values are treated as `off` and logged. |
| `ntpServer` | *(empty)*                       | Upstream NTP host or address, used only when `timeSource` is `static`. An IPv6 literal is stored but cannot sync — lwIP's SNTP client resolves A records only ([IPv6](#ipv6)). |
| `serveNtp` | `on`                            | `off` stops the UDP/123 responder within a second and starts it again on `on`, no reboot. Serving an unlocked clock is safe by construction: it answers stratum 16 with LI 3, which tells clients to ignore it. |

```console
$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"hostname":"p4-ntp","tz":"UTC0"}' http://192.0.2.20/config
{"hostname":"p4-ntp","tz":"UTC0","timeSource":"dhcp","ntpServer":"","serveNtp":"on"}

$ curl -s http://192.0.2.20/healthz
{"uptime":44,"time":"2026-11-11T02:38:44+0000","time_set":true, ... ,
 "hostname":"p4-ntp","link_up":true,"ip":"192.0.2.20", ...}

$ curl -s -X POST http://192.0.2.20/config/reset
{"status":"reset"}
```

## Updating over Ethernet (OTA)

The board's only interface is the RJ45, so the firmware carries an update
endpoint: `POST /firmware` with the raw app image as the body writes it into the
inactive slot and reboots into it.

```console
$ curl --data-binary @build/esp32p4_ntp.bin http://192.0.2.20/firmware
{"written":531328,"partition":"ota_0","rebooting":true}

$ sleep 20; curl -s http://192.0.2.20/firmware
{"version":"67ea5fc","idf_ver":"v6.0.1","built":"Nov 11 2026 16:12:36",
 "partition":"ota_0","confirmed":true,"ota":true,"next_partition":"ota_1","slot_size":1740800}
```

`partition` is where the running image came from, `next_partition` is where the
next upload lands: successive updates alternate `ota_0` / `ota_1`. That
alternation is also the quickest proof an update happened — uploading the same
build twice leaves `version` unchanged but flips the slot. `confirmed` is the
rollback handshake below: `true` means this image passed the reachability check
and will stay; `false` right after a write means it is still `PENDING_VERIFY`.

What checks the update:

- **Payload.** The first `esp_ota_write()` rejects a body whose magic byte is
  not an app image, and `esp_ota_end()` verifies the whole thing (chip id,
  checksum, segment hashes). Both answer
  `400 {"error":"not a valid app image for this chip"}`, and a rejected body
  leaves the boot slot untouched.
- **Size.** `Content-Length` must fit the slot, otherwise `413`.
- **Boot.** `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` marks a newly written image
  `ESP_OTA_IMG_PENDING_VERIFY`. `eth_link.cpp` confirms it with
  `esp_ota_mark_app_valid_cancel_rollback()` as soon as it has a DHCP lease and
  the HTTP server is listening; if an image never gets that far, the bootloader
  returns to the slot that worked. On a box with no other interface, that is the
  difference between a bad update and a trip to find a USB cable. While an image
  is still unconfirmed the endpoint answers `409` — a pending-verify app is not
  allowed to write another one.
- **Trust.** No authentication, like `/config`: anything on this LAN can replace
  the firmware. Fine for a bench network; on an untrusted one this needs a token
  or `esp_https_ota` with a signed image.

`"ota": false` in `GET /firmware` means the flashed partition table has no
second slot — `POST /firmware` then answers `500` and updating needs the cable.
`built` is the app descriptor's link-time stamp, which only moves when that
object is recompiled; use `version` (the git describe stamp) to identify a build.

## Clock (test mode)

There is no RTC on this board, so until something disciplines it `time()` is
1970 + uptime — useless for an NTP *server*. Two things fix that now: the GPS
above (the real reference, `timeSource=gps`) and `NtpTime`, the upstream SNTP
client, which is what `timeSource=dhcp` or `static` still uses — and what runs
when there is no receiver on the pins. It polls and **steps** (not slews) the
system clock; `sdkconfig.defaults` is the map:

| Setting                            | Why |
|------------------------------------|-----|
| `CONFIG_LWIP_DHCP_GET_NTP_SRV=y`   | lwIP asks the DHCP server for option 42 and hands the answer to SNTP |
| `CONFIG_LWIP_DHCP_MAX_NTP_SERVERS=2` | keep primary and backup from one lease |
| `CONFIG_LWIP_SNTP_MAX_SERVERS=2`   | matching SNTP slot count |

`NtpTime::start()` sets the re-poll interval to 60 s (`esp_sntp_set_sync_interval`)
instead of lwIP's default hour, so the responder's freshness window
([Serving NTP](#serving-ntp-udp123)) has fifteen polls of margin rather than
none: with the default, one lost reply would flip the server to stratum 16 for a
whole cycle.

Ordering is the fiddly part, and both halves are load-bearing:

1. `NtpTime::begin()` runs **before** `esp_eth_start()`, because lwIP only keeps
   an option-42 server if `sntp_servermode_dhcp()` was enabled when the DHCP ACK
   is processed. Getting that wrong is silent: the log just never shows
   `upstream time server`.
2. `onGotIp()` re-polls on the lease, because the first poll at boot may have
   gone out before the option-42 address existed. In `dhcp` mode with no option
   42 in the lease it warns once: `no NTP server in the lease (option 42)`.
3. Switching back to `dhcp` while a lease is already held cannot pick option 42
   up until the next DHCP bind — lwIP only hands the address to SNTP at that
   point, so `/ntp/sync` answers 504 in the meantime and `/healthz` shows
   `ntp_server: null`. The switch logs `option 42 arrives at the next DHCP
   bind`; a reboot applies it at once.

```console
$ curl -s -X POST http://192.0.2.20/ntp/sync        # poll now, wait up to 5 s
{"synced":true,"time":"2026-11-11T13:38:44+1100","ntp_server":"192.0.2.1"}

$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"timeSource":"static","ntpServer":"ntp.example.net"}' \
       http://192.0.2.20/config
{"hostname":"esp32p4-ntp","tz":"...","timeSource":"static","ntpServer":"ntp.example.net"}
```

With `timeSource=off` (or `static` and an empty `ntpServer`) `/ntp/sync` answers
`409 {"error":"no time source: set timeSource to dhcp, static or gps"}`; a poll
that gets no answer within 5 s is `504 {"error":"no time sync from the upstream
server"}`. Names are resolved with the DNS server from the lease, so
`ntpServer` can be a hostname.

This path is the fallback, not the reference: it steps the clock from somebody
else's server and cannot say how far away that server is. The stratum-1 path is
the GPS ([GPS](#gps-u-blox-lea-8--pps)), and the responder must not serve a clock
that is still slewing — hence the step here rather than a slew.

Errors (all captured on this unit):

```console
$ curl -s -X POST http://192.0.2.20/healthz
{"error":"method not allowed on /healthz","statusCode":405}
$ curl -s http://192.0.2.20/nope
{"error":"no such endpoint: /nope","statusCode":404}
$ curl -s -X POST -d '[]' http://192.0.2.20/config
{"error":"invalid JSON (expected an object)","statusCode":400}
$ curl -s -X POST http://192.0.2.20/config
{"error":"JSON body required","statusCode":400}
$ curl -s -X POST --data-binary @big.json http://192.0.2.20/config
{"error":"request body too large","statusCode":413}
$ curl -s -X POST http://192.0.2.20/ntp/sync      # with timeSource=off
{"error":"no time source: set timeSource to dhcp, static or gps","statusCode":409}
$ curl -s -X POST http://192.0.2.20/ntp/sync      # server silent for 5 s
{"error":"no time sync from the upstream server","statusCode":504}
```

Body size is capped at 4 KB, and string values at 63 characters
(`Settings::kMaxValueLen`); longer values are truncated and logged. `hostname`
has its own, smaller cap of 32 (`Settings::kMaxHostnameLen`), because
`esp_netif_set_hostname()` refuses anything longer — and refuses it at boot,
where the failure is fatal, so the limit is applied where the value is stored
rather than where it is used. That name travels twice in the DHCP request
(option 12 and ESP-IDF's vendor-class-identifier hook), which is why
`sdkconfig.defaults` also raises `CONFIG_LWIP_DHCP_OPTIONS_LEN` to 128: at the
default 69 the option area fits 14 characters, and anything longer makes lwIP
drop an option and print an `LWIP_ERROR` on every boot.

Settings live as one JSON blob in NVS (namespace `settings`, key `config`),
which is why they survive a reboot. `Settings` is a mianesp `SettingsBase`
subclass: the fields are registered with their compiled-in defaults in the
constructor via `field()`, so a new setting is one `field()` line plus its
member (and, if it needs to take effect immediately, an `onChange()` callback —
that is how a new `tz` reaches `setenv`/`tzset`).

The dependencies are mianesp components pulled over git (`main`), plus
`espressif/cjson` from the registry — cJSON left the IDF core in v6. The first
build downloads them into `managed_components/` (git-ignored), and it needs an
SSH key with read access to `mianos/mianesp`.

## GPS (u-blox LEA-M8T + PPS)

The reference the server exists for. `timeSource=gps` makes the board a
stratum-1 server: the date and time come from a u-blox LEA-M8T over UART, the second
boundary from its timepulse, and `CLOCK_REALTIME` is slewed to match.

| Signal     | GPIO | Direction                          | Carries |
|------------|------|------------------------------------|---------|
| module TX  | 16   | into the P4 (UART1 RX)             | `UBX-NAV-PVT` and `UBX-NAV-TIMELS`, once a second: UTC date/time, fix type, satellite count, leap-second offset |
| module RX  | 15   | out of the P4 (UART1 TX)           | configuration — nine short frames at boot |
| module PPS | 14   | into the P4, rising edge, interrupt | the top of the second |

UART1 at 9600 8N1, no flow control, which is the module's factory default. The
three pins are `kTxPin` / `kRxPin` / `kPpsPin` at the top of `main/gps_time.cpp`;
nothing else in the firmware touches them.

### Which wire does what

The UART says *which* second it is. The PPS says *when* that second started. That
split is the whole accuracy story, and it is why the receiver is not simply read
over the serial port:

- a NAV-PVT frame is 92 bytes and arrives a few tens of milliseconds after the
  boundary. It cannot tell you where in the second you are, so a server built on
  the UART alone would be tens of milliseconds out — worse than the upstream SNTP
  clock it would be replacing;
- the PPS edge is the other half: the interrupt handler stamps
  `esp_timer_get_time()` as it fires, so what limits it is interrupt latency
  (microseconds), not the frame time.

Walk through one second:

1. **Edge.** The ISR stamps the monotonic clock and bumps a counter. It runs from
   IRAM (`CONFIG_ESP_TIMER_IN_IRAM` makes the counter IRAM-safe), so an OTA write
   disabling the flash cache for a few milliseconds cannot turn an edge into a
   crash. Edges less than 0.5 s apart are ignored: that is header-pin bounce, or a
   module reconfiguring its pulse, not a second passing.
2. **Frame.** NAV-PVT arrives, the checksum is verified (a frame that fails it is
   dropped, never guessed at), and if the newest edge is under 900 ms old and has
   not already been paired, the two become an **anchor**: `(UTC second, monotonic
   instant)`.
3. **Serve.** Every timestamp the responder puts on the wire is the anchor plus
   elapsed monotonic time, corrected by the servo's rate (`esp_timer` here
   undercounts by 30 ppm, and 30 µs a second is not nothing). `GpsTimeRef::now()`
   does not read `CLOCK_REALTIME` at all, so an `adjtime()` correction in flight
   can never reach a reply, and two replies cannot go backwards.
4. **Steer.** The same anchor corrects the system clock, because the log lines,
   `/healthz` and the HTTP `Date` header read it: a step when there is no previous
   fix or the offset is over 250 ms, otherwise `adjtime()` with the residual — a
   few tens of microseconds a second, which IDF applies in seconds, not minutes.
5. **Notice.** NAV-TIMELS carries the leap-second offset and any announcement, and
   is kept for the first byte of a reply ([Accuracy](#accuracy-and-the-parts-that-are-not)).

The 900 ms pairing window is the honest part of step 2. If a frame arrives later
than that, it is not describing the edge that was stamped — the edge was missed,
or the receiver is late — and pairing it anyway would put the anchor a whole
second out. Such a second is skipped instead; if that keeps happening, the lock
expires, which is the correct answer.

### What the module has to be set to

The firmware configures the receiver over UART at every boot, with
`UBX-CFG-MSG`: `NAV-PVT` once a second on UART1 — the date, the fix, the satellite
count — `NAV-TIMELS` once a second beside it for the leap-second notice, and the
NMEA sentences (GGA, GLL, GSA, GSV, RMC, VTG, ZDA) off so the line carries only
what is read here. `NAV-TIMELS` is 24 bytes on a line that has room: the GPS-UTC
offset, whether a leap is announced, and the countdown to it, which is the only
source for the leap indicator in a reply's first byte.

The timepulse is **asserted**, not assumed. `UBX-CFG-TP5` goes out first — 1 Hz
and 1 Hz again for the locked case, `alignToTow`, rising edge, UTC grid, 100 ms
wide — with the antenna and user delays left at zero because the lead's delay is
not known and a made-up figure would be worse than none. A pulse somebody moved in
`u-center` is the one fault that makes this server confidently wrong rather than
merely stale, and 32 bytes per boot is cheaper than a README asking the reader to
go and look. The module's answer is reported, not swallowed: `/healthz` carries
`gps_tp5_ok`, and a refusal is a warning at boot.

Each configuration message is sent on its own and the firmware waits for that
message's own ACK or NAK before sending the next, because this receiver **does not
answer everything** — `MON-VER` gets silence, so `gps_module` stays absent — and an
order-based guess drifts apart inside one boot. The three replies it does give are
acted on directly rather than matched up afterwards.

**`UBX-NAV-TIM-TP`** is the obvious way to stop guessing: every pulse would arrive
with the receiver's own quantisation error (`qErr`) and its nominal time of week,
turning root delay from an assumption into a measurement. The receiver is a
LEA-M8T with `PROTVER 22.00`, well past the protocol 15 that introduced the
message, and it still refused the enable for most of a session — until a hardware
reset, after which the same bytes are acknowledged. So a refusal here is worth
suspecting as module state before capability; `/healthz` reports the answer as
`gps_timtp` (`rejected`, `accepted` or `silent`), and the three are worth keeping
apart for exactly that reason.

The message also stays silent without a time solution, so it says nothing while
the receiver has no fix — which is why the budget never depends on it. The jitter
it would have described is measured instead from the servo's own residuals
([the numbers](#accuracy-and-the-parts-that-are-not)), and `qErr` plus the
pulse-position check start contributing the moment both a fix and the message
exist.

Nothing here is followed by `UBX-CFG-CFG`: saving it to the receiver's flash would
outlive the firmware and take the module's own defaults away from `u-center`.
Eleven short frames per boot is the cheaper trade.

### Antennas, and what the front end says about one

The M8T **does have an antenna supervisor**, and it took a while to find because
one flag bit refuses the entire message. Every attempt used `0x0F` — supply,
short detect, open detect, power-down-on-short — and every one came back `NAK`,
which reads exactly like "no supervisor here". It is not: sweeping the bits one at
a time shows `svcs`, `scd` and `pdwnOnSCD` all accepted, and **bit 2, open-circuit
detection, is the one this firmware rejects**. With `0x0B` and the pins named, the
receiver reports `aStatus ok` and `aPower on` and drives the switch.

The working frame is `flags 0x0B, pins 20/27` — the `EXTINT` pair the board wires
to the transistor base and the short-sense divider. `gps_ant` and `gps_ant_power`
now report what the receiver says instead of `unknown`.

The lesson is worth keeping: a `NAK` is about the whole message, so any one
unsupported bit hides an otherwise complete feature. Vary the bits before
concluding a message is absent.

**The board is a WD22UGRC**, a Huawei carrier for the LEA-M8T, and it does not
generate antenna power: the 8-pin header takes `V_ANT` on pin 1 as a *supply
input* separate from Vcc on pin 2, and the transistor pair switches it. No supply
on pin 1 means no bias, however the receiver is configured — it was measured dry
here, and an unpowered preamp is a 10 to 20 dB attenuator: worse than a passive
antenna, not better. That is what `gps_cn0_max` 11 dBHz over a working fix was, and
when the path faded the last few dB it went to zero and stayed there.

The switch is the board's, not the firmware's: the two PNPs take base drive from
the module's `AADET_N` pin and report a short on `SCS1_N`, and the module drives
those once `CFG-ANT` is accepted. `V_ANT` on pin 1 still has to be fed — the board
switches a supply, it does not make one.

A GPIO drive on 17 with a sense on 18 exists as a fallback for a module that
cannot be configured, and costs nothing while unused.

The RF connector is SMB, and the SMA adapter often used on a bench is a snap-fit
with real loss and contact risk — worth reseating, and worth suspecting before the
antenna is blamed for a weak sky.

The module stays a LEA-M8T and is not suspected: unplugging and refitting the
antenna moves `gps_noise_dbm` by 2 dB, `gps_agc` by 78 counts and `gps_jam_ind` by
8, all in the direction an open connector and a matched one should move, so the RF
front end and the board's trace are both alive. What is missing is L1 at the pin.

What software can say is what the front end is doing, which turns out to be
enough to spot a dead antenna path without a supervisor:

| Key | Means |
|-----|-------|
| `gps_cn0_max` / `gps_cn0_mean` / `gps_cn0_used` | C/N0 on **used** satellites, in dBHz. This is how two antennas are compared. Satellite count is not: a better antenna adds the marginal ones last, and a worse one can hold the count up on the strongest handful while everything near the horizon goes. |
| `gps_noise_dbm` | `MON-HW` noise floor, and the preamp tell. A running preamp raises it to roughly −30…−50 dBm; **−100 dBm means the preamp is not powered, or the feed is open.** |
| `gps_agc` | AGC count, for context against the two above. |
| `gps_jam_ind` / `gps_jam` | Interference index, and the receiver's verdict where the firmware offers one. High alongside high C/N0 is a preamp that is too much for this receiver. |
| `gps_ant_age_s` | Age of the `MON-HW` reading, so a frozen set of these cannot pass for a live one. |

Two hooks exist for a bench session:

```console
$ curl -s -X POST -d '{"bias":"on"}' http://192.0.2.20/gps/ant
{"requested":"on","read_back_after_s":3,...}
$ curl -s -X POST http://192.0.2.20/gps/reset                      # hot start, almanac kept
$ curl -s -X POST -d '{"cold":true}' http://192.0.2.20/gps/reset  # discard the almanac
$ curl -s -X POST -d '{"hardware":true}' http://192.0.2.20/gps/reset
```

`/gps/ant` queues the change for the GPS task, which owns the UART, and the answer
is a `gps_ant_power` reading three seconds later. On this module it changes
nothing — there is no supervisor to accept it — but it is the right hook for one
that has.

The three reset strengths are ordered deliberately: a hot start restarts the
navigation engine; a cold one also throws the almanac away; a hardware reset
re-initialises the receiver and drops its whole runtime configuration, so the
message set is re-sent three seconds later — without that it comes back speaking
NMEA at 9600 and the parser has nothing to read. The hardware reset is the one
that changed anything here: it turned a refused `NAV-TIM-TP` enable into an
acknowledged one while leaving `CFG-ANT` refused. Reach for it before concluding
that a message is unsupported.

**The state while this was written is bad, and worth keeping as a baseline.** As
received, the board reported `gps_cn0_max` 11 and `gps_cn0_mean` 9 across 12 used
satellites with a 3D fix — 25 to 35 dB below a healthy active antenna — and then
fell to zero satellites and stayed there through a hot start:

```console
gps_fix 0  gps_sats 0  gps_cn0_max 0  gps_noise_dbm -100  gps_agc 5460  gps_jam_ind 58
```

Zero satellites while PPS still arrives, over a −100 dBm floor, is an open or
unpowered feed rather than a configuration fault. The firmware was cleared the hard way, and the
order matters if anyone has to do this again: the **original image from before any
of the antenna work** (`f84fe50`) was flashed onto a board that had been power
cycled at the wall, and it sat at zero satellites for three minutes. Nothing in
the firmware writes the RF path, so nothing in the firmware can be holding it.

Before that, and with nothing physical touched, the receiver had already been taken
through a hot start, a cold start, a hardware reset and a clear-to-defaults; all
four left it at zero, while the hardware reset was enough to make it acknowledge
messages it had been refusing. `NAV-PVT` and `NAV-TIMELS` are acknowledged and
arriving throughout and the timepulse keeps running. A high-gain antenna whose preamp is not powered is *worse*
than a passive one — the bypass path is a loss, not a gain — which is what 9 dBHz
looks like.

**Baud rate.** 9600 is tried first. With no valid UBX frame in 4 s the firmware
moves to 38400, 115200, 57600 and 19200, then starts over — a receiver that has
been through `u-center` once is usually not on 9600 any more, and sweeping five
baud rates at boot is cheaper for everyone than a datasheet. The rate that
produced frames is logged.

### Lock, holdover, and what a client sees

| Clock state | LI | stratum | refid | A client does |
|-------------|----|--------:|-------|---------------|
| fix ≥ 2D, valid time, PPS edge inside the window | 0 (1 or 2 in the minute before a leap) | 1 | `GPS` | trusts it |
| fix and a measured rate, edge older than 10 minutes | 3 | 16 | `INIT` | discards it |
| no rate yet, and no fix / no edge / edge over 3 s old | 3 | 16 | `INIT` | discards it |

Two windows, and the difference between them is the servo. Before the crystal's
rate is known — the first minute, sixty anchors — there is nothing to hold over
*with*: the anchor plus raw elapsed time is a clock whose second boundary is a
guess, so three missed edges demote the server and the log says so. Once the rate
is measured, the elapsed time is corrected by it, a ten-minute gap costs
microseconds rather than milliseconds, and the window stretches to ten minutes
with root dispersion growing the whole way (2.5 µs at the edge, 602 µs at the end
of it) so a client can see the claim decay instead of having to trust it.

The numbers below are from this unit, and they are the reason the second window
exists. Note what the first line is: not an estimate, a measurement taken when the
PPS came back.

```console
# 151 s with no PPS, before any rate had been measured (span from POST /gps/pps):
W (195684) gps_time: PPS back after 151.0 s: the servo was +4525 us out across the gap (+29.968 ppm residual)
# the same 151 s, with the servo carrying it:
W (428664) gps_time: PPS back after 151.0 s: the servo was -2 us out across the gap (-0.013 ppm residual)
# and 701 s, which is past the window: it demoted at 600 s and came back to this
W (1183644) gps_time: PPS back after 701.0 s: the servo was -4 us out across the gap (-0.006 ppm residual)
```

That is 4.5 ms of drift and then 2 µs of it, from a crystal that is 30 ppm out —
and 4 µs across eleven and a half minutes, where the crystal alone would have put
21 ms on the clock. Nothing was adjusted to make the second line look good: the
rate is refitted from the last five minutes of anchors every second, and a bigger
gap starts the window again rather than fitting a slope across the gap.

The window itself is a cliff, not a taper, and it is worth watching it happen:

```console
# at 600.8 s of age, mid-test: the ten minutes are up
W (1083234) gps_time: not locked (fix 3D, last edge 600825 ms ago): stratum 16 until it returns
$ curl -s http://192.0.2.20/healthz | tr ',' '\n' | grep ntp_
"ntp_stratum":16
"ntp_refid":"INIT"
"ntp_leap":3
```

One second earlier it was a stratum-1 answer with 602 µs of dispersion; one second
later it is `INIT` with LI 3, and `sntp` refuses it — which is the whole point of
having a window: a server that has not seen its reference for ten minutes should
say so in the one field a client is guaranteed to read.

Those are the numbers the budget produces. Sampled out of `/healthz` while a 121 s
holdover ran, and then the edge back:

```console
# POST /gps/pps {"suppress_s":120}, dispersion as the anchor ages away
"gps_pps_age_ms":28721   "gps_dispersion_us":30.529
"gps_pps_age_ms":56801   "gps_dispersion_us":58.529
"gps_pps_age_ms":84885   "gps_dispersion_us":86.529
"gps_pps_age_ms":112963  "gps_dispersion_us":114.529
# the first edge back, 121 s later
"gps_holdover_s":120.9964  "gps_holdover_error_us":-2.94
```

One microsecond a second from 2.5 µs at the edge, which is the 1 ppm wander term
and nothing else. The run ended claiming 114 µs for an error that came back at
2.94 µs: 39 times the truth, in the direction a promise should err.

There is no silent fallback to SNTP either. `timeSource` names the authoritative
source; if that source is unhealthy the answer is stratum 16, not a stratum whose
meaning quietly changed. Switching back is one `POST /config`, and it takes effect
without a reboot: `NtpServer` holds a single `TimeRef` that asks `Settings` on
every call and forwards to the GPS or the upstream implementation, so the serving
task never has a pointer swapped under it.

A receiver that is absent, mis-wired or talking a protocol this does not parse is
a log line and a stratum-16 reference — not a boot failure. That matters on a
headless board whose only other interface is the RJ45.

A holdover does not get to be quiet: the first anchor after a gap logs the gap and
the error, `/healthz` keeps `gps_holdover_s` and `gps_holdover_error_us` from the
last one, and the servo logs a step in the reference (100 ms or more) with the
window discarded, because that is the one failure that would otherwise look like a
healthy clock.

### The servo

Every anchor is one sample of `esp_timer` against the GPS, so the slope of the
last five minutes of them is the crystal's rate error, by least squares, in parts
per billion. On this board that is about `-30 ppm` (`-29.97` when the numbers below
were taken, `-30.1` on the next boot — it wanders with temperature, which is why
the window is ten minutes and not ten hours), and the sign is worth spelling
out: negative means the counter undercounts, so a GPS second takes slightly more
of its microseconds than a million, and the correction hands 30 µs back per
second. The fit's residual is 0.3 µs — the number that says these are samples of a
clock and not of the receiver's mood — and the rate it produced was worth 13 ppb
across a 151 s gap, measured rather than fitted.

The estimate is applied to every served timestamp (`now()` takes the rate out of
the elapsed part of the anchor-plus-elapsed), which does two things that are easy
to confuse:

- it **removes the sawtooth** between edges. Without it, uncorrected timestamps
  fall 30 µs behind each second and snap forward at the edge, which is invisible
  in a mean but not in a client that samples often;
- it **turns a holdover from three seconds into ten minutes**, because the error
  across a gap becomes the fit's own residual instead of the crystal's rate times
  the gap — 2 µs at 151 s and 4 µs at 701 s, instead of 4.5 ms and 21 ms, as
  measured above.

It deliberately does *not* steer the hardware. `esp_timer` has no rate knob, and
the EMAC's PTP counter — which does, in ppb — is the next step, not this one. So
`CLOCK_REALTIME` is still corrected by `adjtime()` phase only, and the servo is
the responder's business rather than the system's.

`POST /gps/pps` exists because the PPS line on this board is soldered. It makes
the interrupt ignore edges for a while, which is the same code path a loose wire
takes, and it is the only way to know what a holdover is worth without waiting for
a fault:

```console
$ curl -s -X POST -H 'Content-Type: application/json' -d '{"suppress_s":150}' \
       http://192.0.2.20/gps/pps
{"pps_suppressed_s":150,"locked":true,"stratum":1,"holdover_s":0}

$ curl -s http://192.0.2.20/healthz | tr ',' '\n' | grep holdover
"gps_holdover_s":150.9955
"gps_holdover_error_us":-2.032
```

`suppress_s` is capped at an hour. `/healthz` reports the time left as
`gps_pps_suppressed_s` while a test is armed, so an artificial demotion is never
mistaken for a fault.

### Accuracy, and the parts that are not

- **The edge** is stamped in the interrupt handler, so the limit is interrupt
  latency: single-digit microseconds. Not sub-microsecond — that needs a hardware
  counter, which the P4 has, and which is the next step below.
- **Between edges** the anchor is extended with `esp_timer_get_time()` (1 µs
  resolution) corrected by the servo's rate, so what is left is the fit's residual
  — 2 µs measured across a two-and-a-half-minute holdover — rather than 30 µs of
  crystal error accumulating over each second.
- **The reply stamps** (`t2`/`t3`) are still software: taken after `recvfrom()` and
  before `sendto()`, so a client also measures the LAN path and scheduler latency
  ([Serving NTP](#serving-ntp-udp123) is explicit about that). A disciplined
  reference removes the *clock* error; it cannot remove the *transport* error.
- **Leap seconds** now come from the receiver rather than being assumed. `NAV-TIMELS`
  is parsed at boot and once a second: it carries the GPS-UTC offset (18 s), and
  when it announces a leap, the last minute before it sets LI=1 (delete) or LI=2
  (insert) in the reply's first byte, as RFC 5905 7.3 asks. The guard matters:
  the module reports a non-zero `timeToLsEvent` even with nothing announced — 18 s
  of it, with `lsChange` 0, while this was being written — so the indicator is
  gated on the change type and the countdown together, and a countdown alone would
  put a warning in front of clients once a month for no reason. Nothing is smeared:
  the u-blox UTC fields already include the offset, the epoch stays POSIX, and the
  second that repeats or vanishes is the second the timestamp repeats or vanishes.
  Unlocked, LI is 3, because a server that has lost its reference should not be
  quoting a leap notice either.
- **What a client is told about the error** is root delay 2 µs and root dispersion
  2.5 µs at the edge, growing at 1 ppm of the anchor's age from there: 3.5 µs a
  second after the edge, 84 µs after 82 s, 602 µs at the end of the ten-minute
  window. The delay is the measurement path — two microseconds of interrupt
  latency, a named constant — plus half of any pulse quantisation the receiver
  states about itself; the dispersion adds the stamping jitter actually measured
  from the servo's residuals, 0.3 to 0.5 µs across the runs below. Both fields used
  to be worse than this: root delay was **0**, which was an assumption dressed as a
  number, and the dispersion was a flat 10 µs guess that no measurement stood
  behind. Growing at 1 ppm is the crystal wandering with temperature, not the fit —
  quoting the fit's ppb would be a promise about a room that has not been measured
  — and it is what makes the holdover honest: a client that watches dispersion can
  tell how stale the claim is getting, and when the window is exceeded the answer
  is stratum 16, not a slightly-more-wrong stratum 1.

  The measured budget is a *narrowing* — 2.5 µs at the edge against the old 10 —
  and it is worth knowing where the floor is: NTP's 16.16 field resolves 15.26 µs,
  and `toFixed()` rounds up so a microsecond clock never claims zero. At the edge
  both fields therefore go out as the same single least significant bit, and the
  improvement only becomes visible on the wire once the anchor has aged a second or
  two. `/healthz` carries the unrounded numbers, which is the honest place to read
  them.

### Enabling it, and what it looks like

```console
$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"timeSource":"gps"}' http://192.0.2.20/config
{"hostname":"esp32p4-ntp","tz":"...","timeSource":"gps","ntpServer":"","serveNtp":"on"}

$ curl -s http://192.0.2.20/healthz
{ ... "ntp_source":"gps","ntp_server":null,"ntp_synced":true,"last_sync":1794526208,
  "gps_fix":3,"gps_sats":18,"gps_pps_age_ms":82111,"gps_offset_us":30,
  "gps_rate_ppm":-29.968,"gps_rate_n":183,"gps_dispersion_us":84.5,
  "gps_lock_window_s":600,"gps_leap_s":18,"gps_leap_pending":false,
  "gps_tp5_ok":true,"gps_timtp":"rejected",
  "gps_holdover_s":150.9955,"gps_holdover_error_us":-2.032,
  "serve_ntp":true,"ntp_stratum":1,"ntp_refid":"GPS","ntp_leap":0,
  "ntp_root_delay_us":0,"ntp_root_dispersion_us":92,"ntp_requests":17}

$ curl -s -X POST http://192.0.2.20/ntp/sync     # nothing to poll: it reports the lock
{"synced":true,"time":"2026-11-13T10:30:25+1100","source":"gps","gps_fix":3,"gps_sats":5}
```

That dump is mid-holdover on purpose — `gps_pps_age_ms` 82111 is the dead edge,
`gps_dispersion_us` 92 is the claim decaying at 1 ppm of it, and the reference is
still serving because the rate is known.

`gps_pps_age_ms` counts 0→1000 every second: a value near 1000 means the last edge
is a whole second old, which is normal just before the next one arrives.
`gps_offset_us` is how far `CLOCK_REALTIME` was from the anchor at the last edge —
a few tens of microseconds here, and the number the slew is working on.
`ntp_synced` is the lock, not "was set once", so a receiver that loses its fix
changes its mind within the lock window.

The servo and the notice fields:

- `gps_rate_ppm` is the crystal's rate as the servo has measured it (negative:
  `esp_timer` undercounts), `gps_rate_n` how many anchors that rests on — it climbs
  to 60 and the first fit lands, which is when the holdover window becomes ten
  minutes;
- `gps_dispersion_us` and `gps_lock_window_s` appear only while locked, because
  they are what a reply would claim and a demoted reply claims neither;
- `gps_leap_s` is the GPS-UTC offset from `NAV-TIMELS`, `gps_leap_pending` whether
  a leap is announced, and `gps_leap_in_s` only while one is — together these are
  the first byte of every reply;
- `gps_pps_suppressed_s` exists only while `POST /gps/pps` has a test armed, and
  `gps_holdover_s` / `gps_holdover_error_us` only after the first holdover, so
  nothing in this group can be mistaken for a permanent state.

## Serving NTP (UDP/123)

`NtpServer` (`main/ntp_server.cpp`) answers NTPv3/v4 client requests on port 123
from one dual-stack socket: `AF_INET6`, `IPV6_V6ONLY=0`, bound to `::`, exactly
as [IPv6](#ipv6) recommended before it existed. It starts on the first DHCP
lease, next to the HTTP server, and serves the clock whichever source above set —
`stratum 3` from the upstream SNTP client, `stratum 1` from the GPS.

The design is one seam wide. A reply builder that knows the protocol, and a
`TimeRef` that knows where the time came from — `reference()` (stratum, refid,
when it last synced, locked or not), `now()` (the timestamp to stamp), and
`describe()` (one phrase for the boot log). There are two implementations,
`UpstreamTimeRef` and `GpsTimeRef`, and the packet code does not know which one
answered. Which one it is comes from `Settings::timeSource()`, re-read behind a
`SelectedTimeRef` on every call so `POST /config` can change it while the
responder is running.

### What it will say, and what it will not

| Clock state | LI | stratum | refid | A client does |
|-------------|----|--------:|-------|---------------|
| GPS locked: fix, valid time, PPS edge inside the window (3 s, or 10 min with a rate) | 0, or 1/2 in the minute before a leap | 1 | `GPS` | trusts it |
| upstream locked: synced less than 15 min ago | 0 | 3 | the upstream's address | trusts it |
| not locked | 3 | 16 | `INIT` | discards it |
| over the rate limit | 0 | 0 | `RATE` | backs off |

Stratum 3, not 2, and that is the honest number: lwIP's SNTP client does not
keep the upstream's own stratum, so this device cannot know whether it is one or
two hops from the reference. Claiming one hop more than a well-behaved internet
server over-states the distance; claiming one less under-states it, and a client
weights a closer server more heavily. Under-claiming is the safe error. With the
GPS the question does not arise — the reference *is* here, so the server says 1
([GPS](#gps-u-blox-lea-8--pps)).

The window is what makes "locked" mean something: fifteen missed polls of
`NtpTime`'s one-a-minute interval. `ntp_synced` in `/healthz` only records that
the clock was set at some point; a responder that keeps saying stratum 3 after
its upstream went away is serving drift. Past the window it
answers `li=3 stratum=16` — RFC 5905's way of saying *do not use me* — and macOS
`sntp` obliges:

```console
$ /usr/bin/sntp -t 2 192.0.2.20
sntp: Exchange failed: Server not synchronized
...
sntp: Clock select failed
        header: E4 (li:3 vn:4 mode:4)
       stratum: 10 (16)
           ref: 494E4954 (84.73.78.73)
         t_ref: 00000000.00000000 (0.000000000)
```

(Captured with `timeSource=off` and a reboot, so nothing had set the clock. That
is also how the `INIT` refid and the zero reference timestamp were verified.)

### On the wire

- **Answers mode 3 only, version 3 or 4, and replies in the request's version.**
  A mode 4 packet (another server), a mode 1 packet, version 2, or anything
  shorter than 48 bytes gets silence — answering a mode 4 is how two servers end
  up talking to each other forever.
- **Always 48 bytes out, never more than came in.** No MAC, no extension fields
  echoed, so this cannot be used as a reflector.
- **Origin timestamp is the client's transmit field byte for byte**, including
  an all-zero one; receive and transmit are taken around the reply build.
- **Precision `-20`** (about 1 µs — what this clock can distinguish), and root
  delay / root dispersion **as the reference states them**, because the two
  references are not the same distance away. From the GPS: root delay 2 µs and root
  dispersion 2.5 µs, + 1 ppm of the anchor's age — 602 µs at the end of the
  ten-minute holdover ([the numbers, and the measurement](#accuracy-and-the-parts-that-are-not)).
  From the upstream: root delay 65 units of 16.16 — 0.992 ms, the standing guess,
  since lwIP exposes neither the upstream's root delay nor our round trip to it —
  and root dispersion the same 0.992 ms plus 15 ppm of the time since the last
  sync, so a stale clock advertises its staleness instead of hiding it.
  `/healthz` reports both before the field rounds them
  (`ntp_root_delay_us` / `ntp_root_dispersion_us`), so the endpoint and the wire
  cannot disagree.
- **The leap indicator is the receiver's, not a constant.** 0 normally; 1 (delete)
  or 2 (insert) for the last minute before a leap the receiver has announced, from
  `NAV-TIMELS`; 3 when the reference is not locked, because a server without a
  clock has no business quoting a leap notice. Nothing is smeared, and from the
  upstream source it is 0, because lwIP's SNTP client does not surface the
  upstream's indicator.
- **refid** is the upstream's IPv4 address at stratum 3; a hostname upstream
  gets a stable FNV-1a of the name instead of RFC 5905's MD5 (which lwIP cannot
  give us the resolved bytes for). Above stratum 1 the field is informational —
  it is how ntpd spots a loop — so this is a documented deviation, not a guess.
  At stratum 1 from the GPS it is the four bytes `"GPS\0"`, which is the value
  RFC 5905 lists for a GNSS reference.
- **Rate limit**: 16-packet burst per source address, then one request per 2 s,
  then one `RATE` kiss-o'-death per second, then silence. Eight addresses are
  remembered; the table recycles the least recently seen rather than growing.
  The burst is sized for people, not for clients: `sntp` sends five queries per
  invocation, so two runs back to back (the obvious thing to try) stay answered.
- **Timestamps are software**: lwIP's port has no `SO_TIMESTAMP` (`lwipopts.h`
  and the Kconfig have no such option in IDF 6.0), so the receive stamp is taken
  after `recvfrom()` returns. The EMAC does have an IEEE 1588 unit — the P4 sets
  `SOC_EMAC_IEEE1588V2_SUPPORTED` — but `esp_eth` exposes it as a *steerable
  clock*, not a per-frame stamper: `esp_eth_mac_ptp_enable()`,
  `esp_eth_mac_get_ptp_time()` / `set_ptp_time()`, `esp_eth_mac_adj_ptp_freq()`,
  `esp_eth_mac_adj_ptp_time()` ("adjust base PTP time frequency increment by
  PPS", in ppb) and a target-time interrupt. Nowhere does a received frame hand
  back the descriptor's hardware timestamp, so making `t2` hardware-accurate
  means patching the driver, not calling an API. That scheduler latency is
  therefore part of the answer — which is why the dispersion above is stated
  rather than implied. What the clock *is* good for is the other half: a PPS
  edge can steer it in ppb, which is exactly the discipline mechanism the
  stratum-1 phase needs.

The log stays quiet on purpose — one line at start-up, one for the first request
of a boot, then one a minute (these are the format strings in the code, with
illustrative values):

```console
I (9124) ntp_server: serving on udp/123 (dual-stack), reference: upstream 192.0.2.1, stratum 3
I (15320) ntp_server: first request from 192.0.2.50:60371: stratum 3
I (75320) ntp_server: answered 12, KoD 1, dropped 3 in the last 60 s
```

### Verified on this unit

macOS `sntp` against the board. The offset is the board's clock against this Mac
— i.e. the reference's accuracy plus the step at sync — not a protocol error;
`delay` 0.36 ms is the whole round trip over the LAN.

```console
$ /usr/bin/sntp -d 192.0.2.20          # timeSource=gps
sntp_exchange {
        result: 0 (Success)
        header: 24 (li:0 vn:4 mode:4)
       stratum: 01 (1)
     precision: FFFFFFEC (9.536743e-07)
         delay: 0000.0000 (0.000000000)
    dispersion: 0000.0001 (0.000015259)
           ref: 47505300 ("GPS ")
         t_ref: EEA0E3E0.00000000 (4003521504.000000000)
            t2: EEA0E3E0.EF5AB909 (4003521504.934978069)
            t3: EEA0E3E0.EF5B2E79 (4003521504.934985069)
        offset: ... (0.152300570)        # this laptop's clock error, not the board's
         delay: ... (0.000355000)
}
```

`ref: 47505300` is `"GPS\0"`, the RFC 5905 stratum-1 refid. `t_ref` has a zero
fraction because it is an anchor: the integer second the last PPS edge marked.
`delay: 0000.0001` is root delay, and it used to be `0000.0000`: the reference is
the receiver on the board, not a server across a network, so there is no path to
measure in the ordinary sense — but there is an interrupt to enter, and calling
that zero was a claim nothing stood behind. One unit of NTP's 16.16 short format is
15.26 µs, which is what the measured 2 µs rounds up to; `toFixed()` rounds up
rather than down so a microsecond clock never files a zero. `dispersion` goes out
as the same unit for the same reason, and grows at 1 ppm of the anchor's age from
there: 84 µs at 82 s, 602 µs at the end of the ten-minute holdover. `/healthz`
reports both before that rounding (`ntp_root_delay_us` 2,
`ntp_root_dispersion_us` 2.5 — the numbers this paragraph is about). Compare with
the upstream case below, where the same arithmetic starts from 0.992 ms and grows
at 15 ppm of the poll interval.

With `timeSource=dhcp` instead:

```console
$ /usr/bin/sntp -d 192.0.2.20
selected:
sntp_exchange {
        result: 0 (Success)
        header: 24 (li:0 vn:4 mode:4)
       stratum: 03 (3)
          poll: 00 (1)
     precision: FFFFFFEC (9.536743e-07)
         delay: 0000.0041 (0.000991821)
    dispersion: 0000.0067 (0.001571655)
           ref: C0000201 (192.0.2.1)
         t_ref: EE9E928F.00000000 (4003369615.000000000)
        offset: ... (0.014526000)
         delay: ... (0.000342000)
+0.014526 +/- 0.002399 192.0.2.20 192.0.2.20
```

`ref: C0000201` is `192.0.2.1` on the wire, printed by sntp as a host-order
integer; `dispersion` 1.57 ms is the 0.992 ms floor plus the ~38 s since the last
sync at 15 ppm. Both are the `ntp_root_*` fields in `/healthz`, and both are
guesses about a path this device cannot measure — which is why the stratum is 3
and not 1.

The rest of the matrix, from `tools/ntp_test.py` (raw sockets, no dependencies, `recv` with a 1 s timeout):

| Sent | Answer |
|------|--------|
| mode 3, vn 4, 48 B over IPv4 | 48 B: `li=0 vn=4 mode=4 stratum=1`, `refid=GPS`, origin echoed |
| the same over the global IPv6 address | identical |
| mode 3, vn 3 | `vn=3` in the reply, same fields |
| mode 4 (server) | nothing |
| mode 1 (symmetric active) | nothing |
| vn 2 | nothing |
| 12 bytes | nothing |
| transmit timestamp all zeros | origin echoed as zeros |
| 20 in a burst from one address | 16 answered, then `RATE`, then silence — the script waits 0.5 s per silent packet, so tokens refilled during the run |
| after the burst, 2.5 s later | answered again |

### Testing it

Start with a client that is already on the machine, because that is the shape of
the question ("will anything accept my clock?"):

```console
$ sntp 192.0.2.20                 # macOS, BSD, Linux (ntp package)
+0.114044 +/- 0.002362 192.0.2.20 192.0.2.20
$ sntp -d 192.0.2.20              # the same, with every header field decoded
```

`chronyd -Q 'server 192.0.2.20 iburst'` (as root) and
`w32tm /stripchart /computer:192.0.2.20 /dataonly /samples:3` on Windows ask
the same question through different code.

`tools/ntp_test.py <ip> [ipv6]` answers the other one — what is actually on the
wire, including the packets a real client never sends: mode 4 / mode 1 / vn 2 /
truncated requests that must get silence, the origin-timestamp echo, the IPv6
address, and a burst that trips the limiter. It prints one line per packet:

```console
$ python3 tools/ntp_test.py 192.0.2.20 2001:db8::50
mode 3, vn 4                       len=48 li=0 vn=4 mode=4 stratum=3 prec=-20 delay=0.000992 disp=0.002121 refid=b'\xc0\x00\x02\x01' origin_ok=True
mode 4 (server)                    NO REPLY
...
answered=16 kod=2 silent=2
```

Two things to know before concluding something is broken:

- **`sntp` is five queries, not one.** Three back-to-back runs fit inside the
  16-packet burst; a tighter loop gets `RATE` and then a second of silence. Wait
  a few seconds — or run the test from another machine — before believing it.
- **`/healthz` is the witness.** `ntp_requests` climbs by one per packet it
  actually answered (a KoD and a dropped packet do not count), `ntp_stratum` and
  `ntp_refid` should read 3 and the upstream's address while locked, and
  `serve_ntp` should be `true`. If the counter climbs and the client still
  fails, the problem is in the client or the header, not the path.

The honesty path is worth testing too, because it is the one that matters when
something is wrong:

```console
$ curl -s -X POST http://192.0.2.20/config -d '{"timeSource":"off"}' >/dev/null
$ sntp 192.0.2.20
sntp: Server not synchronized
$ curl -s http://192.0.2.20/healthz | tr , '\n' | grep ntp_stratum     # 16
$ curl -s -X POST http://192.0.2.20/config -d '{"timeSource":"dhcp"}' >/dev/null
```

Switching back needs a DHCP bind before lwIP re-learns option 42 (a reboot, or a
lease renewal); until then the clock is unlocked and the responder keeps saying
stratum 16, which is correct.


`POST /config '{"serveNtp":"off"}'` stops it within a second (`/healthz` drops to
`"serve_ntp":false`, `sntp` gets `recv failure`), and `on` rebinds the same
socket without a reboot.

Not in this milestone, on purpose — or only half done:

- **NTS** (RFC 8915) — needs TLS and a key exchange; a LAN server gains little.
- **Broadcast / multicast** (RFC 5905 §9) — every client here polls.
- **Interleaved mode** (RFC 9769) — the server must remember each client's last
  transmit timestamp, which is not worth it before hardware stamping exists.
- **A servo that steers the oscillator itself.** The PPS gives a frequency
  measurement for free — the slope of the anchors, five minutes of them — and
  `GpsTimeRef` now fits it and applies it to every timestamp, which is what buys
  the ten-minute holdover ([The servo](#the-servo)). What is *not* steered is the
  hardware: `esp_timer` has no rate knob, so the system clock is still corrected
  every second with `adjtime()` phase only, and the fit lives in the reply path
  rather than in the counter every task reads. That is the EMAC's PTP counter and
  the driver work in [Next step](#next-step).

## Layout

| File                    | Contents                                                        |
|-------------------------|-----------------------------------------------------------------|
| `main/main.cpp`         | boot order only: NVS, settings, netif + event loop, the GPS, the clock, then the link |
| `main/eth_link.[ch]pp`  | `EthernetLink`: EMAC + PHY bring-up, pin map, PHY diagnostics, event handlers, netif, status task, web server and responder start, OTA image confirmation |
| `main/settings.[ch]pp`  | `Settings : SettingsBase` — the five device settings            |
| `main/web_server.[ch]pp`| `NtpWebServer : WebServer` — `/healthz` fields, `/config`, `/config/reset`, `/ntp/sync`, `/gps/pps`, `/firmware` |
| `main/time_ref.h`       | `TimeRef` / `TimeReference`: what the responder may claim (stratum, refid, leap, root figures), and the interface both sources implement |
| `main/gps_time.[ch]pp`  | `GpsTime` + `GpsTimeRef`: UART1 and the PPS interrupt, the UBX `NAV-PVT` / `NAV-TIMELS` parsers, the anchor, the frequency servo and its holdover, the slew of `CLOCK_REALTIME` |
| `main/ntp_time.[ch]pp`  | `NtpTime`: upstream SNTP client for `timeSource=dhcp`/`static`    |
| `main/ntp_server.[ch]pp`| `NtpServer` + `UpstreamTimeRef` + `SelectedTimeRef`: the UDP/123 responder, reply builder, rate limiter |
| `main/net_state.[ch]pp` | read-only netif / link / IPv6-address accessors, fed by `eth_link.cpp` |
| `tools/ntp_test.py`     | wire-level probe: silence cases, origin echo, IPv6, the rate limiter |

Written locally; everything else (JSON, NVS, settings machinery, the HTTP
server) is a mianesp component.

## Troubleshooting

- **`phy: MDIO read failed` or no `MDIO addr N` line at boot**: the SMI pins
  (MDC/MDIO) are wrong, or the PHY is held in reset (check `ETH_PHY_RST_GPIO`).
  On this board MDIO addr 1 answers with `0x0243xxxx`; anything else means
  wiring / PHY-address trouble, not lwIP trouble.
- **PHY ID prints but `link=0` forever**: no link partner. Check the cable and
  the switch port LED; link only comes up when the PHY sees a partner.
- **Link up but no lease** (`waiting for link + DHCP lease` keeps printing):
  the DHCP server must be on that L2 segment; some guest/VLAN networks need
  a MAC registration. For a static IP, fill `esp_netif_ip_info_t` and call
  `esp_netif_set_ip_info()` instead of relying on the DHCP client.
- **No serial output at all**: check `CONFIG_ESP_CONSOLE_UART_DEFAULT` is set.
  Logs only appear if the firmware prints to UART0 (the CH343 bridge); the
  P4's native USB-Serial/JTAG pins are not broken out on this connector.
- **Port busy / `[Errno 16] Resource busy`**: a leftover `esp_idf_monitor`
  still owns `/dev/cu.usbmodem*`; `lsof /dev/cu.usbmodem*` and kill it.
- **Nothing on port 80, but ping works**: the server starts on the first
  `IP_EVENT_ETH_GOT_IP`, so check the log for `WebServer: Starting server on port: 80`.
  If it says `httpd_start: ESP_ERR_HTTPD_...`, the port is already bound.
- **`/config` returns defaults after a reboot**: the NVS write failed — look for
  `settings: failed to persist config` in the log (usually a full partition).
- **`time_set` stays `false` / `time` reads 1970**: nothing set the clock. In
  `dhcp` mode look for `no NTP server in the lease (option 42)` — plenty of
  DHCP servers do not send one, and then `timeSource=static` with an explicit
  `ntpServer` is the fix. If there is no warning but no `clock set` line either,
  the poll is leaving the board but not coming back: check the server is
  reachable (`/ntp/sync` returns 504) and that UDP/123 outbound is allowed.
- **`ntp_time: upstream time server` never appears**: `NtpTime::begin()` must
  run before `esp_eth_start()` or lwIP discards option 42. Check the boot order
  in `app_main()`, and that `CONFIG_LWIP_DHCP_GET_NTP_SRV=y` survived a
  `sdkconfig` edit (run `idf.py menuconfig` → Component config → LWIP → SNTP).
- **A client says `Server not synchronized` / `Clock select failed`**: the
  responder is answering `li=3 stratum=16`, deliberately — the clock is not
  locked. Check `/healthz`: `ntp_synced` false, or `last_sync` older than
  fifteen minutes. Fix the upstream, not the responder. Serving a 1970 clock
  with `stratum=3` would be the actual bug.
- **`serve_ntp` is `false`**: `serveNtp` is `off`. If it is `on` and still
  `false`, the boot log has the reason: `bind udp/123: errno 98` means something
  else owns the port on the device, `cannot start the responder task` means heap
  (it wants 4 KB of stack). A failed start never takes the rest of the firmware
  with it — HTTP keeps answering, which is how you get in to see why.
- **`stratum` is 16 right after switching `timeSource` back to `dhcp`**: the
  clock has not resynced because lwIP only learns option 42 on a DHCP bind — the
  log says `option 42 arrives at the next DHCP bind (a reboot applies it now)`.
  Nothing wrong with the responder; a lease renewal or reboot clears it.
- **`stratum` drops to 16 ten minutes into a GPS outage**: that is the holdover
  window ending, not a new fault. `/healthz` says how long the edge has been gone
  (`gps_pps_age_ms`) and how much dispersion the replies had accumulated. If
  `gps_pps_suppressed_s` is present, someone armed `POST /gps/pps` and the outage
  is a test — `{"suppress_s":0}` stands the edges back up.
- **`gps_rate_n` climbs but `gps_rate_ppm` stays 0**: the fit needs 60 anchors, so
  that would be a boot less than a minute old. If `gps_rate_n` sits still instead,
  no anchors are being paired: the PPS interrupt or the UBX frames are missing
  (`no PPS edges` / `no UBX frames` in the log), and the holdover will stay at
  three seconds because there is no rate to hold with.
- **A client is getting `RATE` kiss-o'-death replies**: it is asking faster than
  one packet per 2 s after its 16-packet burst. macOS `sntp` sends five queries
  per invocation, so a loop of `sntp` runs will get there — wait a few seconds
  between runs, or drive it with `tools/ntp_test.py`, which sends one packet at
  a time and prints what came back. Confirm with `ntp_requests` in `/healthz`
  against the client's own count.
- **`ntp_refid` is a number for a hostname upstream**: that is the FNV-1a of the
  name, not its address — lwIP does not expose the address the SNTP client
  resolved ([Serving NTP](#serving-ntp-udp123)). Set `ntpServer` to the literal
  address if you want to see it there.
- **`ipv6_linklocal` is missing from `/healthz`**: IPv6 never came up. Check the
  log for `IPv6 link-local: ...` right after `link up` — that line comes from
  the `esp_netif_create_ip6_linklocal()` call in `eth_link.cpp`, and without it
  esp_netif leaves the netif with no IPv6 address at all even though lwIP was
  built for it. If the log line is there, the address is still in DAD: it shows
  up a second later.
- **`ping6` to the link-local works but the global address is unreachable**: no
  Router Advertisement reached the board, so SLAAC had nothing to build an
  address from. `ndp -rn` on macOS (or `ip -6 route` elsewhere) shows whether
  that segment is advertised at all; a link-local ping only proves the wiring,
  since the packet never leaves the L2 segment.
- **`curl` says `bad range` or `URL rejected`**: an IPv6 literal needs brackets
  *and* `-g` (curl treats `[` as a glob), and a link-local one needs the zone as
  `%25`: `curl -g 'http://[fe80::200:5eff:fe00:5301%25en0]/healthz'`.
- **`/config` accepted an IPv6 `ntpServer` but the clock stays at 1970**: by
  design — see the warning in the log and [IPv6](#ipv6); lwIP's SNTP client has
  no AAAA path, so the upstream clock is IPv4-only.
- **`POST /firmware` answers `500` "no OTA partition"**: the board is still on
  the old single-app table (`GET /firmware` says `"ota": false`). Flash once over
  USB with the current `sdkconfig.defaults`; `idf.py flash` writes the new table
  and `ota_data_initial.bin` for you.
- **`POST /firmware` answers `409`**: the running image has not confirmed itself
  yet (it has no lease yet, usually right after an update). Retry once `/healthz`
  answers.
- **An update "does not take": the device comes back on the other slot.** That
  is the rollback working — the new image never reached a lease + HTTP server, so
  the bootloader returned to the slot that had one. Look for
  `OTA image confirmed` in the log: absent means the image died earlier, and the
  serial console is the only place that says why.
- **First build fails fetching `espressif/cjson`**: the component registry is
  unreachable. Build once with network access; after that
  `managed_components/` is cached locally.

## Next step

The reference is here, the servo is here, and the leap notice is here. What is
left is making the *edges* as good as the clock, and the first two bullets below
are one change:

- **Sub-microsecond PPS capture, and a rate that steers hardware instead of
  arithmetic.** The ISR stamp is interrupt latency: single-digit microseconds, and
  it is the floor under everything above. The P4's EMAC has an IEEE 1588 counter
  that can be read from the same edge and steered in ppb
  (`esp_eth_mac_adj_ptp_time()`, `esp_eth_mac_adj_ptp_freq()`), which would make
  the edge hardware-sampled *and* give the servo somewhere to put its estimate
  other than `GpsTimeRef::now()`. The servo would then discipline the counter the
  whole system reads rather than the replies alone, and `adjtime()`'s phase-only
  schedule could go.
- **The receive-side accuracy** the dispersion admits to. It is *not* a call away:
  the PTP unit is exposed as a steerable clock, not as a per-frame timestamp, and
  lwIP has no `SO_TIMESTAMP` — so a hardware `t2` needs a driver patch that
  surfaces the DMA descriptor's stamp ([what the API actually offers](#serving-ntp-udp123)).
  Same driver work as the counter above, which is why they are one bullet.
- **A leap second that actually happened.** The path is built and reads a live
  offset, but LI has only ever been 0 here: the next leap is the test, and a
  receiver with a pre-loaded announcement would let it be rehearsed.

Until then this is a correct stratum-1 server: its clock is good to a few
microseconds, it holds over for ten minutes at 2 µs, and its replies are honest
about everything it cannot measure.
