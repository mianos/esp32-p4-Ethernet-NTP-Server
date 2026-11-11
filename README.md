# esp32p4-ntp

An NTP server on the ESP32-P4: **Ethernet bring-up + DHCP + ping**, the device's
HTTP surface (`/healthz`, `/config`, `/config/reset`, `/ntp/sync`) using the
same REST conventions as the other
[mianesp](https://github.com/mianos/mianesp) devices, a test-mode clock that
steps itself from the upstream NTP server the DHCP server hands out (option 42),
and **a responder that serves that clock to the LAN on UDP/123** — over IPv4 and
IPv6, with a stratum that admits what it does and does not know
([Serving NTP](#serving-ntp-udp123)). The HTTP surface and settings are
mianesp's own components (`webserver`, `jsonwrapper`, `nvsstoragemanager`,
`settingsbase`), not local copies of them.

Target board: **Waveshare ESP32-P4-ETH** (ESP32-P4NRW32, 32 MB flash,
IP101GRI 10/100 PHY, RJ45). This is what is currently on
`/dev/cu.usbmodem5CF71255951` (chip reported by esptool: ESP32-P4 rev v3.2).

The P4 has an on-chip 10/100 Ethernet MAC but **no PHY**, so the board's
IP101GRI is driven over RMII. Pins are hard-coded in `main/main.cpp`:

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
but if you ever swap boards, the `kEth*` constants at the top of `main/main.cpp`
are the only thing to change.

## Status: working

Verified on this unit (2026-11-11): PHY `0x02430c54` at MDIO addr 1, link up,
DHCP lease `131.84.1.107/24` gw `131.84.1.2`, `ping` from the Mac 4/4 replies
at ~0.6 ms. HTTP endpoints answered on the same lease; `/config` survives a
reboot (NVS), and `/config/reset` restores the compiled-in defaults. The clock
steps from the DHCP-provided NTP server (option 42) about 2.7 s after the lease,
and `/healthz` then reports `time_set: true` with a real date.

Updates go over the network once the two-slot partition table is in place: this
unit was moved `factory` → `ota_0` by USB, then `ota_0` → `ota_1` → `ota_0` by
`POST /firmware` over the RJ45, including the fix that turned a rejected
payload into a `400` rather than a `500`. Settings survived all of it.

IPv6 came up beside IPv4 on the same netif with no change to the HTTP handlers:
`fe80::82f1:b2ff:fedb:e228` link-local plus the SLAAC global
`2401:d002:120f:8701:82f1:b2ff:fedb:e228` advertised on this LAN, `ping6` from
the Mac to both, and `/healthz` served over `http://[2401:d002:...]/healthz` as
well as over IPv4 — see [IPv6](#ipv6).

The responder serves NTP to both families from one dual-stack socket, and says
what it is: with the upstream clock fresh it answers `li=0 stratum=3
refid=131.84.1.1`, and macOS `sntp` takes it
(`+0.025324 +/- 0.002325 131.84.1.107`). Booted with `timeSource=off` it answers
`li=3 stratum=16 refid=INIT` and `sntp` refuses it with `Server not
synchronized` — which is the point ([Serving NTP](#serving-ntp-udp123)).

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

`build.sh` uses the ESP-IDF v6.0.1 install at
`/Users/robertfowler/.espressif/v6.0.1/esp-idf` and the port
`/dev/cu.usbmodem5CF71255951` (override with `PORT=...`). Use the `/dev/cu.*`
node on macOS: `/dev/tty.*` intermittently reports `EBUSY` here.

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
7. Runs an SNTP **client** (`NtpTime`) that steps the clock once the lease
   arrives — from the server DHCP option 42 named, or from one configured by
   hand — so the NTP responder has real time to serve. See
   [Clock (test mode)](#clock-test-mode).

The app code is C++ (`.cpp` + classes) on top of those components: `Settings`
is a `settingsbase`/`SettingsBase` subclass, the HTTP handlers are
`NtpWebServer : webserver`/`WebServer`, and both `Settings` and `NtpTime` are
singletons reached through `instance()`, with httpd's C callbacks as static
members. `webserver` is the network-agnostic base — the WiFi endpoints
(`/reset`, `/set_hostname`) live in `wifiwebserver`, which this firmware does
not pull, because the P4 has no radio.

## Expected log (captured from this board)

```
I (375) eth_demo: esp32p4-ntp: RMII + IP101, DHCP client, HTTP /healthz, upstream clock
I (385) settings: hostname=esp32p4-ntp tz=AEST-10AEDT,M10.1.0,M4.1.0/3 timeSource=dhcp ntpServer=(unset)
I (385) ntp_time: time source: dhcp (upstream from option 42, once the lease lands)
I (935) eth_demo: MDIO addr 1: PHY ID 0x02430c54 (IP101 = 0x0243xxxx)
I (935) eth_demo: phy: bmsr=0x7849 link=0 autoneg_done=0 bmcr=0x3100
I (935) esp_eth.netif.netif_glue: 80:f1:b2:db:e2:28
I (935) esp_eth.netif.netif_glue: ethernet attached to netif
I (2945) eth_demo: Ethernet started
I (2945) eth_demo: link up   (MAC 80:f1:b2:db:e2:28)
I (2945) eth_demo: requesting DHCP lease ...
I (4385) eth_demo: IPv6 link-local: fe80::82f1:b2ff:fedb:e228
I (4385) eth_demo: http over IPv6: http://[fe80::82f1:b2ff:fedb:e228]/healthz
I (7445) esp_netif_handlers: eth ip: 131.84.1.107, mask: 255.255.255.0, gw: 131.84.1.2
I (7445) eth_demo: DHCP lease: ip 131.84.1.107  mask 255.255.255.0  gw 131.84.1.2
I (7445) eth_demo: ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP
I (7455) ntp_time: upstream time server: 192.0.2.1
I (7455) WebServer: Starting server on port: 80
I (7465) eth_demo: http: http://131.84.1.107/healthz  http://131.84.1.107/config
I (7515) eth_demo: OTA image confirmed: rollback to the previous slot cancelled
I (7915) ntp_time: clock set from 192.0.2.1: 2026-11-11T17:24:10+1100
I (11385) eth_demo: IPv6 global: 2401:d002:120f:8701:82f1:b2ff:fedb:e228
I (11385) eth_demo: http over IPv6: http://[2401:d002:120f:8701:82f1:b2ff:fedb:e228]/healthz
```

The upstream address is printed as `192.0.2.1` (TEST-NET-1) here rather than the
address this LAN's DHCP server actually hands out.

The two IPv6 lines arrive on their own schedule: the link-local at ~1.4 s after
`link up` (Duplicate Address Detection clearing `fe80::`), the global one ~7 s
after the lease (a Router Solicitation going out and the advertisement coming
back). `OTA image confirmed` only appears on the first boot of an image that
arrived over `POST /firmware`.

Later boots show `settings: hostname=...` with no "no stored config" line, i.e.
the NVS blob was found and applied.

Notes: `0x02430c54` is the IP101 PHY ID (addr 1, as strapped on the board);
`bmsr=0x7849 link=0` before the cable is plugged in, `link=1` after. The P4
uses its eFuse base MAC, `80:f1:b2:db:e2:28` on this unit.

## Test from your laptop

```sh
ping -c 4 <leased-ip>           # on this network: 131.84.1.107
```

Captured on this setup:

```
PING 131.84.1.107 (131.84.1.107): 56 data bytes
64 bytes from 131.84.1.107: icmp_seq=0 ttl=64 time=0.805 ms
64 bytes from 131.84.1.107: icmp_seq=1 ttl=64 time=0.799 ms
...
--- 131.84.1.107 ping statistics ---
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
| `main.cpp`, `ETHERNET_EVENT_CONNECTED` | `esp_netif_create_ip6_linklocal()`. esp_netif does **not** create `fe80::` on its own, and without a source address lwIP has nothing to send the Router Solicitation from — so no RA comes back and there is no global address either. Called on every link-up, so a cable bounce re-arms it. |
| `main.cpp`, `IP_EVENT_GOT_IP6` | logs each address as it lands, with the URL it makes reachable |

Nothing above that needed changing:

- `esp_http_server` binds `in6addr_any` whenever lwIP is built with IPv6, so
  `/healthz`, `/config`, `/ntp/sync` and `POST /firmware` answer over IPv6 the
  moment the netif has an address. Same handlers, same auth (none).
- The mianesp `webserver` component never formats an address of its own: the
  handlers get an `httpd_req_t`, and the base class logs the peer with
  `getpeername()` into a `sockaddr_in6`, unwrapping v4-mapped ones, so IPv6
  clients are named correctly with no change there either.
- lwIP answers ICMPv6 echo and NDP by itself, exactly like ICMPv4.

From this laptop (same board as the ping above):

```console
$ ping6 -c 3 -I en0 fe80::82f1:b2ff:fedb:e228
16 bytes from fe80::82f1:b2ff:fedb:e228%en0, icmp_seq=2 hlim=255 time=9.399 ms
--- fe80::82f1:b2ff:fedb:e228 ping6 statistics ---
3 packets transmitted, 3 packets received, 0.0% packet loss

$ ping6 -c 3 2401:d002:120f:8701:82f1:b2ff:fedb:e228
3 packets transmitted, 3 packets received, 0.0% packet loss
round-trip min/avg/max/std-dev = 0.827/1.139/1.474/0.265 ms

$ curl -g -s http://131.84.1.107/healthz | tr , '\n' | grep ipv6
 "ipv6_linklocal":"fe80::82f1:b2ff:fedb:e228"
 "ipv6_global":"2401:d002:120f:8701:82f1:b2ff:fedb:e228"

$ curl -g -s 'http://[2401:d002:120f:8701:82f1:b2ff:fedb:e228]/healthz' | head -c 80
{"uptime":29,"time":"2026-11-11T17:16:11+1100","time_set":true,"version":"...

$ curl -g -s 'http://[fe80::82f1:b2ff:fedb:e228%25en0]/healthz' | head -c 40
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
| ESP32-P4, IPv6 + SLAAC + NTP responder (this firmware) | 539,616 | +44,288 (+8.9%) |
| classic ESP32 (`modswitch`, mianesp stack), IPv6 off | 938,896 | — |
| classic ESP32, IPv6 on as IDF ships it | 969,440 | +30,544 (+3.3%) |
| classic ESP32, IPv6 + SLAAC | 969,456 | +30,560 (+3.3%) |

What that says:

- **The cost is lwIP's IPv6 stack, not this feature.** ~30–38 KB for IPv6 at all;
  SLAAC on top of it is **16 bytes** — a flag in the netif and a couple of
  branches that were compiled in either way.
- **The responder is 6,064 bytes** (533,552 → 539,616): the socket, the header
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
- The code in *this* repo is already guarded: `main.cpp`, `net_state.cpp` and
  `web_server.cpp` put their IPv6 work behind `#if CONFIG_LWIP_IPV6`, so the
  same sources serve an IPv4-only board at zero cost once the component allows
  it.

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
| GET    | `/healthz`      | uptime, clock, build, heap, link, address, clock source |
| GET    | `/config`       | current settings                                 |
| POST   | `/config`       | apply any subset of the keys, persist, echo back |
| POST   | `/config/reset` | restore compiled-in defaults and persist         |
| POST   | `/ntp/sync`     | force an upstream poll, wait up to 5 s for it    |
| GET    | `/firmware`     | build + slot info for the running image          |
| POST   | `/firmware`     | write the body into the inactive OTA slot, reboot into it |

`/healthz` on this unit:

```console
$ curl -s http://131.84.1.107/healthz
{"uptime":29,"time":"2026-11-11T17:57:03+1100","time_set":true,
 "version":"ac65668","idf_ver":"v6.0.1","partition":"ota_1","heap_free":524664,
 "hostname":"esp32p4-ntp","link_up":true,"ip":"131.84.1.107",
 "ipv6_linklocal":"fe80::82f1:b2ff:fedb:e228",
 "ipv6_global":"2401:d002:120f:8701:82f1:b2ff:fedb:e228",
 "ntp_source":"dhcp","ntp_server":"131.84.1.1",
 "ntp_reachable":true,"ntp_synced":true,"last_sync":1794380203,
 "serve_ntp":true,"ntp_stratum":3,"ntp_refid":"131.84.1.1","ntp_requests":0}
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

The last four fields are the responder's: `serve_ntp` is whether it is
listening, `ntp_stratum` / `ntp_refid` are what a reply would claim *right now*
(so they flip to `16` / `"INIT"` the moment the clock stops being fresh), and
`ntp_requests` is how many requests it has answered since boot. They are absent
while it is not running.

`/config` takes and returns a flat JSON object. Values are strings; keys that
are not registered are ignored, so an older/newer client will not break it:

| Key        | Default                          | Effect |
|------------|----------------------------------|--------|
| `hostname` | `esp32p4-ntp`                    | netif hostname, and the name the DHCP client sends as option 12. A change reaches the lease at the next renewal. |
| `tz`       | `AEST-10AEDT,M10.1.0,M4.1.0/3`   | POSIX TZ used by `localtime()` for `/healthz`. Presentation only — NTP always serves UTC. |
| `timeSource` | `dhcp`                         | `dhcp` takes the upstream NTP server from DHCP option 42, `static` polls `ntpServer`, `off` leaves the clock alone. Unknown values are treated as `off` and logged. |
| `ntpServer` | *(empty)*                       | Upstream NTP host or address, used only when `timeSource` is `static`. An IPv6 literal is stored but cannot sync — lwIP's SNTP client resolves A records only ([IPv6](#ipv6)). |
| `serveNtp` | `on`                            | `off` stops the UDP/123 responder within a second and starts it again on `on`, no reboot. Serving an unlocked clock is safe by construction: it answers stratum 16 with LI 3, which tells clients to ignore it. |

```console
$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"hostname":"p4-ntp","tz":"UTC0"}' http://131.84.1.107/config
{"hostname":"p4-ntp","tz":"UTC0","timeSource":"dhcp","ntpServer":"","serveNtp":"on"}

$ curl -s http://131.84.1.107/healthz
{"uptime":44,"time":"2026-11-11T02:38:44+0000","time_set":true, ... ,
 "hostname":"p4-ntp","link_up":true,"ip":"131.84.1.107", ...}

$ curl -s -X POST http://131.84.1.107/config/reset
{"status":"reset"}
```

## Updating over Ethernet (OTA)

The board's only interface is the RJ45, so the firmware carries an update
endpoint: `POST /firmware` with the raw app image as the body writes it into the
inactive slot and reboots into it.

```console
$ curl --data-binary @build/esp32p4_ntp.bin http://131.84.1.107/firmware
{"written":531328,"partition":"ota_0","rebooting":true}

$ sleep 20; curl -s http://131.84.1.107/firmware
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
  `ESP_OTA_IMG_PENDING_VERIFY`. `main.cpp` confirms it with
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
1970 + uptime — useless for an NTP *server*. `NtpTime` fixes that for now by
polling an upstream SNTP server and **stepping** (not slewing) the system clock;
`sdkconfig.defaults` is the map:

| Setting                            | Why |
|------------------------------------|-----|
| `CONFIG_LWIP_DHCP_GET_NTP_SRV=y`   | lwIP asks the DHCP server for option 42 and hands the answer to SNTP |
| `CONFIG_LWIP_DHCP_MAX_NTP_SERVERS=2` | keep primary and backup from one lease |
| `CONFIG_LWIP_SNTP_MAX_SERVERS=2`   | matching SNTP slot count |

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
$ curl -s -X POST http://131.84.1.107/ntp/sync        # poll now, wait up to 5 s
{"synced":true,"time":"2026-11-11T13:38:44+1100","ntp_server":"192.0.2.1"}

$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"timeSource":"static","ntpServer":"ntp.example.net"}' \
       http://131.84.1.107/config
{"hostname":"esp32p4-ntp","tz":"...","timeSource":"static","ntpServer":"ntp.example.net"}
```

With `timeSource=off` (or `static` and an empty `ntpServer`) `/ntp/sync` answers
`409 {"error":"no time source: set timeSource to dhcp or static"}`; a poll that
gets no answer within 5 s is `504 {"error":"no time sync from the upstream
server"}`. Names are resolved with the DNS server from the lease, so
`ntpServer` can be a hostname.

This is scaffolding: the real discipline path (GPS/PPS, or serving from a
disciplined source) replaces it, and the responder must not serve a clock that
is still slewing — hence the step.

Errors (all captured on this unit):

```console
$ curl -s -X POST http://131.84.1.107/healthz
{"error":"method not allowed on /healthz","statusCode":405}
$ curl -s http://131.84.1.107/nope
{"error":"no such endpoint: /nope","statusCode":404}
$ curl -s -X POST -d '[]' http://131.84.1.107/config
{"error":"invalid JSON (expected an object)","statusCode":400}
$ curl -s -X POST http://131.84.1.107/config
{"error":"JSON body required","statusCode":400}
$ curl -s -X POST --data-binary @big.json http://131.84.1.107/config
{"error":"request body too large","statusCode":413}
$ curl -s -X POST http://131.84.1.107/ntp/sync      # with timeSource=off
{"error":"no time source: set timeSource to dhcp or static","statusCode":409}
$ curl -s -X POST http://131.84.1.107/ntp/sync      # server silent for 5 s
{"error":"no time sync from the upstream server","statusCode":504}
```

Body size is capped at 4 KB, and string values are capped at 63 characters
(`Settings::kMaxValueLen`); longer values are truncated and logged.

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

## Serving NTP (UDP/123)

`NtpServer` (`main/ntp_server.cpp`) answers NTPv3/v4 client requests on port 123
from one dual-stack socket: `AF_INET6`, `IPV6_V6ONLY=0`, bound to `::`, exactly
as [IPv6](#ipv6) recommended before it existed. It starts on the first DHCP
lease, next to the HTTP server, and serves the clock the section above set.

The design is one seam wide. A reply builder that knows the protocol, and a
`TimeRef` that knows where the time came from — `reference()` (stratum, refid,
when it last synced, locked or not), `now()` (the timestamp to stamp), and
`describe()` (one phrase for the boot log). Today the only implementation is
`UpstreamTimeRef`, the SNTP clock above; GPS/PPS becomes a second one and the
packet code does not change.

### What it will say, and what it will not

| Clock state | LI | stratum | refid | A client does |
|-------------|----|--------:|-------|---------------|
| locked: synced less than 1 h ago | 0 | 3 | the upstream's address | trusts it |
| not locked | 3 | 16 | `INIT` | discards it |
| over the rate limit | 0 | 0 | `RATE` | backs off |

Stratum 3, not 2, and that is the honest number: lwIP's SNTP client does not
keep the upstream's own stratum, so this device cannot know whether it is one or
two hops from the reference. Claiming one hop more than a well-behaved internet
server over-states the distance; claiming one less under-states it, and a client
weights a closer server more heavily. Under-claiming is the safe error.

The 1 h window is what makes "locked" mean something. `ntp_synced` in `/healthz`
only records that the clock was set at some point; a responder that keeps saying
stratum 3 after its upstream went away is serving drift. Past the window it
answers `li=3 stratum=16` — RFC 5905's way of saying *do not use me* — and macOS
`sntp` obliges:

```console
$ /usr/bin/sntp -t 2 131.84.1.107
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

- **Answers mode 3 only, version 3 or 4.** A mode 4 packet (another server), a
  mode 1 packet, version 2, or anything shorter than 48 bytes gets silence —
  answering a mode 4 is how two servers end up talking to each other forever.
- **Always 48 bytes out, never more than came in.** No MAC, no extension fields
  echoed, so this cannot be used as a reflector.
- **Origin timestamp is the client's transmit field byte for byte**, including
  an all-zero one; receive and transmit are taken around the reply build.
- **Precision `-20`** (about 1 µs — what this clock can distinguish), **root
  delay 1 ms**, **root dispersion 1 ms + 15 ppm × time since the last sync**, so
  a stale clock advertises its staleness instead of hiding it.
- **refid** is the upstream's IPv4 address at stratum 3; a hostname upstream
  gets a stable FNV-1a of the name instead of RFC 5905's MD5 (which lwIP cannot
  give us the resolved bytes for). Above stratum 1 the field is informational —
  it is how ntpd spots a loop — so this is a documented deviation, not a guess.
- **Rate limit**: 8-packet burst per source address, then one request per 2 s,
  then one `RATE` kiss-o'-death per second, then silence. Eight addresses are
  remembered; the table recycles the least recently seen rather than growing.
- **Timestamps are software**: lwIP has no `SO_TIMESTAMP`, so the receive stamp
  is taken after `recvfrom()` returns. Until the hardware path exists (the EMAC's
  PTP clock), that scheduler latency is part of the answer — which is why the
  dispersion above is stated rather than implied.

The log stays quiet on purpose — one line at start-up, one for the first request
of a boot, then one a minute (these are the format strings in the code, with
illustrative values):

```console
I (9124) ntp_server: serving on udp/123 (dual-stack), reference: upstream 131.84.1.1, stratum 3
I (15320) ntp_server: first request from 131.84.1.95:60371: stratum 3
I (75320) ntp_server: answered 12, KoD 1, dropped 3 in the last 60 s
```

### Verified on this unit

macOS `sntp` against the board. The offset is the board's clock against this Mac
— i.e. the upstream server's accuracy plus the step at sync — not a protocol
error; `delay` 0.38 ms is the whole round trip over the LAN.

```console
$ /usr/bin/sntp -d 131.84.1.107
        header: 24 (li:0 vn:4 mode:4)
       stratum: 03 (3)
          poll: 00 (1)
     precision: FFFFFFEC (9.536743e-07)
         delay: 0000.0041 (0.000991821)
    dispersion: 0000.0057 (0.001327515)
           ref: 83540101 (1.1.84.131)
        offset: ... (0.033080500)
         delay: ... (0.000383000)
+0.033080 +/- 0.002206 131.84.1.107 131.84.1.107
```

`ref: 83540101` is `131.84.1.1` on the wire, printed by sntp as a host-order
integer; `dispersion` 1.33 ms is the 1 ms floor plus ~22 s of 15 ppm ageing.

The rest of the matrix, from a 40-line Python client (raw sockets, `recv` with a
1 s timeout):

| Sent | Answer |
|------|--------|
| mode 3, vn 4, 48 B over IPv4 | 48 B: `li=0 vn=4 mode=4 stratum=3`, origin echoed |
| the same over the global IPv6 address | identical |
| mode 4 (server) | nothing |
| mode 1 (symmetric active) | nothing |
| vn 2 | nothing |
| 12 bytes | nothing |
| transmit timestamp all zeros | origin echoed as zeros |
| 20 in a burst from one address | 9 answered, 4 `RATE`, 7 silent — the script waits 0.5 s per silent packet, so tokens refilled during the run |
| after the burst, 2.5 s later | answered again |

### Off, on, and what is deliberately missing

`POST /config '{"serveNtp":"off"}'` stops it within a second (`/healthz` drops to
`"serve_ntp":false`, `sntp` gets `recv failure`), and `on` rebinds the same
socket without a reboot.

Not in this milestone, on purpose:

- **NTS** (RFC 8915) — needs TLS and a key exchange; a LAN server gains little.
- **Broadcast / multicast** (RFC 5905 §9) — every client here polls.
- **Interleaved mode** (RFC 9769) — the server must remember each client's last
  transmit timestamp, which is not worth it before hardware stamping exists.
- **A stratum-1 reference.** That is the next phase, and the seam is ready: GPS
  over UART plus the PPS line, disciplining the EMAC's PTP clock, a `GpsTimeRef`
  returning `stratum = 1` / `refid = "GPS"`, and `now()` reading that clock
  instead of `CLOCK_REALTIME`. The responder itself does not change.

## Layout

| File                    | Contents                                                        |
|-------------------------|-----------------------------------------------------------------|
| `main/main.cpp`         | EMAC + PHY bring-up, event handlers, boot order, web server and responder start, OTA image confirmation |
| `main/settings.[ch]pp`  | `Settings : SettingsBase` — the five device settings            |
| `main/web_server.[ch]pp`| `NtpWebServer : WebServer` — `/healthz` fields, `/config`, `/config/reset`, `/ntp/sync`, `/firmware` |
| `main/ntp_time.[ch]pp`  | `NtpTime`: upstream SNTP client for the test-mode clock          |
| `main/ntp_server.[ch]pp`| `NtpServer` + `TimeRef`/`UpstreamTimeRef`: the UDP/123 responder, reply builder, rate limiter |
| `main/net_state.[ch]pp` | read-only netif / link / IPv6-address accessors, fed by `main.cpp` |

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
  locked. Check `/healthz`: `ntp_synced` false, or `last_sync` older than an
  hour. Fix the upstream, not the responder. Serving a 1970 clock with
  `stratum=3` would be the actual bug.
- **`serve_ntp` is `false`**: `serveNtp` is `off`. If it is `on` and still
  `false`, the boot log has the reason: `bind udp/123: errno 98` means something
  else owns the port on the device, `cannot start the responder task` means heap
  (it wants 4 KB of stack). A failed start never takes the rest of the firmware
  with it — HTTP keeps answering, which is how you get in to see why.
- **`stratum` is 16 right after switching `timeSource` back to `dhcp`**: the
  clock has not resynced because lwIP only learns option 42 on a DHCP bind — the
  log says `option 42 arrives at the next DHCP bind (a reboot applies it now)`.
  Nothing wrong with the responder; a lease renewal or reboot clears it.
- **A client is getting `RATE` kiss-o'-death replies**: it is asking faster than
  one packet per 2 s after its 8-packet burst. Confirm with
  `ntp_requests` in `/healthz` against the client's own count; a monitoring
  script hammering UDP/123 is the usual cause.
- **`ntp_refid` is a number for a hostname upstream**: that is the FNV-1a of the
  name, not its address — lwIP does not expose the address the SNTP client
  resolved ([Serving NTP](#serving-ntp-udp123)). Set `ntpServer` to the literal
  address if you want to see it there.
- **`ipv6_linklocal` is missing from `/healthz`**: IPv6 never came up. Check the
  log for `IPv6 link-local: ...` right after `link up` — that line comes from
  the `esp_netif_create_ip6_linklocal()` call in `main.cpp`, and without it
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
  `%25`: `curl -g 'http://[fe80::82f1:b2ff:fedb:e228%25en0]/healthz'`.
- **`/config` accepted an IPv6 `ntpServer` but the clock stays at 1970**: by
  design — see the warning in the log and [IPv6](#ipv6); lwIP's SNTP client has
  no AAAA path, so the upstream clock is IPv4-only.
  old single-app table (`GET /firmware` says `"ota": false`). Flash once over
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

A stratum-1 reference of its own: GPS over UART for time-of-day, the PPS line
for the sub-microsecond edge, both disciplining the system clock instead of
stepping it once from an upstream server. The responder is already shaped for
it — a second `TimeRef` returning `stratum = 1` and `refid = "GPS"`, the leap
indicator driven by the fix instead of by the freshness window, and `serveNtp`
deciding whether the LAN gets it. The parts that are genuinely new work:

- **A discipline loop**, not a step: a servo that slews the clock from PPS
  instead of jumping it, so a client never sees the jump.
- **The receive-side accuracy** the dispersion above admits to: the P4's EMAC
  carries an IEEE 1588 timestamp unit (`esp_eth` mentions it; check what IDF
  6.0 actually exposes before betting the design on it), which would replace
  the post-`recvfrom()` stamp with a hardware one.
- **Leap-second and holdover behaviour** worth stating out loud before the
  server calls itself stratum 1.

Until then this is a correct stratum-3 server, and honest about being one.
