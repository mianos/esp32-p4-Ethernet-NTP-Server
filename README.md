# esp32p4-ntp

Step 1 of an NTP server on the ESP32-P4: **Ethernet bring-up + DHCP + ping**,
plus the device's HTTP surface (`/healthz`, `/config`, `/config/reset`,
`/ntp/sync`) using the same REST conventions as the other
[mianesp](https://github.com/mianos/mianesp) devices, plus a test-mode clock
that steps itself from the upstream NTP server the DHCP server hands out
(option 42). All of it is C++, in the same shape as mianesp's components.

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

## Build / flash

```sh
cd esp32p4-ntp
./build.sh flash      # build + flash
./build.sh monitor    # build + flash + serial log (Ctrl-] to quit)
```

`build.sh` uses the ESP-IDF v6.0.1 install at
`/Users/robertfowler/.espressif/v6.0.1/esp-idf` and the port
`/dev/cu.usbmodem5CF71255951` (override with `PORT=...`). Use the `/dev/cu.*`
node on macOS: `/dev/tty.*` intermittently reports `EBUSY` here.

The lwIP SNTP/DHCP options live in `sdkconfig.defaults`, which only seeds a
*fresh* `sdkconfig`; an existing one keeps its old values, so after pulling a
change like this run `rm sdkconfig` (or `idf.py menuconfig`) before building.

Console output goes to **UART0**, which on this board is wired to the USB-C
port's CH343 USB-UART bridge (VID `0x1A86`, PID `0x55D3`, enumerates as
`/dev/tty.usbmodem*`), i.e. the same cable used for flashing. The P4's
native USB-Serial/JTAG is not on that connector, which is why
`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` yields a completely silent log.

## What it does

1. `esp_eth_mac_new_esp32()` + RMII pin map → EMAC up.
2. `esp_eth_phy_new_generic()` → standard IEEE 802.3 PHY driver (IP101
   autonegotiates 10/100, full/half duplex).
3. `ESP_NETIF_DEFAULT_ETH()` → DHCP client starts automatically.
4. lwIP answers **ICMP echo** once the netif has an address, so the board is
   pingable with no application code.
5. Diagnostics kept on purpose: at boot it scans the 32 MDIO addresses and
   prints the PHY ID, and while there is no IP it prints `BMSR` every 5 s
   (`link` / `autoneg_done`). If MDIO is miswired you get
   `phy: MDIO read failed` instead — a one-line answer to "are my pins right?".
6. Starts an HTTP server on port 80 with an address on the first DHCP lease
   (see [HTTP API](#http-api)).
7. Runs an SNTP **client** (`NtpTime`) that steps the clock once the lease
   arrives — from the server DHCP option 42 named, or from one configured by
   hand — so the NTP responder has real time to serve. See
   [Clock (test mode)](#clock-test-mode).

The code is C++ (`.cpp` + classes), matching mianesp's components:
`Settings`, `WebServer` and `NtpTime` are singletons reached through
`instance()`, with httpd's C callbacks as static members.

## Expected log (captured from this board)

```
I (339) eth_demo: esp32p4-ntp: RMII + IP101, DHCP client, HTTP /healthz, upstream clock
I (349) settings: no stored config, using defaults
I (349) settings: hostname=esp32p4-ntp tz=AEST-10AEDT,M10.1.0,M4.1.0/3 timeSource=dhcp ntpServer=(unset)
I (349) ntp_time: time source: dhcp (upstream from option 42, once the lease lands)
I (899) eth_demo: MDIO addr 1: PHY ID 0x02430c54 (IP101 = 0x0243xxxx)
I (899) eth_demo: phy: bmsr=0x7849 link=0 autoneg_done=0 bmcr=0x3100
I (899) esp_eth.netif.netif_glue: 80:f1:b2:db:e2:28
I (899) esp_eth.netif.netif_glue: ethernet attached to netif
I (2909) eth_demo: Ethernet started
I (2909) eth_demo: link up   (MAC 80:f1:b2:db:e2:28)
I (2909) eth_demo: requesting DHCP lease ...
I (7409) esp_netif_handlers: eth ip: 131.84.1.107, mask: 255.255.255.0, gw: 131.84.1.2
I (7409) eth_demo: DHCP lease: ip 131.84.1.107  mask 255.255.255.0  gw 131.84.1.2
I (7409) eth_demo: ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP
I (7419) ntp_time: upstream time server: 192.0.2.1
I (7419) webserver: starting http server on port 80
I (7429) eth_demo: http: http://131.84.1.107/healthz  http://131.84.1.107/config
I (8129) ntp_time: clock set from 192.0.2.1: 2026-11-11T13:38:15+1100
```

The upstream address is printed as `192.0.2.1` (TEST-NET-1) here rather than the
address this LAN's DHCP server actually hands out.

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

`/healthz` on this unit:

```console
$ curl -s http://131.84.1.107/healthz
{"uptime":30,"time":"2026-11-11T13:38:38+1100","time_set":true,
 "version":"d235d34","idf_ver":"v6.0.1","partition":"factory",
 "heap_free":552788,"hostname":"esp32p4-ntp","link_up":true,
 "ip":"131.84.1.107","ntp_source":"dhcp","ntp_server":"192.0.2.1",
 "ntp_reachable":true,"ntp_synced":true,"last_sync":1794364695}
```

`time_set` matters: there is no RTC, so the base `time` field is 1970 + uptime
until something disciplines it. `time_set` says whether the clock is plausible
(after 2020) rather than presenting 1970 as a fact, `ntp_synced` says whether
*this* client set it, `last_sync` is the epoch it last set, `ntp_server` is the
upstream actually in use (`null` until one is known) and `ntp_reachable` is the
RFC 5905 reachability register: non-zero means the server has answered. `ip` is
omitted until the DHCP lease lands.

`/config` takes and returns a flat JSON object. Values are strings; keys that
are not registered are ignored, so an older/newer client will not break it:

| Key        | Default                          | Effect |
|------------|----------------------------------|--------|
| `hostname` | `esp32p4-ntp`                    | netif hostname, and the name the DHCP client sends as option 12. A change reaches the lease at the next renewal. |
| `tz`       | `AEST-10AEDT,M10.1.0,M4.1.0/3`   | POSIX TZ used by `localtime()` for `/healthz`. Presentation only — NTP always serves UTC. |
| `timeSource` | `dhcp`                         | `dhcp` takes the upstream NTP server from DHCP option 42, `static` polls `ntpServer`, `off` leaves the clock alone. Unknown values are treated as `off` and logged. |
| `ntpServer` | *(empty)*                       | Upstream NTP host or address, used only when `timeSource` is `static`. |

```console
$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"hostname":"p4-ntp","tz":"UTC0"}' http://131.84.1.107/config
{"hostname":"p4-ntp","tz":"UTC0","timeSource":"dhcp","ntpServer":""}

$ curl -s http://131.84.1.107/healthz
{"uptime":44,"time":"2026-11-11T02:38:44+0000","time_set":true, ... ,
 "hostname":"p4-ntp","link_up":true,"ip":"131.84.1.107", ...}

$ curl -s -X POST http://131.84.1.107/config/reset
{"status":"reset"}
```

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
which is why they survive a reboot. The layout is the C++ counterpart of
mianesp's `SettingsBase`: fields are registered with their compiled-in defaults
in the `Settings` constructor, so a new setting is one `add()` line plus its
member in `Settings::Values`.

`cJSON` is no longer part of the IDF core, so `main/idf_component.yml` pulls
`espressif/cjson` from the component registry; the first build downloads it
into `managed_components/` (git-ignored).

## Layout

| File                    | Contents                                                        |
|-------------------------|-----------------------------------------------------------------|
| `main/main.cpp`         | EMAC + PHY bring-up, event handlers, boot order, web server start |
| `main/settings.[ch]pp`  | `Settings`: NVS-backed settings (mianesp `SettingsBase` shape)   |
| `main/web_server.[ch]pp`| `WebServer`: `/healthz`, `/config`, `/config/reset`, `/ntp/sync` |
| `main/ntp_time.[ch]pp`  | `NtpTime`: upstream SNTP client for the test-mode clock          |
| `main/net_state.[ch]pp` | read-only netif / link accessors, fed by `main.cpp`              |
| `main/json.h`           | `json::Value`: cJSON behind a `unique_ptr`, so nothing hand-frees |

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
  `IP_EVENT_ETH_GOT_IP`, so check the log for `webserver: starting http server`.
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
- **First build fails fetching `espressif/cjson`**: the component registry is
  unreachable. Build once with network access; after that
  `managed_components/` is cached locally.

## Next step

The NTP responder (UDP/123) on top of this netif, serving the clock the test
mode just set — or, better, replacing it with real discipline (GPS/PPS) so the
responder has a stable reference of its own. Its settings slot into the
`Settings` constructor next to `hostname`, `timeSource` and `ntpServer`, and
`/healthz` already reports whether the clock is set, from where, and when.
