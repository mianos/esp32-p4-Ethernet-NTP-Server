# esp32p4-ntp

Step 1 of an NTP server on the ESP32-P4: **Ethernet bring-up + DHCP + ping**,
plus the device's HTTP surface (`/healthz`, `/config`, `/config/reset`) using the
same REST conventions as the other [mianesp](https://github.com/mianos/mianesp)
devices.

Target board: **Waveshare ESP32-P4-ETH** (ESP32-P4NRW32, 32 MB flash,
IP101GRI 10/100 PHY, RJ45). This is what is currently on
`/dev/cu.usbmodem5CF71255951` (chip reported by esptool: ESP32-P4 rev v3.2).

The P4 has an on-chip 10/100 Ethernet MAC but **no PHY**, so the board's
IP101GRI is driven over RMII. Pins are hard-coded in `main/main.c`:

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
but if you ever swap boards, the `ETH_*` defines at the top of `main/main.c`
are the only thing to change.

## Status: working

Verified on this unit (2026-11-11): PHY `0x02430c54` at MDIO addr 1, link up,
DHCP lease `131.84.1.107/24` gw `131.84.1.2`, `ping` from the Mac 4/4 replies
at ~0.6 ms. HTTP endpoints answered on the same lease; `/config` survives a
reboot (NVS), and `/config/reset` restores the compiled-in defaults.

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

## Expected log (captured from this board)

```
I (336) eth_demo: esp32p4-ntp: RMII + IP101, DHCP client, ICMP echo, HTTP /healthz
I (346) settings: no stored config, using defaults
I (346) settings: hostname=esp32p4-ntp tz=AEST-10AEDT,M10.1.0,M4.1.0/3
I (886) eth_demo: MDIO addr 1: PHY ID 0x02430c54 (IP101 = 0x0243xxxx)
I (886) eth_demo: phy: bmsr=0x7849 link=0 autoneg_done=0 bmcr=0x3100
I (886) esp_eth.netif.netif_glue: 80:f1:b2:db:e2:28
I (886) esp_eth.netif.netif_glue: ethernet attached to netif
I (2796) eth_demo: Ethernet started
I (2796) eth_demo: link up   (MAC 80:f1:b2:db:e2:28)
I (2796) eth_demo: requesting DHCP lease ...
I (7296) esp_netif_handlers: eth ip: 131.84.1.107, mask: 255.255.255.0, gw: 131.84.1.2
I (7296) eth_demo: DHCP lease: ip 131.84.1.107  mask 255.255.255.0  gw 131.84.1.2
I (7296) eth_demo: ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP
I (7306) webserver: starting http server on port 80
I (7306) eth_demo: http: http://131.84.1.107/healthz  http://131.84.1.107/config
```

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
| GET    | `/healthz`      | uptime, clock, build, heap, link, address        |
| GET    | `/config`       | current settings                                 |
| POST   | `/config`       | apply any subset of the keys, persist, echo back |
| POST   | `/config/reset` | restore compiled-in defaults and persist         |

`/healthz` on this unit:

```console
$ curl -s http://131.84.1.107/healthz
{"uptime":35,"time":"1970-01-01T11:00:35+1100","time_set":false,
 "version":"ce36551","idf_ver":"v6.0.1","partition":"factory",
 "heap_free":555124,"hostname":"esp32p4-ntp","link_up":true,
 "ip":"131.84.1.107"}
```

`time_set` matters: there is no RTC and no clock source yet, so until an
upstream NTP server or GPS/PPS disciplines the clock the base `time` field is
1970 — `time_set` says so rather than presenting it as a fact. `ip` is omitted
until the DHCP lease lands.

`/config` takes and returns a flat JSON object. Values are strings; keys that
are not registered are ignored, so an older/newer client will not break it:

| Key        | Default                          | Effect |
|------------|----------------------------------|--------|
| `hostname` | `esp32p4-ntp`                    | netif hostname, and the name the DHCP client sends as option 12. A change reaches the lease at the next renewal. |
| `tz`       | `AEST-10AEDT,M10.1.0,M4.1.0/3`   | POSIX TZ used by `localtime()` for `/healthz`. Presentation only — NTP always serves UTC. |

```console
$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"hostname":"p4-ntp","tz":"UTC0"}' http://131.84.1.107/config
{"hostname":"p4-ntp","tz":"UTC0"}

$ curl -s http://131.84.1.107/healthz
{"uptime":44,"time":"1970-01-01T00:00:44+0000","time_set":false, ... ,
 "hostname":"p4-ntp","link_up":true,"ip":"131.84.1.107"}

$ curl -s -X POST http://131.84.1.107/config/reset
{"status":"reset"}
```

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
```

Body size is capped at 4 KB, and string values are capped at 63 characters
(`SETTINGS_STR_LEN`); longer values are truncated and logged.

Settings live as one JSON blob in NVS (namespace `settings`, key `config`),
which is why they survive a reboot. The layout is the C counterpart of
mianesp's `SettingsBase`: fields are registered with their compiled-in defaults
in `settings_init()`, so adding the NTP settings later is one line each.

`cJSON` is no longer part of the IDF core, so `main/idf_component.yml` pulls
`espressif/cjson` from the component registry; the first build downloads it
into `managed_components/` (git-ignored).

## Layout

| File                | Contents                                                        |
|---------------------|-----------------------------------------------------------------|
| `main/main.c`       | EMAC + PHY bring-up, event handlers, boot order, web server start |
| `main/settings.[ch]`| NVS-backed settings (C port of mianesp `SettingsBase`)           |
| `main/web_server.[ch]` | `/healthz`, `/config`, `/config/reset`, JSON error handling    |
| `main/net_state.h`  | read-only netif / link accessors, implemented in `main.c`        |

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
- **First build fails fetching `espressif/cjson`**: the component registry is
  unreachable. Build once with network access; after that
  `managed_components/` is cached locally.

## Next step

The NTP responder (UDP/123) on top of this netif, with the clock disciplined
from an upstream NTP server or GPS/PPS; its settings slot into
`settings_init()` next to `hostname` and `tz`, and `/healthz` already reports
whether the clock has been set.
