# esp32p4-ntp

Step 1 of an NTP server on the ESP32-P4: **Ethernet bring-up + DHCP + ping**.

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
at ~0.6 ms.

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

## Expected log (captured from this board)

```
I (318) eth_demo: esp32p4-ntp eth demo: RMII + IP101, DHCP client, ICMP echo
I (848) eth_demo: MDIO addr 1: PHY ID 0x02430c54 (IP101 = 0x0243xxxx)
I (848) eth_demo: phy: bmsr=0x7849 link=0 autoneg_done=0 bmcr=0x3100
I (848) esp_eth.netif.netif_glue: 80:f1:b2:db:e2:28
I (848) esp_eth.netif.netif_glue: ethernet attached to netif
I (2858) eth_demo: Ethernet started
I (2858) eth_demo: link up   (MAC 80:f1:b2:db:e2:28)
I (2858) eth_demo: requesting DHCP lease ...
I (7358) esp_netif_handlers: eth ip: 131.84.1.107, mask: 255.255.255.0, gw: 131.84.1.2
I (7358) eth_demo: DHCP lease: ip 131.84.1.107  mask 255.255.255.0  gw 131.84.1.2
```

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

## Next step

Add the NTP responder (UDP/123, SNTP-style single-packet reply) on top of
this netif, optionally disciplining the system clock from an upstream NTP
server or GPS/PPS.
