# Plan: move the box onto mega's spare port

Written 2026-11-20, on `b29f645`. The reviewer's dispersion work is finished; this
is the opposite trade — take the server off a shared switch and put it on a wire
with exactly one peer.

## 0. What is actually on the host

| | |
|---|---|
| `enp90s0` | I226-LM (`8086:125b`, the vPro/AMT SKU), MAC `58:47:ca:7b:62:1b`, admin **DOWN**, no address, in no bridge, claimed by no container |
| `enp57s0` | I226-V (`8086:125c`), bound to `igc`, **no netdev at all** — `/sys/bus/pci/devices/0000:57:00.0/net/` is empty, so `enp89s0` (the name `/etc/network/interfaces` still hotplugs) does not exist |
| `br0` | `enp2s0f1` (X710) at `131.84.1.1/24`; `enp2s0f0` is the other X710, down |
| ME | HECI `00:16.0` present, `/dev/mei0`, firmware 16.1.25.2049. The LM port is the one it can own |
| tools | no `ethtool`, no `dmesg` for uid 1000, no syslog, no passwordless sudo; `incus-admin` gives full API access so bridge work needs no sudo, but `ip link` does |
| box | MAC `80:f1:b2:db:e2:28`, `131.84.1.107` by DHCP, SLAAC v6, logs to `131.84.1.95:5514`, `timeSource auto`, `serveNtp on` |

Both candidate ports are onboard and spare. `enp90s0` is the only one with a netdev,
so it is the one to try, and it is also the one the ME can interfere with.

## 1. Topology

```
box 10.99.0.2/28 ──── enp90s0 ──── ntp0 10.99.0.1/28 (bridge, NAT) ──── mega
```

- static on both ends, no DHCP server on the wire, no gateway guessing;
- mega is the only host with a foot on the link; everyone else uses one published
  endpoint, so the box never needs a LAN address again;
- IPv6 on the link is dropped (`ipv6.address=none`); clients use v4;
- the box keeps internet through NAT, so `logHost` and the `auto` upstream survive.

A dedicated /28 rather than a /30 keeps room for a second device on the link later
without renumbering.

## 2. Phase 0 — prove the port, before writing any firmware

This phase needs no firmware change, which is exactly why it goes first.

```
incus network create ntp0 --type=bridge ipv4.address=10.99.0.1/28 \
      ipv4.nat=true ipv6.address=none dns.mode=none \
      bridge.external_interfaces=enp90s0
incus network set ntp0 raw.dnsmasq 'dhcp-host=80:f1:b2:db:e2:28,10.99.0.2'
ip -br link show enp90s0        # want UP and <...LOWER_UP> with a cable in
```

Then plug the box into `enp90s0`. It is still a DHCP client, so it takes
`10.99.0.2` from the reservation and answers on it. That is the whole test: if
`/healthz` comes back from mega, the port, the PHY, the autoneg (IP101 is
100 Mb only), the ME and the bridge are all fine, and Phase 2 is a convenience
rather than a rescue.

If `enp90s0` never shows carrier:

1. try to get `enp89s0` back: `echo 0000:57:00.0 | sudo tee /sys/bus/pci/drivers/igc/bind`,
   then `ip -br link` — a netdev appearing means the port works and the earlier
   absence was a probe failure, not a BIOS decision;
2. if neither port links, it is the platform, not the box: MEBx (`Ctrl+P`) for the
   ME/AMT port, or the BIOS setup for both. That is the reboot the last attempt
   needed, and it should be scheduled, not discovered mid-move.

## 3. Phase 1 — keep it reachable before the box moves

The box loses `131.84.1.107` on the day. Nothing should still be pointed at it.

| consumer | today | after |
|---|---|---|
| viewer `ntp-status` | polls `131.84.1.107` | `incus config device add ntp-status box proxy listen=tcp:10.99.0.1:8081 connect=tcp:10.99.0.2:80` |
| OTA / flashing | `POST http://131.84.1.107/firmware` from `esp-builder` | same URL on `10.99.0.1:8081` |
| LAN time service | clients at `131.84.1.107` | `chronyd` on mega, upstream `10.99.0.2`, `allow 131.84.1.0/24`, clients moved to `131.84.1.1` |
| log sink | `131.84.1.95:5514` direct | unchanged, through NAT — verify one `txprobe` line lands |

Running chrony on mega is the better answer than a `:123` proxy device: the LAN
gets one stable address, and a stratum-2 with a local filter is what a shared
server should be serving anyway. It also survives the box being off.

## 4. Phase 2 — firmware: static addressing and a serial console

Both exist to remove the DHCP server from the wire and keep the box recoverable
when the network is wrong. The USB cable stays in `esp-builder`'s `/dev/ttyACM0`
whatever happens to the Ethernet, so the console is the rollback.

**Console.** The primary console is UART0, which is wired to nothing reachable;
the USB-Serial/JTAG is only the *secondary*, and `esp_console_new_repl_usb_serial_jtag()`
is compiled out under that config. So: make USB-SJT the primary
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, secondary NONE), add `esp_console`, and
start a REPL at init. Logs and the prompt then share the one port the box actually
has, which is what `idf_monitor` already assumes.

**Settings** (`settings.h`, validated in `applyJson`, defaults that never brick):

| key | default | rule |
|---|---|---|
| `ipMode` | `dhcp` | `dhcp` or `static`; anything else is `dhcp` |
| `ipAddress` | — | `a.b.c.d/len`, `inet_pton`, prefix 1–30 |
| `ipGateway` | — | dotted quad, optional, on-link not enforced |
| `ipDns` | — | dotted quad, optional |

A `static` mode whose address does not parse falls back to DHCP with a loud log,
so a half-written config is a slow box, never a dark one.

**`eth_link.cpp`.** Parse once; if static, `esp_netif_dhcpc_stop()` and
`esp_netif_set_ip_info()` before `esp_eth_start()`, plus `esp_netif_set_dns_info()`
when set. The netif's default `ETHERNET_EVENT_CONNECTED` action starts the DHCP
client again, so the CONNECTED handler re-asserts: stop it, re-check the address,
then call the address-up path. `onGotIp()` splits into `onAddressUp(ip_info)` so
both paths start HTTP, NTP and `net_log` identically. Both servers bind
`INADDR_ANY`, so a live re-address needs no restart.

**Console command.**

```
net                       # show mode, address, gateway, dns, link
net dhcp                  # persist ipMode=dhcp, stop dhcpc, re-request
net static <cidr> <gw> [dns]
```

Persists through `Settings::save()`, applies live, prints the resulting address.

**HTTP.** `ipMode`/`ipAddress`/`ipGateway`/`ipDns` join `POST /config`, with an
`EthernetLink::applySettings()` next to the existing hostname hook. `/healthz`
gains `ip_mode` so a static box says so.

Sequence: build, flash over the LAN, commit, **rebuild and reflash so the version
is never `-dirty`** — all while the box is still on the switch.

## 5. Phase 3 — the move

1. `net static 10.99.0.2/28 10.99.0.1` over serial, confirm from mega:
   `curl http://10.99.0.2/healthz`, `ping`, and that `/healthz` reports
   `ip_mode static` on the address above.
2. Unplug from the switch, plug into `enp90s0`.
3. Confirm from mega: link up at 100 Mb, `stratum 1`, `ntp_synced true`, and a
   `servepath`/`replypath` line from mega's own `sntp`.
4. From the Mac: viewer over `10.99.0.1:8081`, `sntp` against `131.84.1.1`.
5. Delete the `131.84.1.107` reservation on the router so nobody inherits it.

## 6. Rollback

`net dhcp` on the console, unplug, plug back into the switch. Two minutes, no
reflash, no reboot. The static config persists but is inert.

If the console is unreachable: `esptool`/`idf.py flash` over `/dev/ttyACM0` still
works, because the USB path never depended on the network.

## 7. Risks

| risk | severity | answer |
|---|---|---|
| ME owns the LM port and drops the link when provisioned | high | Phase 0 catches it before any firmware work; MEBx is the fix |
| `enp89s0` has no netdev, so the fallback port may not exist either | medium | rebind first; if it is a BIOS disable, only the BIOS fixes it |
| IP101 is 100 Mb only | low | fine for NTP; the I226 autonegotiates down |
| box unreachable after a bad static config | medium | console + `net dhcp` + the invalid-config fallback |
| viewer and OTA silently break | medium | Phase 1 lands before the cut, and is tested from the Mac |
| LAN time clients left on `.107` | medium | clients move to `131.84.1.1` in Phase 1, not after |
| a second `dnsmasq` on the host fighting port 53 | low | `dns.mode=none` |

## 8. Open questions

1. `enp90s0` or `enp89s0` — which port is the one that was tried, and is it the one
   with a cable run?
2. Is a cable to that port in place, and may the box drop off the LAN for the
   Phase 0 test?
3. LAN time: chrony relay on mega as above, or keep clients pointed at a `:123`
   proxy device?
