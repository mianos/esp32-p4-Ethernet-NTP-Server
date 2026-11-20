# Plan: move the box onto `enp2s0f0`

Written 2026-11-20, on `b29f645`, revised after the port was named. The reviewer's
dispersion work is finished; this is the opposite trade — take the server off a
shared switch and put it on a wire with exactly one peer.

## 0. What is actually there

| | |
|---|---|
| `enp2s0f0` | **the named spare port**: Intel X710 port 0, `8086:1572`, `i40e`, MAC `58:47:ca:7b:62:18`, `portname p0`, alias `enp2s0f0np0` |
| | an **SFP+ cage**, not an RJ45. **Confirmed empty** 2026-11-20: brought up in a scratch container, `Supported ports: [ ]`, `Port: Other`, `ethtool -m` refused, carrier 0 |
| `enp2s0f1` | X710 port 1, carries `br0` at `131.84.1.1/24`, linked at **10 Gb/s** — so the machine has no spare 1G module to borrow |
| `enp90s0` | I226-LM onboard RJ45, admin DOWN, spare; the AMT SKU, and the ME (`/dev/mei0`, FW 16.1.25.2049) can own its link |
| `enp89s0` | I226-V onboard, bound to `igc`, **no netdev at all** (`/sys/.../0000:57:00.0/net/` empty), so the name `/etc/network/interfaces` hotplugs does not exist |
| router | `hfc` (container), `131.84.1.2`, masquerade is `ip saddr 131.84.0.0/16`, `DEFAULT_FORWARD_POLICY=ACCEPT`, ruleset is live-only (`nftables.service` disabled) |
| **leftovers from the last attempt** | `hfc` still has `10.99.0.0/29 via 131.84.1.1 dev eth0`. Nothing on `mega` answers it: no interface, no bridge, no dnsmasq, no container |
| tools | no `ethtool`, no `dmesg` for uid 1000, no syslog, no passwordless sudo; `incus-admin` covers the bridge, `ip link` does not |
| box | MAC `80:f1:b2:db:e2:28`, `131.84.1.107` by DHCP, SLAAC v6, logs to `131.84.1.95:5514`, `timeSource auto`, `serveNtp on`, PHY is 10/100 only |

So the last attempt got as far as a route on the router and no further. The block
is already chosen: **`10.99.0.0/29`**, mega `.1`, box `.2`. Reuse it.

### The thing that probably broke it

`enp2s0f0` is a 10 GbE cage and the box is a 100 Mb PHY. A 10G optic or a DAC in
that cage cannot negotiate with IP101 at all; only a **1000BASE-T SFP** can, and
even then it is the module's copper PHY that autonegotiates down to 100 M while
`i40e` sees a 1G link. Without a 1000BASE-T module in the cage there is nothing to
debug and the BIOS/ME story does not apply — the X710 is not the ME port.

## 1. Topology

```
box 10.99.0.2/29 ──── enp2s0f0 ──── ntp0 10.99.0.1/29 (bridge, no DHCP) ──── mega
                                                                             │
            131.84.1.0/24 ──── hfc ── route 10.99.0.0/29 via 131.84.1.1 ────┘
```

- static on both ends, no DHCP server, no gateway guessing;
- routed, not proxied: anyone on the LAN reaches `10.99.0.2` directly, so the
  viewer and OTA keep their own source addresses and nothing is DNAT'd;
- mega NATs the box's outbound at `hfc` so `logHost` and the `auto` upstream still
  work;
- IPv6 on the link is dropped; clients use v4. The box loses its SLAAC address, so
  anything reaching it over IPv6 has to move.

## 2. Phase 0 — prove the port, before any firmware work

Needs a 1000BASE-T SFP in `enp2s0f0` and a Cat5e lead to the box. Check the module
is physically there first; if it is a 10G optic or a DAC, stop here.

```
incus network create ntp0 --type=bridge ipv4.address=10.99.0.1/29 \
      ipv4.dhcp=false ipv6.address=none dns.mode=none \
      bridge.external_interfaces=enp2s0f0
incus network set ntp0 raw.dnsmasq 'dhcp-host=80:f1:b2:db:e2:28,10.99.0.2'
ip -br link show enp2s0f0        # want UP and <...LOWER_UP>, not noop
cat /sys/class/net/enp2s0f0/carrier
```

The box is still a DHCP client, so once it is plugged in it takes a lease and
answers `/healthz` from mega with **no firmware change**. That is the whole test.

If the port stays down with a known-good module:

1. `i40e` may have rejected it — `dmesg | grep -i "unsupported sfp"`, then
   `options i40e allow_unsupported_sfp=1` in `/etc/modprobe.d/` and reload the
   module. Reloading `i40e` drops **both** X710 ports, so `br0` blips for a few
   seconds — no reboot, but not while something is mid-transfer;
2. try the RJ45 instead: `ip link set enp90s0 up`, move the bridge's
   `bridge.external_interfaces`, and accept the AMT port's quirks;
3. only if neither links is it the platform, and then MEBx/BIOS is the answer.

## 3. Phase 1 — make the path work before the box moves

| step | command |
|---|---|
| route already exists | `hfc`: `10.99.0.0/29 via 131.84.1.1` — verify, do not re-add |
| masquerade for the box's outbound | `hfc`: `nft add rule ip nat POSTROUTING oifname "hfc" ip saddr 10.99.0.0/29 masquerade` |
| keep the ruleset | `hfc`: `nft list ruleset > /root/nft.$(date +%F)` before the change — the live set is not in `/etc/nftables.conf` and dies with the container |
| viewer | target `http://10.99.0.2/healthz` |
| OTA | `POST http://10.99.0.2/firmware` from `esp-builder` |
| log sink | `131.84.1.95:5514`, already routed; confirm one `txprobe` line lands |

No proxy devices: with a route, the containers and the Mac all reach the box by
its own address, and the box's replies carry its own source. That keeps the
timestamps and the `servepath` attribution honest.

## 4. Phase 2 — firmware: static addressing and a serial console

Both exist to remove the DHCP server from the wire and keep the box recoverable
when the network is wrong. The USB cable stays in `esp-builder`'s `/dev/ttyACM0`
whatever happens to the Ethernet, so the console is the rollback.

**Console.** The primary console is UART0, wired to nothing reachable; USB-Serial/JTAG
is only the *secondary*, and `esp_console_new_repl_usb_serial_jtag()` is compiled
out under that config. So: make USB-SJT the primary
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, secondary NONE), add `esp_console`, start a
REPL at init. Logs and the prompt then share the one port the box has, which is
what `idf_monitor` already assumes.

**Settings** (`settings.h`, validated in `applyJson`, defaults that never brick):

| key | default | rule |
|---|---|---|
| `ipMode` | `dhcp` | `dhcp` or `static`; anything else is `dhcp` |
| `ipAddress` | — | `a.b.c.d/len`, `inet_pton`, prefix 1–30 |
| `ipGateway` | — | dotted quad, optional |
| `ipDns` | — | dotted quad, optional |

A `static` mode whose address does not parse falls back to DHCP with a loud log, so
a half-written config is a slow box, never a dark one.

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
`EthernetLink::applySettings()` beside the existing hostname hook. `/healthz` gains
`ip_mode`.

Sequence: build, flash over the LAN, commit, **rebuild and reflash so the version is
never `-dirty`** — all while the box is still on the switch.

## 5. Phase 3 — the move

1. `net static 10.99.0.2/29 10.99.0.1` over serial; confirm from mega:
   `curl http://10.99.0.2/healthz` reports `ip_mode static` on that address.
2. Unplug from the switch, plug into `enp2s0f0`.
3. Confirm from mega: carrier up, `stratum 1`, `ntp_synced true`, and a
   `servepath`/`replypath` line from mega's own `sntp`.
4. From a LAN container and the Mac: `/healthz` over the routed path, `sntp` against
   `10.99.0.2` within its bound.
5. Delete the `131.84.1.107` reservation on the router so nobody inherits it.

## 6. Rollback

`net dhcp` on the console, unplug, plug back into the switch. Two minutes, no
reflash, no reboot. The static config persists but is inert.

Worst case the console is unreachable too: `idf.py flash` over `/dev/ttyACM0` still
works, because the USB path never depended on the network.

## 7. Risks

| risk | severity | answer |
|---|---|---|
| no 1000BASE-T module in the cage, or a 10G optic | high | check before Phase 0; it is the likeliest reason the last attempt died |
| `i40e` refuses the module | medium | `allow_unsupported_sfp=1`; the reload blips `br0` |
| box unreachable after a bad static config | medium | console, `net dhcp`, and the invalid-config fallback |
| `hfc` ruleset is live-only and dies with the container | medium | snapshot it first; restore on the way back |
| viewer and OTA silently break | medium | Phase 1 lands before the cut and is tested from the Mac |
| LAN time clients left on `.107` | medium | they move to `10.99.0.2` in Phase 1, not after |
| IP101 is 100 Mb, the cage is 10G | low | module autoneg handles it; throughput is irrelevant to NTP |
| `enp90s0` fallback is the AMT port | low | only used if the cage fails; unprovisioned AMT is inert |

## 8. Open questions

1. What is in the `enp2s0f0` cage now — 1000BASE-T SFP, 10G optic, DAC, or nothing?
2. Is there a Cat5e lead from that port to the box, and may the box drop off the
   LAN for the Phase 0 test once the module is in?
