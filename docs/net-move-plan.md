# Plan: move the box onto `enp90s0`

Written 2026-11-20, on `b29f645`; retargeted after the SFP+ cage turned out to be
empty. The reviewer's dispersion work is finished; this is the opposite trade —
take the server off a shared switch and put it on a wire with exactly one peer.

## 0. What is actually there

| | |
|---|---|
| `enp90s0` | **the target**: Intel I226-LM (`8086:125b`), the onboard RJ45 behind PCH root port `00:1d.2`, MAC `58:47:ca:7b:62:1b`, admin DOWN, spare, no bridge, claimed by no container |
| | it is the **LM / AMT SKU**, so the ME (`/dev/mei0` on HECI `00:16.0`, FW 16.1.25.2049) can own its link. It is native 10/100/1000, so nothing has to be bought |
| `enp2s0f0` | the other spare: X710 port 0, an **empty SFP+ cage** on the card whose port 1 carries the LAN. Held in reserve; needs a 1000BASE-T module |
| `enp2s0f1` | X710 port 1, carries `br0` at `131.84.1.1/24`, linked at **10 Gb/s** — the machine has no spare 1G module to borrow |
| `enp89s0` | I226-V onboard, bound to `igc`, **no netdev at all** — the name `/etc/network/interfaces` hotplugs does not exist |
| router | `hfc` (container), `131.84.1.2`, masquerade is `ip saddr 131.84.0.0/16`, `DEFAULT_FORWARD_POLICY=ACCEPT`, ruleset live-only (`nftables.service` disabled) |
| **leftover from the last attempt** | `hfc` still carries `10.99.0.0/29 via 131.84.1.1`. Nothing on `mega` answers it: no interface, no bridge, no dnsmasq, no container |
| tools | no `ethtool`, no `dmesg` for uid 1000, no syslog, no passwordless sudo; `incus-admin` covers the bridge, `ip link` does not |
| box | MAC `80:f1:b2:db:e2:28`, `131.84.1.107` by DHCP, SLAAC v6, logs to `131.84.1.95:5514`, `timeSource auto`, `serveNtp on`, PHY is 10/100 only |

The block is already chosen by that leftover route: **`10.99.0.0/29`**, mega `.1`,
box `.2`. Reuse it rather than inventing one.

Why not the cage: `enp2s0f0` has no module in it, and a 10G one would not
negotiate with a 100 Mb PHY anyway. The RJ45 links at 100 M by itself, today, for
nothing. The cost is the ME, and the ME is only a problem if AMT is provisioned —
it plainly is not, or the port would already be carrying something.

## 1. Topology

```
box 10.99.0.2/29 ──── enp90s0 ──── ntp0 10.99.0.1/29 (bridge, no DHCP) ──── mega
                                                                             │
            131.84.1.0/24 ──── hfc ── route 10.99.0.0/29 via 131.84.1.1 ────┘
```

- static on both ends, no DHCP server, no gateway guessing;
- routed, not proxied: anyone on the LAN reaches `10.99.0.2` directly, so the
  viewer and OTA keep their own source addresses and nothing is DNAT'd;
- mega NATs the box's outbound at `hfc`, so `logHost` and the `auto` upstream keep
  working even if the receiver drops;
- IPv6 on the link is dropped; clients use v4. The box loses its SLAAC address, so
  anything reaching it over IPv6 has to move.

## 2. Phase 0 — prove the port, before any firmware work

Needs a Cat5e lead to the box and nothing else. The box stays a DHCP client, so
this phase changes no firmware.

```
incus network create ntp0 --type=bridge ipv4.address=10.99.0.1/29 \
      ipv4.dhcp=true ipv6.address=none dns.mode=none \
      bridge.external_interfaces=enp90s0
incus network set ntp0 raw.dnsmasq 'dhcp-host=80:f1:b2:db:e2:28,10.99.0.2'
ip -br link show enp90s0        # want UP and <...LOWER_UP> once a cable is in
cat /sys/class/net/enp90s0/carrier
```

Plug the box in; it leases `10.99.0.2` from the reservation and answers `/healthz`
from mega. That is the whole test: port, PHY, autoneg down to 100 M, the ME and the
bridge all proven before a line of code moves.

If the port will not come up or drops the link when the ME is awake:

1. `ip link set enp90s0 up` by hand and re-read carrier — it may just be administratively
   down and unattached;
2. if carrier comes and goes with no cable movement, it is the ME: MEBx (`Ctrl+P`)
   to disable or unprovision AMT, which needs the reboot that has been avoided;
3. fall back to the cage once a 1000BASE-T module exists.

## 3. Phase 1 — make the path work before the box moves

| step | command |
|---|---|
| route already exists | `hfc`: `10.99.0.0/29 via 131.84.1.1` — verify, do not re-add |
| masquerade for the box's outbound | `hfc`: `nft add rule ip nat POSTROUTING oifname "hfc" ip saddr 10.99.0.0/29 masquerade` |
| keep the ruleset | `hfc`: `nft list ruleset > /root/nft.$(date +%F)` first — the live set is not in `/etc/nftables.conf` and dies with the container |
| viewer | target `http://10.99.0.2/healthz` |
| OTA | `POST http://10.99.0.2/firmware` from `esp-builder` |
| log sink | `131.84.1.95:5514`, already routed; confirm one `txprobe` line lands |

No proxy devices: with a route, the containers and the Mac all reach the box by its
own address and its replies carry its own source. That keeps the timestamps and the
`servepath` attribution honest.

## 4. Phase 2 — firmware: static addressing and a serial console

Built, flashed and exercised on 2026-11-20; the README section *The serial
console* is the operator-facing half and the source is main/console.cpp.

The console exists to remove the DHCP server from the wire and to keep the box
recoverable when the network is wrong. The USB cable stays in `esp-builder`'s
`/dev/ttyACM0` whatever happens to the Ethernet, so the console is the rollback.

The reference is IDF's own console component and its two examples, both in the
tree on `esp-builder`: `examples/system/console/basic` for the REPL lifecycle and
`examples/system/console/advanced/components` for the ready-made `cmd_system` and
`cmd_nvs`. Take the structure from those; `cmd_wifi` is pointless here, the box has
no radio.

| take | from |
|---|---|
| why the secondary console must be off | `examples/system/console/basic/main/console_example_main.c:29-30` (`#warning`) |
| transport `#if` chain, P4-viable branch included | same file, `:105-121` |
| `cmd_system` (`restart`, `free`, `tasks`, `uptime`, `version`) | `examples/system/console/advanced/components/cmd_system` |
| `cmd_nvs` (`nvs_list`, `nvs_get`) | `examples/system/console/advanced/components/cmd_nvs` |
| the two FreeRTOS options `tasks` needs | `examples/system/console/advanced/sdkconfig.defaults:14-15` |
| `esp_console_cmd_register()` with `func_w_context` | `components/console/esp_console.h` |
| component dependency | `PRIV_REQUIRES console`, path deps in `main/idf_component.yml` |

**Transport.** Primary console becomes USB-Serial/JTAG with the secondary NONE
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`).
UART0 goes back to being unused, which is what it already is in practice. No
partition change: history needs a `fat` partition and the box is on
`CONFIG_PARTITION_TABLE_TWO_OTA_LARGE`, so `CONFIG_CONSOLE_STORE_HISTORY` stays off
until someone actually misses it — a table change cannot be delivered by OTA.

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
when set. The netif's default `ETHERNET_EVENT_CONNECTED` action starts the DHCP client
again, so the CONNECTED handler re-asserts: stop it, re-check the address, then call
the address-up path. `onGotIp()` splits into `onAddressUp(ip_info)` so both paths
start HTTP, NTP and `net_log` identically. Both servers bind `INADDR_ANY`, so a live
re-address needs no restart.

**Commands.** One generic path plus the shorthands worth typing:

```
net                                   # mode, address, gateway, dns, link state
net dhcp                              # persist ipMode=dhcp, stop dhcpc, re-request
net static <cidr> <gw> [dns]          # validate, persist, apply live
config                                # the settings JSON
config set <key> <value>              # any registered key, through applyJson
config reset                          # defaults
hostname <name> | serve <on|off> | timesource <off|dhcp|static|gps|auto>
ntpserver <host> | log <ip> <port> | loglevel <x>
version | free | uptime | tasks | restart
nvs_list | nvs_get                     # from cmd_nvs
```

Everything routes through `Settings::applyJson()` + the existing onChange hooks, so
the console cannot invent a state the HTTP surface does not also validate. `restart`
uses `cmd_system`; it is the one command that costs the box a reboot, so it is worth
having and worth not automating.

**REPL config.** `ESP_CONSOLE_REPL_CONFIG_DEFAULT()`, then prompt `ntp>`,
`max_cmdline_length` 512 and `task_stack_size` 6144 — the 4096 default is thin for
stdio plus argtable. Start it from `app_main` right
after `Settings::begin()` and *before* `EthernetLink::begin()`, so the console is up
even if the Ethernet init hangs. Failure to create the REPL is logged, not fatal —
a box with no console must still boot.

**HTTP.** `ipMode`/`ipAddress`/`ipGateway`/`ipDns` join `POST /config`, with an
`EthernetLink::applySettings()` beside the existing hostname hook. `/healthz` gains
`ip_mode`.

**Interleaving.** Logs and the prompt share one port, so raw `cat` garbles and
`idf_monitor` is the interactive tool. Capture stays on `net_log`'s UDP mirror, which
is unaffected.

**Trust.** USB is the only door: no auth on the console, and `config` prints the
whole blob. That is fine while nothing secret is in it; it is a reason not to put a
secret there without adding a redaction.

Sequence: build, flash over the LAN, commit, **rebuild and reflash so the version is
never `-dirty`** — all while the box is still on the switch.

## 5. Phase 3 — the move

1. `net static 10.99.0.2/29 10.99.0.1` over serial; confirm from mega that
   `curl http://10.99.0.2/healthz` reports `ip_mode static` on that address.
2. Unplug from the switch, plug into `enp90s0`.
3. Confirm from mega: carrier up at 100 M, `stratum 1`, `ntp_synced true`, and a
   `servepath`/`replypath` line from mega's own `sntp`.
4. From a LAN container and the Mac: `/healthz` over the routed path, `sntp` against
   `10.99.0.2` within its bound.
5. Delete the `131.84.1.107` reservation on the router so nobody inherits it.

## 6. Rollback

`net dhcp` on the console, unplug, plug back into the switch. Two minutes, no reflash,
no reboot. The static config persists but is inert.

Worst case the console is unreachable too: `idf.py flash` over `/dev/ttyACM0` still
works, because the USB path never depended on the network.

## 7. Risks

| risk | severity | answer |
|---|---|---|
| ME drops or owns the link | medium | Phase 0 catches it with no code deployed; MEBx disables AMT if needed |
| box unreachable after a bad static config | medium | console, `net dhcp`, and the invalid-config fallback |
| `hfc` ruleset is live-only and dies with the container | medium | snapshot it first, restore on the way back |
| viewer and OTA silently break | medium | Phase 1 lands before the cut and is tested from the Mac |
| LAN time clients left on `.107` | medium | they move to `10.99.0.2` in Phase 1, not after |
| console shares the port with the logs | low | `idf_monitor` for interaction, `net_log` for capture |
| console runtime and `cmd_system` cost flash and stack | low | ~40 KB and 6 KB, against 1700 K slots and 8 KB of main stack |
| a console that can change anything is a physical-access backdoor | low | no secrets in settings; `config` prints the blob |
| AMT shares the port and its MAC | low | unprovisioned AMT is inert; note it if the ME is ever enabled |
| IP101 is 100 Mb, the port is 1G | low | autoneg settles at 100; throughput is irrelevant to NTP |

## 8. Open questions

1. Is there a Cat5e lead from a motherboard RJ45 to the box, and may the box drop off
   the LAN for the Phase 0 test?
