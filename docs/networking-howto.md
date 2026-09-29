# Running firmware with the network

How to give an unmodified firmware image a WiFi connection and let it reach the real network.
What is behind it: [wifi-plan.md](wifi-plan.md) (the MAC model and the virtual AP),
[networking-plan.md](networking-plan.md) (the subnet and the NAT).

## The short version

```sh
esp32sim --board none --boot rom --console usb --max-seconds 20 \
    --bootloader build/bootloader/bootloader.bin \
    --ptable build/partition_table/partition-table.bin \
    --app build/your_app.bin --elf build/your_app.elf \
    --wifi "ssid=esp32sim,psk=esp32sim-pass"
```

`--wifi` attaches the access point *and* the network behind it; there is nothing else to turn on.
The station comes up on **10.0.2.15/24**, gateway **10.0.2.2**, resolver **10.0.2.3**, and outbound
TCP/UDP is relayed to the host's own network (`--net nat`, the default).

The SSID and passphrase must match what the firmware is configured to look for — the emulator's AP
adopts whatever you give it. The AP is WPA2-PSK when `psk=` is present and open when it is not.

| Key | Meaning | Default |
| --- | --- | --- |
| `ssid=NAME` | network name the AP beacons | `esp32sim` |
| `psk=PASS` | WPA2-PSK passphrase; omit for an open network | none (open) |
| `chan=N` | channel in beacons and probe responses | 6 |
| `bssid=xx:xx:..` | AP MAC | `02:53:49:4d:00:01` |

`--net none` keeps the emulated subnet (DHCP, DNS, SNTP still answer) but refuses anything past the
gateway with an immediate RST, so applications fail fast instead of hanging — useful when you want a
run to be reproducible and offline.

`--net bridge:PATH` (macOS and Linux) puts the station on a real LAN instead: a
[socket_vmnet](https://github.com/lima-vm/socket_vmnet) daemon in bridged mode, running as root,
listens at PATH (for example `/var/run/socket_vmnet.bridged.en0`), and esp32sim connects to it as
an ordinary user. The station's Ethernet frames go out as they are and the LAN's router gives it
an address; none of the emulated services answer. The daemon floods every frame to every client,
so only frames for the station's MAC (the source address of what it sends, `--mac` until then),
broadcast and multicast are passed on. A reader thread empties the socket all the time, because a
client that stops reading stalls the daemon for everyone (socket_vmnet issue #173); what the
emulation has not taken in time is dropped. When the daemon goes away (sleep, a Wi-Fi reconnect)
esp32sim reconnects with a growing delay. The counters (`rx_ok`, `rx_filtered`, `rx_dropped`, `tx`,
`tx_dropped`, `reconnects`) are printed at the end of the run. Over Wi-Fi the host kernel rewrites
the station's MAC to the host's own on the air (MAC-NAT), so the LAN's ARP tables show the host's
MAC for the station's address; its own MAC still appears in DHCP.

The same `--wifi` and `--net` work on the **ESP32-C6** (`esp32sim-c6`): the access point and the
network are the same code, the MAC model is the C6's own ([wifi-c6-plan.md](wifi-c6-plan.md)). A C6
radio run also needs `--stub bb_init=0`, and what has been tried is one station on one open or
WPA2 network; `examples/c6-wifi-station` has the full command. The C3 has no WiFi model.

## Reaching the firmware from the Mac

A web UI, an API or a UDP listener inside the guest is reached through host ports forwarded into
it, as QEMU's `hostfwd` does:

```sh
esp32sim ... --wifi "ssid=esp32sim,psk=esp32sim-pass" --realtime \
    --hostfwd tcp:8080-80 --hostfwd tcp:8081-81 --hostfwd udp:4210-4210
curl http://127.0.0.1:8080/
```

`--hostfwd PROTO:HOSTPORT-GUESTPORT` is repeatable; `PROTO` is `tcp` or `udp`. It needs `--wifi`
and the NAT (`--net nat`, the default).

- **Only 127.0.0.1.** The host port is bound on the loopback address, never on `0.0.0.0`, so the
  guest is reachable from this Mac and not from the LAN. There is no switch to widen it.
- **The station is whatever DHCP leased** (10.0.2.15 by default). Until the firmware has taken its
  lease, connections and datagrams wait on the host side (the listen backlog, the socket buffer);
  they go in once it has. Firmware with a static IP and no DHCP is not reached.
- **TCP.** Each accepted connection is opened toward the guest from the gateway, 10.0.2.2, with a
  port from 49152 up, so the firmware sees every client as 10.0.2.2. From the handshake on it is the
  same relay as an outbound connection. A guest port nobody listens on answers with a reset, and the
  host connection is closed at once. The relay closes the host socket rather than resetting it;
  the host kernel turns that close into a reset when the client's request is still unread (RFC 2525
  section 2.17), so curl reports `(56) Recv failure: Connection reset by peer`, and a client that
  sent nothing yet sees end of file. The same happens in the seconds between the lease and the
  firmware starting its server, when a SYN goes unanswered for 30 s of emulated time, and when the
  guest resets a connection later on.
- **Several connections** are fine, up to the NAT's 64 flows shared with outbound traffic; beyond
  that, new ones wait in the backlog until a flow ends. A server that handles one client at a time
  (Arduino `WebServer`) serves parallel requests one after another.
- **A chip reset** (the page's Restart, `esp_restart()`, the reboot after an OTA) leaves the AP,
  the lease, the forwarded ports and the UDP flows in place. The NAT forgets the station's TCP
  connections and closes their host sockets: the rebooted firmware's stack starts from nothing and
  draws the same local ports as before (the emulated RNG repeats), so a flow left over would take
  its first connections for old ones and drop them.
- **UDP.** A datagram to `HOSTPORT` reaches the guest's `GUESTPORT` from `10.0.2.2:HOSTPORT`; what
  the guest sends back to that address goes to the host peer that sent the last datagram. Payloads
  over 1472 bytes (one unfragmented packet) are dropped.
- **Pace it.** Add `--realtime` (or `--web`) when anything on the host talks to the guest: without
  pacing, emulated time runs faster than wall time, and a browser's seconds become the firmware's
  minutes (timeouts, keep-alives).
- The end-of-run report counts it: `[emu] hostfwd: N TCP connections into the guest (M refused or
  unanswered), N UDP datagrams in, N answers out`; `ESP_EMU_DEBUG_NET=1` logs each one.

## What the network gives the firmware

- **DHCP** — address, mask, gateway, DNS, so `esp_netif` reaches `IP_EVENT_STA_GOT_IP`.
- **DNS** — with NAT, queries go to the host's first `nameserver` from `/etc/resolv.conf` and the
  answer comes back looking as if 10.0.2.3 produced it. With `--net none`, the built-in responder
  answers every A query with 10.0.2.3 itself, so lookups succeed and the connection is then refused.
- **Time** — with NAT, SNTP requests reach the real server the firmware asked for. With `--net none`,
  the built-in SNTP server answers from the host clock (stratum 1), so firmware that waits for time
  still gets it in the first seconds.
- **ICMP echo** — answered inside the emulator for any address, including internet ones; pings never
  leave the host, so they are not a reachability test.
- **TCP/UDP out** — real connections to real hosts: an HTTP API, a Home Assistant instance on the
  LAN, MQTT, HTTPS (TLS runs on the emulated AES/SHA/RSA accelerators).

## Watching what happens

| Switch | Shows |
| --- | --- |
| `ESP_EMU_DEBUG_NET=1` | DHCP/ARP/ICMP/DNS exchanges, every NAT flow and every forwarded connection |
| `ESP_EMU_DEBUG_WIFI=1` | MAC-level events: descriptors, interrupts, TX queues |
| `ESP_EMU_DEBUG_WIFI_FRAMES=1` | every 802.11 frame on the air, decoded |
| `ESP_EMU_DEBUG_AES/SHA/RSA=1` | each accelerator operation as firmware requests it |
| end-of-run `[emu] wifi/net/nat/crypto` lines | frame, lease, flow, byte and operation counts |

## When it does not connect

- **`Send disconnect event, reason=210`** — the firmware has a passphrase compiled in and refuses an
  open AP (`NO_AP_FOUND_W_COMPATIBLE_SECURITY`). Pass a matching `psk=`.
- **No beacons seen / scan finds nothing** — check the SSID matches exactly, including case.
- **Connects but no IP** — look for the DHCP exchange with `ESP_EMU_DEBUG_NET=1`; if the four-way
  handshake did not finish, the station is not yet decrypting anything.
- **TLS fails or hangs** — the accelerators are the usual suspects; the `[emu] crypto:` counters
  show whether AES, SHA and RSA are all being exercised.
- **A host on the LAN is unreachable** — the NAT connects from the *host's* address, so the target
  must be reachable from the Mac, and any firewall there sees the Mac, not the guest.

## Known limits

- **Inbound** is by forwarded port only (`--hostfwd`, above), from 127.0.0.1; the guest sees
  every forwarded client as the gateway.
- **Multicast and mDNS** do not cross the NAT: `something.local` will not resolve, and Home
  Assistant / ESP-IDF discovery protocols will not see anything. Use IP addresses.
- **UDP reply peers** must match the destination IP and port of the outgoing datagram. The
  [connected UDP relay](../esp-soc/src/nat.rs) does not support TFTP's server-selected transfer
  port or replies from a different address on a multi-homed server. Failed host sends drop
  the datagram without retrying and increment `udp_send_errors`,
  with details under `ESP_EMU_DEBUG_NET=1`. The 64-flow UDP table evicts its least recently
  active flow for a new destination or guest port and increments `udp_evicted`; an evicted
  flow's outstanding replies are lost.
- **One station**, no roaming, no power save, no 802.11n rates, no WPA3/SAE or PMF.
- **Traffic is not really encrypted** over the air — frames are plaintext framed as CCMP, which is
  what firmware sees anyway, but a capture of the emulated air is not a realistic WPA2 capture.
