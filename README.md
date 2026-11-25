# Tellstick Net v2 – UDP "Hello Peter"

Bare-metal firmware for PIC18F67J60 (internal Ethernet MAC/PHY, no TCP/IP stack).
Gets its IP via DHCP (renews at half the lease, restarts on NAK/expiry).
Receives UDP to 255.255.255.255:30303 (or its own IP) and replies "Hello Peter"
to the sender, UDP destination port 2048 (source port 30303). Also answers ARP, sends a gratuitous ARP on a new lease, and answers ping (ICMP echo).
The responder is silent until a lease is obtained.

## Build (MPLAB X + XC8)
1. File > New Project > Microchip Embedded > Standalone Project, device PIC18F67J60, tool PICkit 4, compiler XC8.
2. Add `main.c` under Source Files.
3. Edit `MY_MAC` in `main.c` (default is a locally administered placeholder).
4. Make and Program Device.

Check: the internal PHY needs a 25 MHz crystal (config `FOSC=HSPLL`). Confirm on your board.
Find the assigned IP in your router's DHCP lease list (client MAC = `MY_MAC`).
DHCP timing uses Timer0 (~0.4 s ticks) and assumes Fosc = 41.667 MHz (`OSCTUNE = 0x40`, as in the Telldus firmware).
RE1 drives the board's Ethernet RX polarity swap; the code toggles it until the first frame is received.
The LEDs on RA0/RA1 are the Ethernet link/activity LEDs.
Untested on hardware; it was only syntax-checked on a host compiler.

## Test
```bash
echo -n hi | socat - UDP-DATAGRAM:255.255.255.255:30303,broadcast &   # send
nc -ul 2048                                                            # receive
```
