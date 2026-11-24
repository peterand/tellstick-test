/*
 * Tellstick Net v2 (PIC18F67J60) - DHCP client + UDP discovery responder.
 *
 * - Gets its IPv4 address via DHCP (DISCOVER/OFFER/REQUEST/ACK, renews at
 *   half the lease, restarts if the lease expires or a NAK arrives).
 * - Listens for UDP to 255.255.255.255 (or its own IP) port 30303 and answers
 *   the sender with "Hello Peter" to UDP destination port 2048.
 * - Answers ARP for its IP.
 *
 * Bare-metal on the PIC's internal Ethernet MAC/PHY (no Microchip TCP/IP
 * stack). Toolchain: MPLAB X + XC8, device PIC18F67J60. Assumes a 25 MHz
 * crystal (required by the internal Ethernet PHY) - verify on your board.
 */
#include <xc.h>
#include <stdint.h>
#include <string.h>

/* ---- config bits (same as Microchip's TCP/IP stack for 25 MHz parts) ---- */
#pragma config WDT = OFF, FOSC2 = ON, FOSC = HSPLL, ETHLED = ON
#pragma config XINST = OFF, DEBUG = OFF, STVR = ON

/* ---- user settings ---- */
static const uint8_t MY_MAC[6] = {0xac, 0xca, 0x54, 0x00, 0x6C, 0xac}; /* locally administered */
#define LISTEN_PORT  30303u
#define REPLY_PORT   2048u
static const char REPLY_TEXT[] = "Hello Peter";

/* Timer0: 16-bit, 1:256, Fcy = 10.4167 MHz -> overflow every ~1.61 s.
 * All DHCP timing is counted in these ticks. */
#define TICK_X10      16u        /* tick length in 1/10 s */
#define T0CON_VAL     0x87       /* TMR0ON, 16-bit, internal clock, prescaler 1:256 */

/* ---- register bit masks ---- */
#define ECON1_TXRST   0x80
#define ECON1_TXRTS   0x08
#define ECON1_RXEN    0x04
#define ECON2_PKTDEC  0x40
#define ECON2_ETHEN   0x20
#define ESTAT_PHYRDY  0x01
#define EIR_TXIF      0x08
#define EIR_TXERIF    0x02
#define MISTAT_BUSY   0x01
#define INTCON_TMR0IF 0x04
#define PHCON2        0x10
#define PHCON2_HDLDIS 0x0100

/* ---- ETH buffer layout (8 KB). ERXST must be 0 (silicon errata) ---- */
#define RXSTART 0x0000u
#define RXSTOP  0x19FFu          /* odd */
#define TXSTART 0x1A00u

#define RXBUF_LEN 80u            /* headers only; DHCP options are streamed */
#define TXBUF_LEN 80u

static uint8_t  rx[RXBUF_LEN];
static uint8_t  tx[TXBUF_LEN];
static uint16_t rx_len;          /* full frame length without CRC */
static uint16_t rx_buffered;     /* bytes of it copied into rx[] */
static uint16_t next_packet = RXSTART;

/* ---------------------------------------------------------------- PHY --- */
static void phy_write(uint8_t reg, uint16_t val)
{
    MIREGADR = reg;
    MIWRL = (uint8_t)val;
    MIWRH = (uint8_t)(val >> 8);
    while (MISTAT & MISTAT_BUSY) { }
}

/* ------------------------------------------------------------- ETH init -- */
static void eth_init(void)
{
    ECON1 = 0;                              /* RX and TX off */
    while (!(ESTAT & ESTAT_PHYRDY)) { }     /* wait for 25 MHz oscillator/PHY */
    ECON2 |= ECON2_ETHEN;                   /* enable Ethernet module */

    ERXSTL = RXSTART & 0xFF;  ERXSTH = RXSTART >> 8;
    ERXNDL = RXSTOP & 0xFF;   ERXNDH = RXSTOP >> 8;
    ERXRDPTL = RXSTOP & 0xFF; ERXRDPTH = RXSTOP >> 8;
    ERDPTL = RXSTART & 0xFF;  ERDPTH = RXSTART >> 8;
    next_packet = RXSTART;

    /* accept unicast-to-us, broadcast, with good CRC only */
    ERXFCON = 0xA1;                         /* UCEN | CRCEN | BCEN */

    /* MAC, half duplex (PHY has no auto-negotiation; switch parallel-detects) */
    MACON1 = 0x0D;                          /* MARXEN | RXPAUS | TXPAUS */
    MACON3 = 0x32;                          /* pad to 60, TXCRCEN, FRMLNEN */
    MACON4 = 0x40;                          /* DEFER */
    MABBIPG = 0x12;
    MAIPGL  = 0x12;
    MAIPGH  = 0x0C;
    MAMXFLL = 1518 & 0xFF;
    MAMXFLH = 1518 >> 8;

    MAADR1 = MY_MAC[0]; MAADR2 = MY_MAC[1]; MAADR3 = MY_MAC[2];
    MAADR4 = MY_MAC[3]; MAADR5 = MY_MAC[4]; MAADR6 = MY_MAC[5];

    phy_write(PHCON2, PHCON2_HDLDIS);       /* no half-duplex loopback */

    ECON1 |= ECON1_RXEN;                    /* start receiving */
}

/* ---------------------------------------------------------------- RX ----- */
/* If a frame is waiting: reads its header and the first RXBUF_LEN bytes into
 * rx[], leaves the ETH read pointer right after them (so further EDATA reads
 * stream the rest of the frame), and returns 1. Call eth_release() when done. */
static uint8_t eth_poll(void)
{
    uint16_t len, i;

    if (EPKTCNT == 0) return 0;

    ERDPTL = next_packet & 0xFF;
    ERDPTH = next_packet >> 8;

    next_packet  = EDATA;                   /* next packet pointer */
    next_packet |= (uint16_t)EDATA << 8;
    len  = EDATA;                           /* byte count (incl. 4-byte CRC) */
    len |= (uint16_t)EDATA << 8;
    (void)EDATA; (void)EDATA;               /* receive status */

    rx_len = (len > 4) ? len - 4 : 0;
    rx_buffered = (rx_len > RXBUF_LEN) ? RXBUF_LEN : rx_len;
    for (i = 0; i < rx_buffered; i++) rx[i] = EDATA;
    return 1;
}

static void eth_release(void)
{
    /* free the buffer space; ERXRDPT must be odd (errata) */
    uint16_t rd = (next_packet == RXSTART) ? RXSTOP : (uint16_t)(next_packet - 1);
    ERXRDPTL = rd & 0xFF;
    ERXRDPTH = rd >> 8;
    ECON2 |= ECON2_PKTDEC;
}

/* ---------------------------------------------------------------- TX ----- */
/* Start a frame of 'len' bytes; then write bytes to EDATA; then eth_tx_go(). */
static void eth_tx_start(uint16_t len)
{
    uint16_t end = TXSTART + len;           /* control byte at TXSTART, frame follows */

    /* errata: reset TX logic if a previous transmit errored */
    if (EIR & EIR_TXERIF) {
        ECON1 |= ECON1_TXRST;
        ECON1 &= ~ECON1_TXRST;
        EIR &= ~(EIR_TXERIF | EIR_TXIF);
    }
    EWRPTL = TXSTART & 0xFF;  EWRPTH = TXSTART >> 8;
    ETXSTL = TXSTART & 0xFF;  ETXSTH = TXSTART >> 8;
    ETXNDL = end & 0xFF;      ETXNDH = end >> 8;
    EDATA = 0;                              /* per-packet control: use MACON3 defaults */
}

static void eth_tx_go(void)
{
    ECON1 |= ECON1_TXRTS;
    while (ECON1 & ECON1_TXRTS) { }         /* wait until sent (or aborted) */
}

static void eth_send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    eth_tx_start(len);
    for (i = 0; i < len; i++) EDATA = buf[i];
    eth_tx_go();
}

/* ------------------------------------------------------------ helpers ---- */
static uint16_t ip_checksum(const uint8_t *p, uint8_t len)
{
    uint32_t sum = 0;
    uint8_t i;
    for (i = 0; i < len; i += 2) sum += ((uint16_t)p[i] << 8) | p[i + 1];
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)~sum;
}

/* ================================================================ DHCP === */
enum { S_INIT, S_SELECTING, S_REQUESTING, S_BOUND, S_RENEWING };

static uint8_t  state = S_INIT;
static uint8_t  my_ip[4];            /* valid in S_BOUND / S_RENEWING */
static uint8_t  offered_ip[4];
static uint8_t  server_id[4];
static uint8_t  xid[4];
static uint8_t  wait_ticks;          /* ticks since last request was sent */
static uint32_t elapsed;             /* ticks since lease start */
static uint32_t lease_ticks;

#define DHCP_COOKIE_OFFSET 278u      /* 14 eth + 20 ip + 8 udp + 236 bootp; options follow the 4-byte cookie */

static uint8_t has_ip(void)
{
    return state == S_BOUND || state == S_RENEWING;
}

/* msg: 1 DISCOVER, 3 REQUEST. req_ip/srv_id: options 50/54 (or NULL).
 * ciaddr: our current IP when renewing (or NULL). */
static void dhcp_send(uint8_t msg, const uint8_t *req_ip, const uint8_t *srv_id,
                      const uint8_t *ciaddr)
{
    uint8_t  opt_len = 3 + 4 + 1;                 /* 53, 55(1,3), end */
    uint16_t udp_len, ip_len, total, csum;
    uint8_t  i;

    if (req_ip) opt_len += 6;
    if (srv_id) opt_len += 6;
    udp_len = 8 + 240 + opt_len;
    ip_len  = 20 + udp_len;
    total   = 14 + ip_len;

    memset(tx, 0, 42);
    memset(&tx[0], 0xFF, 6);                      /* broadcast */
    memcpy(&tx[6], MY_MAC, 6);
    tx[12] = 0x08; tx[13] = 0x00;
    tx[14] = 0x45;
    tx[16] = ip_len >> 8; tx[17] = (uint8_t)ip_len;
    tx[22] = 128; tx[23] = 17;
    if (ciaddr) memcpy(&tx[26], ciaddr, 4);       /* src 0.0.0.0 otherwise */
    memset(&tx[30], 0xFF, 4);                     /* dst 255.255.255.255 */
    csum = ip_checksum(&tx[14], 20);
    tx[24] = csum >> 8; tx[25] = (uint8_t)csum;
    tx[34] = 0; tx[35] = 68;                      /* src port 68 */
    tx[36] = 0; tx[37] = 67;                      /* dst port 67 */
    tx[38] = udp_len >> 8; tx[39] = (uint8_t)udp_len;

    eth_tx_start(total);
    for (i = 0; i < 42; i++) EDATA = tx[i];

    /* BOOTP fixed part */
    EDATA = 1; EDATA = 1; EDATA = 6; EDATA = 0;   /* op, htype, hlen, hops */
    for (i = 0; i < 4; i++) EDATA = xid[i];
    EDATA = 0; EDATA = 0;                         /* secs */
    EDATA = 0x80; EDATA = 0;                      /* flags: broadcast reply */
    for (i = 0; i < 4; i++) EDATA = ciaddr ? ciaddr[i] : 0;
    for (i = 0; i < 12; i++) EDATA = 0;           /* yiaddr, siaddr, giaddr */
    for (i = 0; i < 6; i++) EDATA = MY_MAC[i];    /* chaddr */
    for (i = 0; i < 10; i++) EDATA = 0;           /* chaddr padding */
    for (i = 0; i < 192; i++) EDATA = 0;          /* sname + file */
    EDATA = 0x63; EDATA = 0x82; EDATA = 0x53; EDATA = 0x63;

    EDATA = 53; EDATA = 1; EDATA = msg;
    if (req_ip) { EDATA = 50; EDATA = 4; for (i = 0; i < 4; i++) EDATA = req_ip[i]; }
    if (srv_id) { EDATA = 54; EDATA = 4; for (i = 0; i < 4; i++) EDATA = srv_id[i]; }
    EDATA = 55; EDATA = 2; EDATA = 1; EDATA = 3;  /* want subnet mask, router */
    EDATA = 255;

    eth_tx_go();
}

static void dhcp_start(void)
{
    xid[0] = MY_MAC[2]; xid[1] = MY_MAC[3]; xid[2] = MY_MAC[4];
    xid[3]++;                                     /* new transaction */
    state = S_SELECTING;
    wait_ticks = 0;
    dhcp_send(1, 0, 0, 0);
}

/* Called about every 1.6 s. */
static void dhcp_tick(void)
{
    switch (state) {
    case S_INIT:
        dhcp_start();
        break;
    case S_SELECTING:
    case S_REQUESTING:
        if (++wait_ticks >= 3) dhcp_start();      /* no answer: start over */
        break;
    case S_BOUND:
        if (++elapsed >= (lease_ticks >> 1)) {    /* T1: renew */
            xid[3]++;
            state = S_RENEWING;
            wait_ticks = 0;
            dhcp_send(3, 0, 0, my_ip);
        }
        break;
    case S_RENEWING:
        ++elapsed;
        if (elapsed >= lease_ticks) {             /* lease lost */
            memset(my_ip, 0, 4);
            dhcp_start();
        } else if (++wait_ticks >= 3) {
            wait_ticks = 0;
            dhcp_send(3, 0, 0, my_ip);
        }
        break;
    }
}

/* Returns 1 if the frame was a DHCP packet (UDP dst port 68), else 0. */
static uint8_t dhcp_rx(void)
{
    uint16_t i, pos;
    uint8_t  code, olen, n, b, type = 0;
    uint8_t  optb[4], srv[4], cookie[4];
    uint8_t  have_srv = 0;
    uint32_t lease = 0;

    if (rx_buffered < 62 || rx[14] != 0x45 || rx[23] != 17) return 0;
    if (rx[36] != 0 || rx[37] != 68) return 0;
    if (rx_len < 286 || rx_buffered < RXBUF_LEN) return 1;
    if (rx[42] != 2 || memcmp(&rx[46], xid, 4) != 0) return 1;  /* BOOTREPLY, our xid */

    /* ETH read pointer is at byte RXBUF_LEN: skip to the magic cookie */
    for (i = RXBUF_LEN; i < DHCP_COOKIE_OFFSET; i++) (void)EDATA;
    for (n = 0; n < 4; n++) cookie[n] = EDATA;
    if (cookie[0] != 0x63 || cookie[1] != 0x82 || cookie[2] != 0x53 || cookie[3] != 0x63)
        return 1;

    pos = DHCP_COOKIE_OFFSET + 4;
    while (pos < rx_len) {
        code = EDATA; pos++;
        if (code == 255) break;
        if (code == 0) continue;
        if (pos >= rx_len) break;
        olen = EDATA; pos++;
        for (n = 0; n < olen; n++) { b = EDATA; if (n < 4) optb[n] = b; }
        pos += olen;
        if (code == 53 && olen >= 1) type = optb[0];
        else if (code == 54 && olen == 4) { memcpy(srv, optb, 4); have_srv = 1; }
        else if (code == 51 && olen == 4)
            lease = ((uint32_t)optb[0] << 24) | ((uint32_t)optb[1] << 16)
                  | ((uint16_t)optb[2] << 8) | optb[3];
    }

    if (type == 2 && state == S_SELECTING && have_srv) {          /* OFFER */
        memcpy(offered_ip, &rx[58], 4);                           /* yiaddr */
        memcpy(server_id, srv, 4);
        state = S_REQUESTING;
        wait_ticks = 0;
        dhcp_send(3, offered_ip, server_id, 0);
    } else if (type == 5 && (state == S_REQUESTING || state == S_RENEWING)) { /* ACK */
        if (state == S_REQUESTING) memcpy(my_ip, &rx[58], 4);
        if (lease == 0) lease = 3600;
        lease_ticks = (lease / TICK_X10) * 10;                    /* seconds -> ticks */
        if (lease_ticks < 4) lease_ticks = 4;
        elapsed = 0;
        wait_ticks = 0;
        state = S_BOUND;
    } else if (type == 6) {                                       /* NAK */
        memset(my_ip, 0, 4);
        dhcp_start();
    }
    return 1;
}

/* --------------------------------------------------------------- ARP ----- */
static void handle_arp(void)
{
    /* rx: eth(14) + arp(28). op at 20..21, sender MAC 22, sender IP 28, target IP 38 */
    if (!has_ip() || rx_buffered < 42) return;
    if (rx[20] != 0 || rx[21] != 1) return;         /* not a request */
    if (memcmp(&rx[38], my_ip, 4) != 0) return;

    memcpy(&tx[0], &rx[6], 6);                      /* dst = requester */
    memcpy(&tx[6], MY_MAC, 6);
    tx[12] = 0x08; tx[13] = 0x06;
    tx[14] = 0; tx[15] = 1; tx[16] = 0x08; tx[17] = 0; tx[18] = 6; tx[19] = 4;
    tx[20] = 0; tx[21] = 2;                         /* reply */
    memcpy(&tx[22], MY_MAC, 6);
    memcpy(&tx[28], my_ip, 4);
    memcpy(&tx[32], &rx[22], 6);
    memcpy(&tx[38], &rx[28], 4);
    eth_send(tx, 42);
}

/* --------------------------------------------------------------- UDP ----- */
static void handle_ip(void)
{
    uint8_t  ihl;
    uint16_t dport, plen, csum;
    uint8_t  o;
    static const uint8_t bcast[4] = {255, 255, 255, 255};

    if (!has_ip()) return;
    if (rx_len < 14 + 20 + 8) return;
    if ((rx[14] >> 4) != 4) return;
    ihl = (rx[14] & 0x0F) * 4;
    if (ihl < 20 || (uint16_t)(14 + ihl + 4) > rx_buffered) return;
    if (rx[14 + 9] != 17) return;                   /* UDP only */
    if (memcmp(&rx[14 + 16], bcast, 4) != 0 && memcmp(&rx[14 + 16], my_ip, 4) != 0) return;

    dport = ((uint16_t)rx[14 + ihl + 2] << 8) | rx[14 + ihl + 3];
    if (dport != LISTEN_PORT) return;

    /* ---- build reply ---- */
    plen = sizeof(REPLY_TEXT) - 1;
    memcpy(&tx[0], &rx[6], 6);                      /* dst MAC = sender MAC */
    memcpy(&tx[6], MY_MAC, 6);
    tx[12] = 0x08; tx[13] = 0x00;

    o = 14;
    tx[o + 0] = 0x45; tx[o + 1] = 0;                /* v4, 20-byte header, TOS 0 */
    tx[o + 2] = 0; tx[o + 3] = (uint8_t)(20 + 8 + plen);
    tx[o + 4] = 0; tx[o + 5] = 0;                   /* id */
    tx[o + 6] = 0; tx[o + 7] = 0;                   /* flags/frag */
    tx[o + 8] = 64; tx[o + 9] = 17;                 /* ttl, UDP */
    tx[o + 10] = 0; tx[o + 11] = 0;                 /* checksum placeholder */
    memcpy(&tx[o + 12], my_ip, 4);
    memcpy(&tx[o + 16], &rx[14 + 12], 4);           /* dst IP = sender IP */
    csum = ip_checksum(&tx[o], 20);
    tx[o + 10] = csum >> 8; tx[o + 11] = (uint8_t)csum;

    o = 34;
    tx[o + 0] = LISTEN_PORT >> 8; tx[o + 1] = (uint8_t)LISTEN_PORT;  /* src port */
    tx[o + 2] = REPLY_PORT >> 8;  tx[o + 3] = (uint8_t)REPLY_PORT;   /* dst port */
    tx[o + 4] = 0; tx[o + 5] = (uint8_t)(8 + plen);
    tx[o + 6] = 0; tx[o + 7] = 0;                   /* UDP checksum optional in IPv4 */
    memcpy(&tx[o + 8], REPLY_TEXT, plen);

    eth_send(tx, 14 + 20 + 8 + plen);
}

/* -------------------------------------------------------------- main ----- */
void main(void)
{
    eth_init();
    T0CON = T0CON_VAL;
    INTCON &= ~INTCON_TMR0IF;
    dhcp_start();

    for (;;) {
        if (INTCON & INTCON_TMR0IF) {
            INTCON &= ~INTCON_TMR0IF;
            dhcp_tick();
        }

        if (eth_poll()) {
            if (rx_buffered >= 14) {
                if (rx[12] == 0x08 && rx[13] == 0x06)      handle_arp();
                else if (rx[12] == 0x08 && rx[13] == 0x00) { if (!dhcp_rx()) handle_ip(); }
            }
            eth_release();
        }
    }
}
