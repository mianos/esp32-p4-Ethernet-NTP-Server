/*
 * The u-center bridge on tcp/3333: one client-facing task that owns the UART
 * while a session is attached and forwards both ways.
 */
#include "gps/gps_internal.h"

using namespace gps_detail;

void GpsTime::bridgeTaskEntry(void *arg)
{
    static_cast<GpsTime *>(arg)->bridgeTask();
}

void GpsTime::bridgeTask()
{
    /* Four clients, and the listening socket stays open. u-center taking the
     * port must not lock out a shell, and the single-client version did. Four
     * rather than two because a socket that was closed without the peer saying
     * so keeps its slot until the next select sees it readable, and u-center
     * dead against a bridge with both slots full looks exactly like u-center
     * dead against a bridge that is ignoring it. */
    constexpr int kMaxClients = 4;

    /* What a session sets, in the CFG-MSG shape configureModule() uses: the
     * message class, its id, then one rate per port -- DDC, UART1, UART2, USB,
     * SPI. UART1 only, because it is the only port wired to anything.
     *
     * NAV-SAT is here because the sky view and the C/N0 bars are drawn from it;
     * it is what a bench session exists for. NAV-PVT and NAV-TIMELS are because
     * a session can set every rate to zero -- u-center's revert-to-defaults does
     * exactly that -- and without them a session stops the clock as well as
     * drawing nothing. Three frames and three ACKs, once, at attach. */
    static constexpr uint8_t kSessionPvt[8]    = {0x01, 0x07, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSessionTimels[8] = {0x01, 0x26, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kSessionSat[8]    = {0x01, 0x35, 0, 1, 0, 0, 0, 0};

    for (;;) {
        const int srv = socket(AF_INET, SOCK_STREAM, 0);
        if (srv < 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        int yes = 1;
        setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr = {};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(kBridgePort);
        if (bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
            listen(srv, kMaxClients) != 0) {
            close(srv);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_LOGI(TAG, "u-center bridge on tcp/%d", kBridgePort);

        int clients[kMaxClients] = {-1, -1, -1, -1};
        bool quieted = false;
        /* u-center opens a TCP port by polling MON-COMMS, and this receiver
         * ignores it. A synthetic reply goes out if the module stays silent --
         * 400 ms, because the module's silence is a property of the message and
         * not of the line, and a client waiting on it is waiting on us. */
        int     commsClient   = -1;
        int64_t commsDeadline = 0;
        /* An armed rate change: what was asked for, whether the receiver has
         * acknowledged it, and when to stop waiting. */
        uint32_t baudWanted = 0;
        bool     baudAcked  = false;
        int64_t  baudAtUs   = 0;

        /* The two directions are decoded separately because they interleave and
         * must not share parse state: the client's session and the receiver's
         * message set are different streams that happen to cross one socket. */
        ubx::Decoder clientDecoder;
        ubx::Decoder moduleDecoder;

        /* What the bridge wants to notice in the client's traffic: a request to
         * move the port's rate, and a MON-COMMS poll that will need answering. */
        struct ClientSniffer : ubx::Listener {
            uint32_t *wanted;
            bool     *acked;
            int64_t  *atUs;
            int      *commsClient;
            int64_t  *commsDeadline;
            int       fd = -1;

            ClientSniffer(uint32_t *w, bool *a, int64_t *at, int *cp, int64_t *cd,
                          volatile uint32_t *dr)
                : wanted(w), acked(a), atUs(at), commsClient(cp), commsDeadline(cd), dropped(dr)
            {
            }

            volatile uint32_t *dropped;

            void onDrop(ubx::Drop why, const ubx::Frame &) override
            {
                if (why == ubx::Drop::Checksum) {
                    *dropped = *dropped + 1;
                }
            }

            void onFrame(const ubx::Frame &f) override
            {
                if (f.cls == 0x06 && f.id == 0x00) {
                    /* CFG-PRT is 20 bytes and the decoder has already enforced
                     * it. baudRate is a four-byte field at offset 8; reading two
                     * of those bytes, as this used to, turns 115200 into 49664
                     * and 460800 into 2048. */
                    const uint8_t  port = f.u8(0);
                    const uint32_t baud = f.u4(8);
                    if (port != 1) {
                        ESP_LOGI(TAG, "port %u set to %u: not this line", port,
                                 static_cast<unsigned>(baud));
                        return;
                    }
                    if (baud < 4800 || baud > 921600) {
                        ESP_LOGW(TAG, "%u baud is outside 4800..921600: not following",
                                 static_cast<unsigned>(baud));
                        return;
                    }
                    *wanted = baud;
                    *acked  = false;
                    *atUs   = esp_timer_get_time() + 250000;
                } else if (f.cls == 0x0A && f.id == 0x36) {
                    *commsClient   = fd;
                    *commsDeadline = esp_timer_get_time() + 400000;
                }
            }
        } clientSniffer(&baudWanted, &baudAcked, &baudAtUs, &commsClient, &commsDeadline,
                       &m_bridgeDroppedCrc);

        /* ...and in the receiver's: the answer to that request, and a MON-COMMS
         * reply it bothered to send itself. */
        struct ModuleSniffer : ubx::Listener {
            uint32_t       *wanted;
            bool           *acked;
            int64_t        *atUs;
            int            *commsClient;
            int             fd = -1;

            ModuleSniffer(uint32_t *w, bool *a, int64_t *at, int *cp)
                : wanted(w), acked(a), atUs(at), commsClient(cp)
            {
            }

            void onDrop(ubx::Drop, const ubx::Frame &) override {}

            void onFrame(const ubx::Frame &f) override
            {
                if (f.cls == 0x05 && (f.id == 0x00 || f.id == 0x01) && *wanted != 0 &&
                    f.len >= 2 && f.u8(0) == 0x06 && f.u8(1) == 0x00) {
                    /* 0x01 is ACK-ACK and 0x00 the NAK. They were the other
                     * way round here, so an accepted rate change was dropped
                     * and a refused one followed. */
                    if (f.id == 0x01) {
                        *acked = true;
                        *atUs  = esp_timer_get_time();
                    } else {
                        ESP_LOGW(TAG, "receiver refused the %u baud change",
                                 static_cast<unsigned>(*wanted));
                        *wanted = 0;
                    }
                } else if (f.cls == 0x0A && f.id == 0x36 && *commsClient >= 0) {
                    *commsClient = -1;
                }
            }
        } moduleSniffer(&baudWanted, &baudAcked, &baudAtUs, &commsClient);

        clientDecoder.listen(&clientSniffer);
        moduleDecoder.listen(&moduleSniffer);

        /* Every way a client ends goes through here, so the detached state --
         * port handed back, own configuration restored -- cannot be forgotten
         * on one of the exits. It was forgotten on all of them before: the flag
         * that stops the GPS task reading was only ever cleared when the
         * listening socket itself failed, so after the first session the task
         * never read the port again and the receiver kept a session's rates. */
        auto dropClient = [&](int i, const char *why) {
            text::put(m_bridgeEnd, "{}", why);
            ESP_LOGW(TAG, "client gone: %s", why);
            close(clients[i]);
            clients[i] = -1;
            m_bridgeClients = m_bridgeClients - 1;
            bool any = false;
            for (int j = 0; j < kMaxClients; ++j) {
                any = any || clients[j] >= 0;
            }
            if (!any) {
                quieted        = false;
                m_bridgeActive = false;
                /* Two seconds, not one, and it is a debounce rather than a
                 * delay: a client that is unhappy reconnects about once a
                 * second, and configureModule() takes roughly three of them
                 * because it waits out an ACK on every message. Restoring on
                 * the first detach therefore never finishes before the next
                 * attach stands it aside again, and a bouncing client would
                 * thrash the receiver without either side settling. A pending
                 * reconfigure is stood aside on attach anyway. */
                m_reconfigAtUs = esp_timer_get_time() + 2000000;
            }
        };

        for (;;) {
            fd_set set;
            FD_ZERO(&set);
            FD_SET(srv, &set);
            int maxFd = srv;
            for (int i = 0; i < kMaxClients; ++i) {
                if (clients[i] >= 0) {
                    FD_SET(clients[i], &set);
                    if (clients[i] > maxFd) {
                        maxFd = clients[i];
                    }
                }
            }
            timeval tv = {.tv_sec = 0, .tv_usec = 50000};
            if (select(maxFd + 1, &set, nullptr, nullptr, &tv) < 0) {
                break;
            }

            if (FD_ISSET(srv, &set)) {
                sockaddr_in peer    = {};
                socklen_t   peerLen = sizeof(peer);
                const int   c       = accept(srv, reinterpret_cast<sockaddr *>(&peer), &peerLen);
                if (c >= 0) {
                    int slot = -1;
                    for (int i = 0; i < kMaxClients; ++i) {
                        if (clients[i] < 0) {
                            slot = i;
                            break;
                        }
                    }
                    if (slot < 0) {
                        /* Both slots full. Worth reporting: a client that is
                         * accepted and immediately closed looks identical from
                         * the outside to one that attaches and is ignored, and
                         * telling those apart has cost a round already. */
                        m_bridgeRefused = m_bridgeRefused + 1;
                        text::put(m_bridgeEnd, "refused: both slots taken");
                        ESP_LOGW(TAG, "client refused from %s: both slots taken",
                                 inet_ntoa(peer.sin_addr));
                        close(c);
                    } else {
                        fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
                        clients[slot]    = c;
                        clientSniffer.fd = c;
                        moduleSniffer.fd = c;
                        text::put(m_bridgeLast, "attach");
                        ESP_LOGW(TAG, "client attached from %s", inet_ntoa(peer.sin_addr));
                        m_bridgeSessions = m_bridgeSessions + 1;
                        m_bridgeClients  = m_bridgeClients + 1;
                        if (!quieted) {
                            /* Claim the port and put the session's message set
                             * on it. No delay first: u-center's polls are
                             * already in the socket by the time we get here, so
                             * waiting only moves our frames into the middle of
                             * the handshake it is trying to run. */
                            quieted        = true;
                            m_bridgeActive = true;
                            sendUbx(0x06, 0x01, kSessionPvt, sizeof(kSessionPvt));
                            sendUbx(0x06, 0x01, kSessionTimels, sizeof(kSessionTimels));
                            sendUbx(0x06, 0x01, kSessionSat, sizeof(kSessionSat));
                        }
                    }
                }
            }

            for (int i = 0; i < kMaxClients; ++i) {
                if (clients[i] < 0 || !FD_ISSET(clients[i], &set)) {
                    continue;
                }
                uint8_t       in[256];
                const int     n = recv(clients[i], in, sizeof(in), 0);
                if (n <= 0 && !(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
                    text::Buf<48> why;
                    text::put(why, "recv {} errno {}", n, errno);
                    dropClient(i, why.data());
                    continue;
                }
                if (n <= 0) {
                    continue;
                }
                m_bridgeRx = m_bridgeRx + static_cast<uint32_t>(n);

                /* Built up as text and then cut to the buffer: snprintf returns
                 * the length it *wanted*, so accumulating its return value into
                 * an offset lets the offset pass the end of the array on a long
                 * frame and `sizeof - offset` underflow to something huge. */
                std::string hex;
                for (int k = 0; k < n && k < 40; ++k) {
                    hex += std::format("{:02x} ", static_cast<unsigned>(in[k]));
                }
                text::put(m_bridgeLast, "{}", hex);
                ESP_LOGI(TAG, "client -> %d bytes: %s", n, hex.c_str());

                clientDecoder.feed(std::span(in, static_cast<std::size_t>(n)));
                uart_write_bytes(kUart, in, n);
            }

            uint8_t   out[256];
            /* Only while a client holds the port. With nobody attached the GPS
             * task is the reader, and a second one here took whatever sat in
             * the ring between its reads -- 7280 bytes with no session ever
             * opened, each theft a frame with a hole in it for the clock's
             * decoder. */
            const int got = m_bridgeActive ? uart_read_bytes(kUart, out, sizeof(out), 0) : 0;
            moduleDecoder.feed(std::span(out, static_cast<std::size_t>(got > 0 ? got : 0)));

            /* The rate change, if one is armed. The old code switched the baud
             * the instant it found the request in the client's bytes -- which is
             * when uart_write_bytes() returns, not when the bytes have gone --
             * so the tail of the very frame asking for the change could leave at
             * the new rate while its head left at the old one. It now waits for
             * the module's ACK, drains the transmitter, and switches. A NAK
             * cancels; an ACK that never comes does not, because u-blox sends it
             * on whichever side of the change it likes and a client left at the
             * wrong rate is worse than a rate change the receiver grumbled at. */
            if (baudWanted != 0 && esp_timer_get_time() >= baudAtUs) {
                const uint32_t want = baudWanted;
                baudWanted          = 0;
                if (baudAcked) {
                    ESP_LOGW(TAG, "client asked for %u baud: following", static_cast<unsigned>(want));
                } else {
                    ESP_LOGW(TAG, "client asked for %u baud: no ACK, following anyway",
                             static_cast<unsigned>(want));
                }
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                uart_set_baudrate(kUart, want);
                m_baud = want;
                /* The receiver has almost certainly been asked to change too, so
                 * the next PPS pairing is not comparable with the last. */
                m_frameSeen = false;
                moduleDecoder.reset();
            }
            if (commsClient >= 0 && esp_timer_get_time() > commsDeadline) {
                /* The receiver ignores MON-COMMS outright -- MON-VER, CFG-PRT and
                 * NAV-SAT polls all get answers on the same line, this one gets
                 * nothing -- and u-center opens a TCP port by polling it. So the
                 * bridge answers for the module.
                 *
                 * Built as a real reply for one port: version 0, nPorts 1, port 1
                 * (UART1, the only port here), every counter zero. The first
                 * version of this sent nPorts 0 in a 12-byte payload when a
                 * zero-port payload is 8, so a client that enumerates ports to
                 * decide what it is talking to had a length it could not walk
                 * and nothing in the list. The counters stay honest zeros: the
                 * module does not report them here and inventing figures would
                 * be worse than a blank. */
                constexpr int kLen = 8 + 34;
                uint8_t       reply[kLen] = {0xB5, 0x62, 0x0A, 0x36, 34, 0x00};
                reply[6 + 4]              = 1; /* nPorts */
                reply[6 + 8]              = 1; /* portId, little endian */
                uint8_t a = 0;
                uint8_t b = 0;
                for (int i = 2; i < kLen - 2; ++i) {
                    a = static_cast<uint8_t>(a + reply[i]);
                    b = static_cast<uint8_t>(b + a);
                }
                reply[kLen - 2] = a;
                reply[kLen - 1] = b;
                if (send(commsClient, reply, sizeof(reply), 0) > 0) {
                    m_bridgeTx = m_bridgeTx + sizeof(reply);
                }
                ESP_LOGI(TAG, "answered MON-COMMS locally (the module does not)");
                commsClient = -1;
            }

            if (got > 0) {
                m_bridgeTx = m_bridgeTx + static_cast<uint32_t>(got);
                for (int i = 0; i < kMaxClients; ++i) {
                    if (clients[i] < 0) {
                        continue;
                    }
                    int sent   = 0;
                    int64_t by = esp_timer_get_time() + 500000;
                    while (sent < got && esp_timer_get_time() < by) {
                        const int w = send(clients[i], out + sent, got - sent, 0);
                        if (w > 0) {
                            sent += w;
                        } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                            vTaskDelay(1); /* one tick: 2 ms rounds to none at 100 Hz, a spin */
                        } else {
                            text::Buf<48> why;
                            text::put(why, "send errno {}", errno);
                            dropClient(i, why.data());
                            break;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < kMaxClients; ++i) {
            if (clients[i] >= 0) {
                close(clients[i]);
            }
        }
        close(srv);
        m_bridgeActive  = false;
        m_bridgeClients = 0;
        ESP_LOGW(TAG, "bridge reset; re-applying configuration");
        m_reconfigAtUs = esp_timer_get_time() + 500000;
    }
}
