/* The u-center bridge on tcp/3333: owns the UART while a client is attached
 * and forwards both ways. */
#include "gps/gps_internal.h"

using namespace gps_detail;

void GpsTime::bridgeTaskEntry(void *arg)
{
    static_cast<GpsTime *>(arg)->bridgeTask();
}

void GpsTime::bridgeTask()
{
    /* A closed peer keeps its slot until select() sees it, so there is room
     * for more than one live client. */
    constexpr int kMaxClients = 4;

    /* CFG-MSG sent at attach (UART1). NAV-PVT and NAV-TIMELS keep the clock
     * running if u-center zeroes every rate; NAV-SAT feeds its sky view. */
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

        sockaddr_in addr     = {};
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

        int  clients[kMaxClients] = {-1, -1, -1, -1};
        bool quieted              = false;
        /* u-center polls MON-COMMS on connect and this receiver ignores it, so
         * the bridge answers after 400 ms of silence. */
        int     commsClient   = -1;
        int64_t commsDeadline = 0;
        /* A client's pending CFG-PRT rate change. */
        uint32_t baudWanted = 0;
        bool     baudAcked  = false;
        int64_t  baudAtUs   = 0;

        /* One decoder per direction; the streams interleave. */
        ubx::Decoder clientDecoder;
        ubx::Decoder moduleDecoder;

        /* Client side: watch for CFG-PRT and MON-COMMS polls. */
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
                    /* baudRate is u4 at offset 8. */
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

        /* Receiver side: the ACK for that change, or a real MON-COMMS reply. */
        struct ModuleSniffer : ubx::Listener {
            uint32_t *wanted;
            bool     *acked;
            int64_t  *atUs;
            int      *commsClient;
            int       fd = -1;

            ModuleSniffer(uint32_t *w, bool *a, int64_t *at, int *cp)
                : wanted(w), acked(a), atUs(at), commsClient(cp)
            {
            }

            void onDrop(ubx::Drop, const ubx::Frame &) override {}

            void onFrame(const ubx::Frame &f) override
            {
                if (f.cls == 0x05 && (f.id == 0x00 || f.id == 0x01) && *wanted != 0 && f.len >= 2 &&
                    f.u8(0) == 0x06 && f.u8(1) == 0x00) {
                    /* 0x01 is ACK-ACK, 0x00 ACK-NAK. */
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

        /* Every client exit comes through here: hand the port back and schedule
         * our configuration to be restored. */
        auto dropClient = [&](int i, const char *why) {
            text::put(m_bridgeEnd, "{}", why);
            ESP_LOGW(TAG, "client gone: %s", why);
            close(clients[i]);
            clients[i]      = -1;
            m_bridgeClients = m_bridgeClients - 1;
            bool any        = false;
            for (int j = 0; j < kMaxClients; ++j) {
                any = any || clients[j] >= 0;
            }
            if (!any) {
                quieted        = false;
                m_bridgeActive = false;
                /* Debounced: a client that reconnects every second would
                 * otherwise restart a ~3 s configureModule() each time. */
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
                        /* All slots full; reported, since from outside it looks
                         * like being ignored. */
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
                            /* At once: u-center's polls are already queued. */
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
                uint8_t   in[256];
                const int n = recv(clients[i], in, sizeof(in), 0);
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

                /* Built as a string and then truncated: summing snprintf's
                 * return values can run past the buffer. */
                std::string hex;
                for (int k = 0; k < n && k < 40; ++k) {
                    hex += std::format("{:02x} ", static_cast<unsigned>(in[k]));
                }
                text::put(m_bridgeLast, "{}", hex);
                ESP_LOGI(TAG, "client -> %d bytes: %s", n, hex.c_str());

                clientDecoder.feed(std::span(in, static_cast<std::size_t>(n)));
                uart_write_bytes(kUart, in, n);
            }

            uint8_t out[256];
            /* Only with a client attached; otherwise the GPS task reads. */
            const int got = m_bridgeActive ? uart_read_bytes(kUart, out, sizeof(out), 0) : 0;
            moduleDecoder.feed(std::span(out, static_cast<std::size_t>(got > 0 ? got : 0)));

            /* Switch the host rate after the ACK and a TX drain, or at the
             * deadline without one (the ACK may come at either rate). A NAK
             * cancels. */
            if (baudWanted != 0 && esp_timer_get_time() >= baudAtUs) {
                const uint32_t want = baudWanted;
                baudWanted          = 0;
                if (baudAcked) {
                    ESP_LOGW(TAG, "client asked for %u baud: following",
                             static_cast<unsigned>(want));
                } else {
                    ESP_LOGW(TAG, "client asked for %u baud: no ACK, following anyway",
                             static_cast<unsigned>(want));
                }
                uart_wait_tx_done(kUart, pdMS_TO_TICKS(50));
                uart_set_baudrate(kUart, want);
                m_baud = want;

                m_frameSeen = false;
                moduleDecoder.reset();
            }
            if (commsClient >= 0 && esp_timer_get_time() > commsDeadline) {
                /* MON-COMMS reply for the silent module: version 0, one port
                 * (UART1), all counters zero. */
                constexpr int kLen        = 8 + 34;
                uint8_t       reply[kLen] = {0xB5, 0x62, 0x0A, 0x36, 34, 0x00};
                reply[6 + 4]              = 1; /* nPorts */
                reply[6 + 8]              = 1; /* portId, little endian */
                uint8_t a                 = 0;
                uint8_t b                 = 0;
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
                    int     sent = 0;
                    int64_t by   = esp_timer_get_time() + 500000;
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
