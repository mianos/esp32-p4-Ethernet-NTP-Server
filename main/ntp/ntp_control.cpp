#include "ntp/ntp_control.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <inttypes.h>

#include "clock/time_ref.h"

namespace {

/* Protocol constants, RFC 9327 sections 2, 3 and 4. */
constexpr size_t kHeaderLen = 12;
constexpr size_t kMaxData   = kControlChunk;

constexpr uint8_t kBitResponse = 0x80;
constexpr uint8_t kBitMore     = 0x20;
constexpr uint8_t kBitError    = 0x40;
constexpr uint8_t kOpMask      = 0x1f;

constexpr uint8_t kOpReadStat  = 1;
constexpr uint8_t kOpReadVar   = 2;
constexpr uint8_t kOpWriteVar  = 3;
constexpr uint8_t kOpReadClock = 4;

/* Error status words, RFC 9327 table 9. */
constexpr uint8_t kErrPermission = 1;
constexpr uint8_t kErrBadOp      = 3;
constexpr uint8_t kErrBadAssoc   = 4;
constexpr uint8_t kErrUnknownVar = 5;

/* Peer status word: configured and reachable, and selected as the source. */
constexpr uint8_t kPstConfig  = 0x80;
constexpr uint8_t kPstReach   = 0x10;
constexpr uint8_t kSelSysPeer = 6;
/* No system peer, RFC 9327 table 4. */
constexpr uint8_t kEvNoSysPeer = 8;

/* System status word: what is doing the disciplining. */
constexpr uint8_t kSrcPps = 1; /* "calibrated atomic clock (e.g. PPS)" */
constexpr uint8_t kSrcNtp = 6;

/* The only association this device has. */
constexpr uint16_t kAssocId = 1;

constexpr uint32_t kNtpUnixDelta = 2208988800u;

/* 2^-20 s, about 1 us, as in the replies. */
constexpr int32_t kPrecision = -20;

/* Does a comma-separated request field name `var`? An empty field names
 * everything, which is what `ntpq -c rv` sends. */
bool wants(const uint8_t *field, size_t len, const char *var)
{
    if (len == 0) {
        return true;
    }
    const char  *text    = reinterpret_cast<const char *>(field);
    const size_t wantLen = std::strlen(var);
    for (size_t i = 0; i < len; ++i) {
        if (i != 0 && text[i - 1] != ',') {
            continue;
        }
        if (len - i < wantLen || std::strncmp(text + i, var, wantLen) != 0) {
            continue;
        }
        if (len - i == wantLen || text[i + wantLen] == ',') {
            return true;
        }
    }
    return false;
}

/* Appends whole `name=value,` pairs, only for the names requested; a pair that
 * does not fit is dropped entire. */
class Writer {
public:
    Writer(uint8_t *out, size_t cap, const uint8_t *filter, size_t filterLen)
        /* The whole buffer less padding: the list is fragmented afterwards. */
        : m_out(out), m_cap(cap >= 3 ? cap - 3 : 0), m_filter(filter), m_filterLen(filterLen)
    {
        m_len = kHeaderLen;
    }

    /* Checked against the literal at every call: this is what keeps a "%s"
     * from being handed an integer, which nothing else in the file would. */
    __attribute__((format(printf, 3, 4))) void pair(const char *name, const char *fmt, ...)
    {
        if (!wants(m_filter, m_filterLen, name)) {
            return;
        }
        char    line[96];
        va_list args;
        va_start(args, fmt);
        const int n = std::vsnprintf(line, sizeof(line), fmt, args);
        va_end(args);
        if (n <= 0) {
            return;
        }
        const size_t used =
            static_cast<size_t>(n) < sizeof(line) ? static_cast<size_t>(n) : sizeof(line) - 1;
        const size_t nameLen = std::strlen(name);
        if (m_len + nameLen + 1 + used + 1 > m_cap) {
            return;
        }
        std::memcpy(m_out + m_len, name, nameLen);
        m_len += nameLen;
        m_out[m_len++] = '=';
        std::memcpy(m_out + m_len, line, used);
        m_len += used;
        m_out[m_len++] = ',';
    }

    void integer(const char *name, long long value) { pair(name, "%lld", value); }

    /* A counter, in both the plain and the `_r` "with units" form ntpq asks
     * for when it renders sysstats. */
    void counter(const char *name, long long value)
    {
        pair(name, "%lld", value);
        char raw[64];
        std::snprintf(raw, sizeof(raw), "%s_r", name);
        pair(raw, "%lld", value);
    }

    /* Binary rows, for READSTAT: no name, so no filter. */
    void raw(const uint8_t *bytes, size_t n)
    {
        if (m_len + n > m_cap) {
            return;
        }
        std::memcpy(m_out + m_len, bytes, n);
        m_len += n;
    }

    size_t len() const { return m_len; }
    size_t dataLen() const { return m_len - kHeaderLen; }

private:
    uint8_t       *m_out;
    size_t         m_cap;
    const uint8_t *m_filter;
    size_t         m_filterLen;
    size_t         m_len;
};

/* l_fp as 0xSSSSSSSS.FFFFFFFF; ntpq rejects it without the 0x. */
void stamp(Writer &w, const char *name, time_t when)
{
    if (when == 0) {
        return;
    }
    /* PRIx32 rather than %x: uint32_t is `unsigned long` here, and %x takes
     * an `unsigned int`, which only looked right because they are both 32
     * bits. */
    w.pair(name, "0x%08" PRIx32 ".%08" PRIx32, static_cast<uint32_t>(when) + kNtpUnixDelta,
           static_cast<uint32_t>(0));
}

/* Every name the writers can emit. Anything else gets "unknown variable",
 * which makes ntpq retry one name at a time rather than give up. */
constexpr const char *kSystemNames[] = {
    "version",      "processor",     "system",        "peermode",    "peeradr",    "stratum",
    "refid",        "leap",          "precision",     "rootdelay",   "rootdisp",   "rootdist",
    "reftime",      "sys_jitter",    "clk_jitter",    "clk_wander",  "authdelay",  "ss_uptime",
    "ss_numctlreq", "ss_received",   "ss_processed",  "ss_kodsent",  "ss_limited", "ss_badformat",
    "ss_badauth",   "ss_declined",   "ss_restricted", "ss_reset",    "ss_thisver", "ss_oldver",
    "ss_ver1",      "ss_ver1client", "ss_ver1zero",   "ss_ver1symm",
};

constexpr const char *kPeerNames[] = {
    "srcadr",   "srchost",   "dstadr",     "hmode",  "hpoll",     "ppoll",
    "reach",    "unreach",   "stratum",    "refid",  "leap",      "precision",
    "delay",    "offset",    "dispersion", "jitter", "filtdelay", "filtoffset",
    "filtdisp", "rootdelay", "rootdisp",   "flash",  "reftime",   "rec",
};

bool knownName(const char *name, size_t len, const char *const *list, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (std::strlen(list[i]) == len && std::strncmp(name, list[i], len) == 0) {
            return true;
        }
    }
    /* Any known name is also known with an `_r` suffix. */
    if (len > 2 && name[len - 2] == '_' && name[len - 1] == 'r') {
        return knownName(name, len - 2, list, count);
    }
    return false;
}

/* All requested names must be known, or the request is an error. */
bool allKnown(const uint8_t *field, size_t len, const char *const *list, size_t count)
{
    const char *text = reinterpret_cast<const char *>(field);
    size_t      from = 0;
    while (from <= len) {
        size_t to = from;
        while (to < len && text[to] != ',') {
            ++to;
        }
        if (to > from && !knownName(text + from, to - from, list, count)) {
            return false;
        }
        from = to + 1;
    }
    return true;
}

double ms(int64_t ns)
{
    return static_cast<double>(ns) / 1e6;
}

/* System variables, named per RFC 5905 appendix B for ntpq sysinfo and
 * sysstats. */
void writeSystemVars(Writer &w, const ControlFacts &f)
{
    w.pair("version", "\"%s\"", f.version != nullptr && f.version[0] != '\0' ? f.version : "?");
    w.pair("processor", "\"ESP32-P4\"");
    w.pair("system", "\"FreeRTOS\"");
    w.pair("peermode", "%d", 3);
    w.pair("peeradr", "\"%s\"", f.peer.c_str());
    w.integer("stratum", f.stratum);
    w.pair("refid", "\"%s\"", refIdText(f.refId, f.stratum).c_str());
    w.integer("leap", f.locked ? f.leap : 3);
    w.integer("precision", kPrecision);
    w.pair("rootdelay", "%.6f", ms(f.rootDelayNs));
    w.pair("rootdisp", "%.6f", ms(f.dispersionNs));
    w.pair("rootdist", "%.6f", ms(f.rootDelayNs / 2 + f.dispersionNs));
    stamp(w, "reftime", f.lastSync);
    w.pair("sys_jitter", "%.6f", ms(f.jitterNs));
    w.pair("clk_jitter", "%.6f", ms(f.jitterNs));
    w.pair("clk_wander", "0.000000");
    w.pair("authdelay", "0.000000");

    w.counter("ss_uptime", f.uptimeS);
    w.counter("ss_numctlreq", f.controlRequests);
    w.counter("ss_received", f.requests);
    w.counter("ss_thisver", f.requests);

    w.counter("ss_oldver", 0);
    w.counter("ss_ver1", 0);
    w.counter("ss_ver1client", 0);
    w.counter("ss_ver1zero", 0);
    w.counter("ss_ver1symm", 0);
    w.counter("ss_processed", f.answered);
    w.counter("ss_kodsent", f.kod);
    w.counter("ss_limited", f.dropped);
    w.counter("ss_badformat", 0);
    w.counter("ss_badauth", 0);
    w.counter("ss_declined", 0);
    w.counter("ss_restricted", 0);
    /* The counters are never cleared, so this is the uptime; ntpq divides by
     * it. */
    w.counter("ss_reset", f.uptimeS);
}

/* Association variables. The receiver appears as ntpd shows a reference clock:
 * 127.127.20.0, with srchost in parentheses. */
void writePeerVars(Writer &w, const ControlFacts &f)
{
    w.pair("srcadr", "%s", f.peer.c_str());
    if (f.upstream) {
        w.pair("dstadr", "%s", f.peer.c_str());
    } else {
        w.pair("srchost", "\"GPS(0)\"");
    }
    w.pair("hmode", "%d", 3);
    w.integer("hpoll", 4);
    w.integer("ppoll", 4);
    w.pair("reach", "0x%03x", f.locked ? 0x377u : 0x000u);
    w.integer("unreach", f.locked ? 0 : 1);
    w.integer("stratum", f.stratum);
    w.pair("refid", "\"%s\"", refIdText(f.refId, f.stratum).c_str());
    w.integer("leap", f.locked ? f.leap : 3);
    w.integer("precision", kPrecision);
    w.pair("delay", "%.6f", ms(f.rootDelayNs));
    w.pair("offset", "%.6f", ms(f.offsetNs));
    w.pair("dispersion", "%.6f", ms(f.dispersionNs));
    w.pair("jitter", "%.6f", ms(f.jitterNs));
    w.pair("filtdelay", "%.6f", ms(f.rootDelayNs));
    w.pair("filtoffset", "%.6f", ms(f.offsetNs));
    w.pair("filtdisp", "%.6f", ms(f.dispersionNs));
    w.pair("rootdelay", "%.6f", ms(f.rootDelayNs));
    w.pair("rootdisp", "%.6f", ms(f.dispersionNs));
    w.pair("flash", "0x%04x", 0u);
    stamp(w, "reftime", f.lastSync);
    stamp(w, "rec", f.lastSync);
}

}  // namespace

size_t buildControlReply(uint8_t *out, size_t cap, const uint8_t *req, size_t len,
                         const ControlFacts &facts, const uint16_t offset, uint16_t &total)
{
    total = 0;
    if (req == nullptr || len < kHeaderLen || cap < kHeaderLen) {
        return 0;
    }

    /* Versions 1 to 4 only (RFC 9327 section 2). */
    const uint8_t version = static_cast<uint8_t>((req[0] >> 3) & 0x07);
    if (version < 1 || version > 4) {
        return 0;
    }

    /* Never answer a response, or two servers can loop. */
    if ((req[1] & kBitResponse) != 0) {
        return 0;
    }

    const uint8_t  op       = static_cast<uint8_t>(req[1] & kOpMask);
    const uint16_t sequence = static_cast<uint16_t>((req[2] << 8) | req[3]);
    const uint16_t assoc    = static_cast<uint16_t>((req[6] << 8) | req[7]);

    /* The request's data field. */
    const size_t   claimed  = static_cast<size_t>((req[10] << 8) | req[11]);
    const uint8_t *field    = req + kHeaderLen;
    size_t         fieldLen = len - kHeaderLen;
    if (fieldLen > kMaxData) {
        fieldLen = kMaxData;
    }
    /* Bounded by the count field when it is non-zero. */
    if (claimed != 0 && fieldLen > claimed) {
        fieldLen = claimed;
    }
    /* Strip the 32-bit padding. */
    while (fieldLen > 0 && field[fieldLen - 1] == 0) {
        --fieldLen;
    }

    Writer w(out, cap, field, fieldLen);

    uint8_t error = 0;
    if (op == kOpWriteVar) {
        error = kErrPermission;
    } else if (op != kOpReadStat && op != kOpReadVar && op != kOpReadClock) {
        error = kErrBadOp;
    } else if (assoc != 0 && assoc != kAssocId) {
        error = kErrBadAssoc;
    }

    if (error == 0 && op == kOpReadVar && fieldLen != 0) {
        const bool ok = assoc == 0 ? allKnown(field, fieldLen, kSystemNames,
                                              sizeof(kSystemNames) / sizeof(kSystemNames[0]))
                                   : allKnown(field, fieldLen, kPeerNames,
                                              sizeof(kPeerNames) / sizeof(kPeerNames[0]));
        if (!ok) {
            error = kErrUnknownVar;
        }
    }

    /* The association gets the peer status word and the system the system
     * word; ntpq takes the selection character from the peer word. */
    const uint8_t peerFlags =
        facts.locked ? static_cast<uint8_t>(kPstConfig | kPstReach | kSelSysPeer) : kPstConfig;
    const uint16_t peerStatus = static_cast<uint16_t>(static_cast<uint16_t>(peerFlags) << 8);
    const uint16_t sysStatus  = static_cast<uint16_t>(((facts.locked ? facts.leap : 3u) << 14) |
                                                      ((facts.upstream ? kSrcNtp : kSrcPps) << 8) |
                                                      (facts.locked ? 0u : kEvNoSysPeer));

    uint16_t status = 0;
    if (error == 0) {
        status = assoc == 0 ? sysStatus : peerStatus;

        switch (op) {
        case kOpReadStat: {
            /* One association id and its status word, big-endian. */
            const uint8_t row[4] = {0, static_cast<uint8_t>(kAssocId),
                                    static_cast<uint8_t>(peerStatus >> 8),
                                    static_cast<uint8_t>(peerStatus & 0xFF)};
            w.raw(row, sizeof(row));
            break;
        }
        case kOpReadVar:
            if (assoc == 0) {
                writeSystemVars(w, facts);
            } else {
                writePeerVars(w, facts);
            }
            break;
        case kOpReadClock:
        default:           break;
        }
    } else {
        /* Error status word: the code in the high byte. */
        status = static_cast<uint16_t>(static_cast<uint16_t>(error) << 8);
    }

    /* Echo version, sequence and association; LI is 0 (RFC 9327 section 2). */
    out[0]             = static_cast<uint8_t>((version << 3) | 6);
    out[1]             = static_cast<uint8_t>(kBitResponse | (error != 0 ? kBitError : 0) | op);
    out[2]             = static_cast<uint8_t>(sequence >> 8);
    out[3]             = static_cast<uint8_t>(sequence & 0xFF);
    out[4]             = static_cast<uint8_t>(status >> 8);
    out[5]             = static_cast<uint8_t>(status & 0xFF);
    out[6]             = static_cast<uint8_t>(assoc >> 8);
    out[7]             = static_cast<uint8_t>(assoc & 0xFF);
    out[8]             = static_cast<uint8_t>(offset >> 8);
    out[9]             = static_cast<uint8_t>(offset & 0xFF);
    const size_t count = w.dataLen();
    out[10]            = static_cast<uint8_t>(count >> 8);
    out[11]            = static_cast<uint8_t>(count & 0xFF);

    total = static_cast<uint16_t>(count);

    /* One chunk from `offset`; the M bit marks all but the last. */
    size_t chunk = count > offset ? count - offset : 0;
    if (chunk > kMaxData) {
        chunk = kMaxData;
    }
    if (offset + chunk < count) {
        out[1] |= kBitMore;
    }
    out[10] = static_cast<uint8_t>(chunk >> 8);
    out[11] = static_cast<uint8_t>(chunk & 0xFF);
    std::memmove(out + kHeaderLen, out + kHeaderLen + offset, chunk);

    /* Pad to 32 bits; the padding is not counted. */
    size_t sent = kHeaderLen + chunk;
    while (sent & 0x3) {
        out[sent++] = 0;
    }
    return sent;
}
