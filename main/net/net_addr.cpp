#include "net/net_addr.h"

#include <algorithm>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "lwip/def.h"

namespace net_addr {
namespace {

/* Exactly four decimal parts, 0..255; unlike ip4addr_aton, no "10.1" or hex. */
bool parseQuadText(const std::string &text, uint32_t &hostOrder)
{
    uint32_t parts[4] = {};
    size_t   index    = 0;
    int      count    = 0;

    while (index < text.size() && count < 4) {
        uint32_t value  = 0;
        int      digits = 0;
        while (index < text.size() && text[index] >= '0' && text[index] <= '9') {
            value = value * 10 + static_cast<uint32_t>(text[index] - '0');
            ++index;
            if (++digits > 3 || value > 255) {
                return false;
            }
        }
        if (digits == 0) {
            return false;
        }
        parts[count++] = value;
        if (index < text.size()) {
            if (text[index] != '.' || index + 1 == text.size()) {
                return false;
            }
            ++index;
        }
    }
    if (count != 4 || index != text.size()) {
        return false;
    }

    hostOrder = (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3];
    return true;
}

}  // namespace

bool parseCidr(const std::string &text, esp_ip4_addr_t &addr, uint8_t &prefix)
{
    const size_t slash = text.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 == text.size()) {
        return false;
    }

    const std::string len = text.substr(slash + 1);
    /* Digits only, so "2x" is rejected rather than read as 2. */
    if (len.size() > 2 ||
        !std::all_of(len.begin(), len.end(), [](char c) { return c >= '0' && c <= '9'; })) {
        return false;
    }
    const long value = std::strtol(len.c_str(), nullptr, 10);
    /* /31 and /32 leave no room for a gateway. */
    if (value < 1 || value > 30) {
        return false;
    }

    uint32_t host = 0;
    if (!parseQuadText(text.substr(0, slash), host)) {
        return false;
    }
    prefix    = static_cast<uint8_t>(value);
    addr.addr = htonl(host);
    return true;
}

bool parseQuad(const std::string &text, esp_ip4_addr_t &addr)
{
    uint32_t host = 0;
    if (!parseQuadText(text, host)) {
        return false;
    }
    addr.addr = htonl(host);
    return true;
}

esp_ip4_addr_t maskFromPrefix(uint8_t prefix)
{
    esp_ip4_addr_t mask{};
    const uint32_t bits = prefix == 0 ? 0 : 0xffffffffu << (32 - prefix);
    mask.addr           = htonl(bits);
    return mask;
}

std::string text(const esp_ip4_addr_t &addr)
{
    const uint32_t host    = ntohl(addr.addr);
    char           buf[16] = {};
    std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", static_cast<unsigned>((host >> 24) & 0xff),
                  static_cast<unsigned>((host >> 16) & 0xff),
                  static_cast<unsigned>((host >> 8) & 0xff), static_cast<unsigned>(host & 0xff));
    return buf;
}

}  // namespace net_addr
