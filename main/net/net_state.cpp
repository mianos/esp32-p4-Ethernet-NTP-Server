#include "net/net_state.h"

#if CONFIG_LWIP_IPV6
#include <cctype>
#include <cstring>

#include "lwip/ip6_addr.h"
#endif

namespace net_state {
namespace {

esp_netif_t      *s_netif = nullptr;
std::atomic<bool> s_link_up{false};

#if CONFIG_LWIP_IPV6
/* Both address types start with the four words; the zone stays IP6_NO_ZONE.
 * Lower-cased, as RFC 5952 asks; lwIP prints upper case. */
std::string text6(const esp_ip6_addr_t &addr)
{
    ip6_addr_t lwip{};
    std::memcpy(lwip.addr, addr.addr, sizeof(lwip.addr));
    char buf[IP6ADDR_STRLEN_MAX] = {};
    ip6addr_ntoa_r(&lwip, buf, sizeof(buf));
    std::string text(buf);
    for (char &c : text) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
}
#endif

}  // namespace

esp_netif_t *netif()
{
    return s_netif;
}

void setNetif(esp_netif_t *netif)
{
    s_netif = netif;
}

bool linkUp()
{
    return s_link_up.load(std::memory_order_relaxed);
}

void setLinkUp(bool up)
{
    s_link_up.store(up, std::memory_order_relaxed);
}

#if CONFIG_LWIP_IPV6
std::string ipv6Text(const esp_ip6_addr_t &addr)
{
    return text6(addr);
}

std::string ipv6LinkLocal()
{
    esp_ip6_addr_t addr{};
    if (s_netif == nullptr || esp_netif_get_ip6_linklocal(s_netif, &addr) != ESP_OK) {
        return {};
    }
    return text6(addr);
}

std::string ipv6Global()
{
    /* The first non-link-local address. esp_netif_get_ip6_global() only
     * accepts 2000::/3, which would miss a ULA (fd00::/8). */
    if (s_netif == nullptr) {
        return {};
    }
    esp_ip6_addr_t addrs[CONFIG_LWIP_IPV6_NUM_ADDRESSES] = {};
    const int      count                                 = esp_netif_get_all_ip6(s_netif, addrs);
    for (int i = 0; i < count; ++i) {
        if (esp_netif_ip6_get_addr_type(&addrs[i]) != ESP_IP6_ADDR_IS_LINK_LOCAL) {
            return text6(addrs[i]);
        }
    }
    return {};
}
#endif

}  // namespace net_state
