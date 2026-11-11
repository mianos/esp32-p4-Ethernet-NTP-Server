#include "net_state.h"

#if CONFIG_LWIP_IPV6
#include <cctype>
#include <cstring>

#include "lwip/ip6_addr.h"
#endif

namespace net_state {
namespace {

esp_netif_t       *s_netif = nullptr;
std::atomic<bool> s_link_up{false};

#if CONFIG_LWIP_IPV6
/* esp_ip6_addr_t and lwIP's ip6_addr_t both start with the four address words,
 * so copying just those is enough; the zone stays IP6_NO_ZONE, which is what
 * formatting wants anyway. lwIP prints hex in upper case; RFC 5952 says lower,
 * and lower is what people paste into curl. */
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
    esp_ip6_addr_t addr{};
    if (s_netif == nullptr || esp_netif_get_ip6_global(s_netif, &addr) != ESP_OK) {
        return {};
    }
    return text6(addr);
}
#endif

}  // namespace net_state
