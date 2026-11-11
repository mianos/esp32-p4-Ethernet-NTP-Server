#include "net_state.h"

namespace net_state {
namespace {

esp_netif_t       *s_netif = nullptr;
std::atomic<bool> s_link_up{false};

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

}  // namespace net_state
