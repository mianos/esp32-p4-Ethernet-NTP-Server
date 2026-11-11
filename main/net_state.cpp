#include "net_state.h"

namespace net_state {
namespace {

esp_netif_t *s_netif = nullptr;
/* Written from the ETH_EVENT handler, read from the httpd task: a plain flag is
 * all the precision a status readout needs. */
volatile bool s_link_up = false;

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
    return s_link_up;
}

void setLinkUp(bool up)
{
    s_link_up = up;
}

}  // namespace net_state
