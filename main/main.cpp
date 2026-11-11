/*
 * esp32p4-ntp — Ethernet bring-up, DHCP, ping, the HTTP surface, and a clock.
 *
 * Board: Waveshare ESP32-P4-ETH (ESP32-P4NRW32 + IP101GRI 10/100 PHY).
 *
 * The ESP32-P4 has a 10/100 MAC on chip but *no* PHY, so the board's IP101
 * PHY is driven over RMII. The P4 can route EMAC signals to almost any pad
 * (SOC_EMAC_USE_MULTI_IO_MUX), so the pin map below has to match the board
 * schematic -- it is the Espressif ESP32-P4 reference-design wiring, which
 * this board follows. If you ever see "link down" with a known-good cable,
 * this table is the first thing to check against the schematic.
 *
 * What this firmware does:
 *   1. brings up EMAC + IP101 in RMII mode (50 MHz REF_CLK comes from the PHY)
 *   2. starts the default ESP-IDF Ethernet netif, which runs a DHCP client
 *   3. logs the lease (IP/netmask/gateway) as soon as it arrives
 *   4. answers ICMP echo requests -- lwIP replies to ping by default once the
 *      netif is up, no extra code needed
 *   5. serves the mianesp HTTP endpoints (/healthz, /config, /config/reset,
 *      /ntp/sync) once there is an address to reach them on
 *   6. (test mode) steps the clock from an upstream SNTP server -- the one the
 *      DHCP server names in option 42, or one from /config -- so that the NTP
 *      responder in the next milestone has a real time to serve
 *   7. takes firmware updates over the MII/RJ45 link itself (POST /firmware,
 *      into the inactive slot) and confirms a freshly booted image once it is
 *      reachable, so a bad one rolls back instead of needing a cable
 *
 * It is intentionally small: it is the plumbing the NTP server will sit on.
 */

#include <inttypes.h>

#include <atomic>

#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "net_state.h"
#include "ntp_time.h"
#include "settings.h"
#include "web_server.h"

namespace {

/* ---- Board wiring (Waveshare ESP32-P4-ETH, IP101GRI) --------------------
 * Reference design values. MDC/MDIO are the SMI (management) bus, the rest
 * are the RMII data plane. PHY_RST is active low; PHY_ADDR 1 is the IP101
 * default set by the PHYAD strap resistors on the board.
 */
constexpr int kEthMdcGpio        = 31;
constexpr int kEthMdioGpio       = 52;
constexpr int kEthPhyResetGpio   = 51;
constexpr int kEthPhyAddr        = 1;
constexpr int kEthRmiiRefclkGpio = 50; /* 50 MHz REF_CLK from PHY -> P4 (input) */
constexpr int kEthRmiiTxEnGpio   = 49;
constexpr int kEthRmiiTxd0Gpio   = 34;
constexpr int kEthRmiiTxd1Gpio   = 35;
constexpr int kEthRmiiRxd0Gpio   = 29;
constexpr int kEthRmiiRxd1Gpio   = 30;
constexpr int kEthRmiiCrsDvGpio  = 28;

const char *TAG = "eth_demo";

esp_eth_handle_t s_eth_handle;
/* Written by the IP/ETH event handlers, read by the status task. */
std::atomic<bool> s_have_ip{false};
/* Only ever touched from the event task, so a plain bool is enough. */
bool s_web_started = false;

/* Read one PHY register through the SMI (MDC/MDIO) bus. */
esp_err_t phyReadReg(uint32_t reg, uint32_t *val)
{
    esp_eth_phy_reg_rw_data_t rd = {};
    rd.reg_addr    = reg;
    rd.reg_value_p = val;
    return esp_eth_ioctl(s_eth_handle, ETH_CMD_READ_PHY_REG, &rd);
}

/* Scan the 32 MDIO addresses for a PHY and print what is there. If the SMI
 * pins in ethInit() are wrong, every address reads 0xffff/0 and nothing is
 * logged -- that is the diagnostic signal.
 */
void phyScan()
{
    for (uint32_t addr = 0; addr < 32; addr++) {
        uint32_t id1 = 0, id2 = 0;
        esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PHY_ADDR, &addr);
        if (phyReadReg(2, &id1) != ESP_OK || phyReadReg(3, &id2) != ESP_OK) {
            continue;
        }
        if (id1 == 0 || id1 == 0xffff) {
            continue;
        }
        ESP_LOGI(TAG, "MDIO addr %" PRIu32 ": PHY ID 0x%04" PRIx32 "%04" PRIx32
                 " (IP101 = 0x0243xxxx)", addr, id1, id2);
    }
    uint32_t addr = kEthPhyAddr;
    esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PHY_ADDR, &addr);
}

/* One line of PHY status: BMSR gives link + autoneg-done, BMCR the rest. */
void phyLogStatus()
{
    uint32_t bmsr = 0, bmcr = 0;
    if (phyReadReg(1, &bmsr) != ESP_OK || phyReadReg(0, &bmcr) != ESP_OK) {
        ESP_LOGW(TAG, "phy: MDIO read failed (check MDC/MDIO wiring)");
        return;
    }
    ESP_LOGI(TAG, "phy: bmsr=0x%04" PRIx32 " link=%d autoneg_done=%d bmcr=0x%04" PRIx32,
             bmsr, (bmsr >> 2) & 1, (bmsr >> 5) & 1, bmcr);
}

/* -------------------------------------------------------------------------
 * Ethernet driver: EMAC (RMII) + generic IEEE 802.3 PHY driver.
 * The generic PHY driver speaks the standard MII registers, which is all the
 * IP101 needs for autonegotiation and link-up detection.
 * ---------------------------------------------------------------------- */
esp_err_t ethInit(esp_eth_handle_t *eth_handle_out)
{
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr       = kEthPhyAddr;
    phy_config.reset_gpio_num = kEthPhyResetGpio;

    /* ETH_ESP32_EMAC_DEFAULT_CONFIG() is written with C99 designated
     * initialisers in an order C++ rejects, so its defaults are set by hand
     * here -- same values, plus this board's pin map. */
    eth_esp32_emac_config_t emac_config = {};
    emac_config.smi_gpio.mdc_num  = kEthMdcGpio;
    emac_config.smi_gpio.mdio_num = kEthMdioGpio;

    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    /* The PHY's 50 MHz REF_CLK output clocks the MAC. */
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = kEthRmiiRefclkGpio;

    emac_config.clock_config_out_in.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config_out_in.rmii.clock_gpio = -1; /* no clock out */
    emac_config.dma_burst_len = ETH_DMA_BURST_LEN_32;
    emac_config.intr_priority = 0;
    emac_config.mdc_freq_hz   = 0; /* driver default */

    /* P4 routes the RMII data signals through the GPIO matrix. */
    emac_config.emac_dataif_gpio.rmii.tx_en_num  = kEthRmiiTxEnGpio;
    emac_config.emac_dataif_gpio.rmii.txd0_num   = kEthRmiiTxd0Gpio;
    emac_config.emac_dataif_gpio.rmii.txd1_num   = kEthRmiiTxd1Gpio;
    emac_config.emac_dataif_gpio.rmii.rxd0_num   = kEthRmiiRxd0Gpio;
    emac_config.emac_dataif_gpio.rmii.rxd1_num   = kEthRmiiRxd1Gpio;
    emac_config.emac_dataif_gpio.rmii.crs_dv_num = kEthRmiiCrsDvGpio;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (mac == nullptr) {
        ESP_LOGE(TAG, "failed to create EMAC instance");
        return ESP_FAIL;
    }

    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    if (phy == nullptr) {
        ESP_LOGE(TAG, "failed to create PHY instance");
        mac->del(mac);
        return ESP_FAIL;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&eth_config, eth_handle_out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(err));
        mac->del(mac);
        phy->del(phy);
        return err;
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Event handlers
 * ---------------------------------------------------------------------- */
void ethEvent(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED: {
        uint8_t mac_addr[6] = {};
        esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        net_state::setLinkUp(true);
        ESP_LOGI(TAG, "link up   (MAC %02x:%02x:%02x:%02x:%02x:%02x)",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGI(TAG, "requesting DHCP lease ...");
        break;
    }
    case ETHERNET_EVENT_DISCONNECTED:
        s_have_ip.store(false, std::memory_order_relaxed);
        net_state::setLinkUp(false);
        ESP_LOGW(TAG, "link down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet stopped");
        break;
    default:
        break;
    }
}

/* An image that arrived over POST /firmware boots in ESP_OTA_IMG_PENDING_VERIFY:
 * the bootloader goes back to the previous slot unless the app confirms this
 * one. The bar here is a DHCP lease plus a listening HTTP server -- an image
 * that gets that far is reachable and can be replaced again, which is the
 * property that matters on a box whose only interface is the RJ45.
 */
void confirmRunningImage()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   state   = ESP_OTA_IMG_UNDEFINED;

    if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return;
    }
    if (state != ESP_OTA_IMG_PENDING_VERIFY) {
        return; /* first boot after a USB flash: nothing to confirm */
    }

    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA image confirmed: rollback to the previous slot cancelled");
    } else {
        ESP_LOGW(TAG, "could not confirm the OTA image: %s", esp_err_to_name(err));
    }
}

void gotIpEvent(void *, esp_event_base_t, int32_t, void *event_data)

{
    auto       *event   = static_cast<ip_event_got_ip_t *>(event_data);
    const auto *ip_info = &event->ip_info;

    s_have_ip.store(true, std::memory_order_relaxed);
    ESP_LOGI(TAG, "DHCP lease: ip " IPSTR "  mask " IPSTR "  gw " IPSTR,
             IP2STR(&ip_info->ip), IP2STR(&ip_info->netmask), IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP");

    /* The lease is where the option-42 NTP server (if any) becomes known, so
     * the clock can poll it now rather than at the next SNTP interval. */
    NtpTime::instance().onGotIp();

    /* The HTTP surface is only reachable once there is an address, so start it
     * on the first lease. Later rebinds leave it listening on all addresses. */
    if (!s_web_started) {
        esp_err_t err = NtpWebServer::instance().begin();
        if (err == ESP_OK) {
            s_web_started = true;
            ESP_LOGI(TAG, "http: http://" IPSTR "/healthz  http://" IPSTR "/config",
                     IP2STR(&ip_info->ip), IP2STR(&ip_info->ip));
            /* Reachable is the condition for keeping an OTA'd image; the slot
             * only needs confirming once, on its first boot. */
            confirmRunningImage();
        } else {
            ESP_LOGE(TAG, "http server failed to start: %s", esp_err_to_name(err));
        }
    }
}

/* -------------------------------------------------------------------------
 * Status task: says what the demo is waiting for, then goes quiet.
 * ---------------------------------------------------------------------- */
void statusTask(void *)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!s_have_ip.load(std::memory_order_relaxed)) {
            ESP_LOGI(TAG, "waiting for link + DHCP lease (is the RJ45 plugged in?)");
            phyLogStatus();
        }
    }
}

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "esp32p4-ntp: RMII + IP101, DHCP client, HTTP /healthz, upstream clock");

    /* Settings live in NVS. A stale blob from an older layout is erased rather
     * than carried forward: the defaults are the documented starting point. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "erasing nvs partition (%s)", esp_err_to_name(nvs_err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    Settings::instance().begin();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Armed before the first DHCP DISCOVER: lwIP only keeps an option-42 NTP
     * server if sntp_servermode_dhcp() is on when the ACK is processed. */
    ESP_ERROR_CHECK(NtpTime::instance().begin());

    ESP_ERROR_CHECK(ethInit(&s_eth_handle));

    /* Which PHY answers on the SMI bus, and what does it see? */
    phyScan();
    phyLogStatus();

    /* Default Ethernet netif: DHCP client on, ICMP echo on (lwIP default). */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t       *netif     = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(netif != nullptr ? ESP_OK : ESP_FAIL);
    net_state::setNetif(netif);

    /* DHCP option 12 is built when the client's first DISCOVER goes out, which
     * happens once the netif comes up -- so the hostname has to be in place
     * before esp_eth_start(), not after the first lease. */
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, Settings::instance().hostname().c_str()));

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(netif, glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &ethEvent, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &gotIpEvent, nullptr));

    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    xTaskCreate(statusTask, "eth_status", 3072, nullptr, 3, nullptr);
}
