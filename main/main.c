/*
 * esp32p4-ntp — step 1: Ethernet bring-up, DHCP, ICMP echo (ping).
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
 *
 * It is intentionally small: it is the plumbing the NTP server will sit on.
 */

#include <inttypes.h>
#include <string.h>

#include "esp_check.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/* ---- Board wiring (Waveshare ESP32-P4-ETH, IP101GRI) --------------------
 * Reference design values. MDC/MDIO are the SMI (management) bus, the rest
 * are the RMII data plane. PHY_RST is active low; PHY_ADDR 1 is the IP101
 * default set by the PHYAD strap resistors on the board.
 */
#define ETH_MDC_GPIO            31
#define ETH_MDIO_GPIO           52
#define ETH_PHY_RST_GPIO        51
#define ETH_PHY_ADDR            1
#define ETH_RMII_REFCLK_GPIO    50  /* 50 MHz REF_CLK from PHY -> P4 (input) */
#define ETH_RMII_TX_EN_GPIO     49
#define ETH_RMII_TXD0_GPIO      34
#define ETH_RMII_TXD1_GPIO      35
#define ETH_RMII_RXD0_GPIO      29
#define ETH_RMII_RXD1_GPIO      30
#define ETH_RMII_CRS_DV_GPIO    28

static const char *TAG = "eth_demo";

static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static volatile bool s_have_ip;

/* Read one PHY register through the SMI (MDC/MDIO) bus. */
static esp_err_t phy_read_reg(uint32_t reg, uint32_t *val)
{
    esp_eth_phy_reg_rw_data_t rd = { .reg_addr = reg, .reg_value_p = val };
    return esp_eth_ioctl(s_eth_handle, ETH_CMD_READ_PHY_REG, &rd);
}

/* Scan the 32 MDIO addresses for a PHY and print what is there. If the SMI
 * pins in eth_init() are wrong, every address reads 0xffff/0 and nothing is
 * logged -- that is the diagnostic signal.
 */
static void phy_scan(void)
{
    for (uint32_t addr = 0; addr < 32; addr++) {
        uint32_t id1 = 0, id2 = 0;
        esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PHY_ADDR, &addr);
        if (phy_read_reg(2, &id1) != ESP_OK || phy_read_reg(3, &id2) != ESP_OK) {
            continue;
        }
        if (id1 == 0 || id1 == 0xffff) {
            continue;
        }
        ESP_LOGI(TAG, "MDIO addr %" PRIu32 ": PHY ID 0x%04" PRIx32 "%04" PRIx32
                 " (IP101 = 0x0243xxxx)", addr, id1, id2);
    }
    uint32_t addr = ETH_PHY_ADDR;
    esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PHY_ADDR, &addr);
}

/* One line of PHY status: BMSR gives link + autoneg-done, BMCR the rest. */
static void phy_log_status(void)
{
    uint32_t bmsr = 0, bmcr = 0;
    if (phy_read_reg(1, &bmsr) != ESP_OK || phy_read_reg(0, &bmcr) != ESP_OK) {
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
static esp_err_t eth_init(esp_eth_handle_t *eth_handle_out)
{
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;

    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    /* The PHY's 50 MHz REF_CLK output clocks the MAC. */
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = ETH_RMII_REFCLK_GPIO;

    /* P4 routes the RMII data signals through the GPIO matrix. */
    emac_config.emac_dataif_gpio.rmii.tx_en_num = ETH_RMII_TX_EN_GPIO;
    emac_config.emac_dataif_gpio.rmii.txd0_num = ETH_RMII_TXD0_GPIO;
    emac_config.emac_dataif_gpio.rmii.txd1_num = ETH_RMII_TXD1_GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd0_num = ETH_RMII_RXD0_GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd1_num = ETH_RMII_RXD1_GPIO;
    emac_config.emac_dataif_gpio.rmii.crs_dv_num = ETH_RMII_CRS_DV_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (mac == NULL) {
        ESP_LOGE(TAG, "failed to create EMAC instance");
        return ESP_FAIL;
    }

    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    if (phy == NULL) {
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
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED: {
        uint8_t mac_addr[6] = {0};
        esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "link up   (MAC %02x:%02x:%02x:%02x:%02x:%02x)",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_LOGI(TAG, "requesting DHCP lease ...");
        break;
    }
    case ETHERNET_EVENT_DISCONNECTED:
        s_have_ip = false;
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

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    s_have_ip = true;
    ESP_LOGI(TAG, "DHCP lease: ip " IPSTR "  mask " IPSTR "  gw " IPSTR,
             IP2STR(&ip_info->ip), IP2STR(&ip_info->netmask), IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "ready: ping the IP above from your LAN -- ICMP echo is answered by lwIP");
}

/* -------------------------------------------------------------------------
 * Status task: says what the demo is waiting for, then goes quiet.
 * ---------------------------------------------------------------------- */
static void status_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!s_have_ip) {
            ESP_LOGI(TAG, "waiting for link + DHCP lease (is the RJ45 plugged in?)");
            phy_log_status();
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "esp32p4-ntp eth demo: RMII + IP101, DHCP client, ICMP echo");

    ESP_ERROR_CHECK(eth_init(&s_eth_handle));

    /* Which PHY answers on the SMI bus, and what does it see? */
    phy_scan();
    phy_log_status();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Default Ethernet netif: DHCP client on, ICMP echo on (lwIP default). */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(s_eth_netif ? ESP_OK : ESP_FAIL);

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    xTaskCreate(status_task, "eth_status", 3072, NULL, 3, NULL);
}
