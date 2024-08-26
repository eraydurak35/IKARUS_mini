#include <string.h>
#include "comminication.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nv_storage.h"

static const uint8_t drone_mac_address[6] = {0x04, 0x61, 0x05, 0x05, 0x3A, 0xE4};
static const uint8_t ground_station_mac_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peerInfo;

static gamepad_t *gamepad_ptr = NULL;
static config_t *config_ptr = NULL;
static uint8_t *new_data_recv_flag = NULL;
static uint8_t *motor_test_num_ptr = NULL;
static float *mag_calib_data = NULL;
static waypoint_t waypoint;
static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

void comminication_init(gamepad_t *gmpd, config_t *cfg, float *mg_cal, uint8_t *flg, uint8_t *mtr_tst)
{
    nvs_flash_init();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // reduce power consumption
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8));

    // Set Wi-Fi protocol to long range mode
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, &drone_mac_address[0]));

    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
    memcpy(peerInfo.peer_addr, ground_station_mac_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    gamepad_ptr = gmpd;
    config_ptr = cfg;
    new_data_recv_flag = flg;
    mag_calib_data = mg_cal;
    motor_test_num_ptr = mtr_tst;
}

void comm_send_telem(telemetry_t *telem)
{
    uint8_t buffer[sizeof(telemetry_t) + 1];
    buffer[0] = TELEM_HEADER;
    memcpy(buffer + 1, (uint8_t *)telem, sizeof(telemetry_t));
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}

void comm_send_conf(config_t *conf)
{
    uint8_t buffer[sizeof(config_t) + 1];
    buffer[0] = CONF_HEADER;
    memcpy(buffer + 1, (uint8_t *)conf, sizeof(config_t));
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}

void comm_send_wp()
{
    uint8_t buffer[sizeof(waypoint.latitude) + sizeof(waypoint.longitude) + sizeof(waypoint.altitude) + sizeof(waypoint.end_of_mission_behaviour) + 1];
    buffer[0] = WP_HEADER;
    memcpy(buffer + 1, waypoint.latitude, sizeof(waypoint.latitude));
    memcpy(buffer + 1 + sizeof(waypoint.latitude), waypoint.longitude, sizeof(waypoint.longitude));
    memcpy(buffer + 1 + sizeof(waypoint.latitude) + sizeof(waypoint.longitude), waypoint.altitude, sizeof(waypoint.altitude));
    buffer[226] = waypoint.end_of_mission_behaviour;
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}

void comm_send_motor_test_result(float *result)
{
    uint8_t buffer[sizeof(float) * 4 + 1];
    buffer[0] = MTR_TEST_HEADER;
    memcpy(buffer + 1, result, sizeof(float) * 4);
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (data[0] == 0xFF && len == sizeof(gamepad_t) + 1)
    {
        memcpy(gamepad_ptr, data + 1, sizeof(gamepad_t));
    }
    else if (data[0] == 0xFE && len == sizeof(config_t) + 1)
    {
        memcpy(config_ptr, data + 1, sizeof(config_t));
        save_config(config_ptr);
        *new_data_recv_flag = 1;
    }
    else if (data[0] == 0xFC && len == 2)
    {
        if (data[1] == 10)
        {
            *new_data_recv_flag = 3;
        }
        else if (data[1] == 20)
        {
            *new_data_recv_flag = 4;
        }
    }
    
    else if (data[0] == 0xFD && len == 227)
    {
        *new_data_recv_flag = 2;
        memcpy(waypoint.latitude, data + 1, sizeof(waypoint.latitude));
        memcpy(waypoint.longitude, data + sizeof(waypoint.longitude) + 1, sizeof(waypoint.longitude));
        memcpy(waypoint.altitude, data + (sizeof(waypoint.longitude) * 2) + 1, sizeof(waypoint.altitude));
        waypoint.end_of_mission_behaviour = data[226];

        for (uint8_t i = 0; i < 25; i++)
        {
            printf("%d      %ld     %ld     %d\n", i, waypoint.latitude[i], waypoint.longitude[i], waypoint.altitude[i]);
        }
    }
    else if (data[0] == 0xFB && len == 49)
    {
        *new_data_recv_flag = 5;
        memcpy(mag_calib_data, data + 1, 48);
        save_mag_cal(mag_calib_data);
    }
    else if (data[0] == 0xFA)
    {
        *motor_test_num_ptr = data[1];
    }
    else if (data[0] == 0xF9)
    {
        float x = 0;
        float y = 0;

        memcpy(&x, data + 1, 4);
        memcpy(&y, data + 5, 4);

        printf("%f\t%f\n", x, y);
    }
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}
