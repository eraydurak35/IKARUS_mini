#ifndef COMM_H
#define COMM_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "nvs_flash.h"
#include "typedefs.h"

#define CHANNEL 6
#define DATARATE WIFI_PHY_RATE_54M//WIFI_PHY_RATE_24M

#define TELEM_HEADER 0xFF
#define CONF_HEADER 0xFE
#define WP_HEADER 0xFD
#define MTR_TEST_HEADER 0xFC


//esp_err_t esp_wifi_set_max_tx_power(int8_t power);
/*   * @attention This API gets maximum WiFi transmiting power. Values got
  *            from power are mapped to transmit power levels as follows:
  *            - 78: 19.5dBm
  *            - 76: 19dBm
  *            - 74: 18.5dBm
  *            - 68: 17dBm
  *            - 60: 15dBm
  *            - 52: 13dBm
  *            - 44: 11dBm
  *            - 34: 8.5dBm
  *            - 28: 7dBm
  *            - 20: 5dBm
  *            - 8:  2dBm
  *            - -4: -1dBm */


void comminication_init(gamepad_t *gmpd, config_t *cfg, float *mg_cal, uint8_t *flg, uint8_t *mtr_tst);
void comm_send_telem(telemetry_t *telem);
void comm_send_conf(config_t *conf);
void comm_send_wp();
void comm_send_motor_test_result(float *result);



#endif