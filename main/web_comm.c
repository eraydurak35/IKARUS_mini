#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include <esp_http_server.h>
#include "typedefs.h"

static const char *TAG = "web_socket_server";

gamepad_t *gamepad_ptr = NULL;
telemetry_small_t *telem_small_p = NULL;

const char* html_page = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset=\"UTF-8\">"
"<style>"
"  :root {"
"    --joystick-size: 36vw;"
"    --handle-size: 8vw;"
"    --circle-size: 8vw;"
"    --button-width: 16vw;"
"    --button-height: 4vh;"
"    --font-size: 2.2vw;"
"    --arm-disarm-font-size: 1.5vw;"
"  }"
"  body {"
"    margin: 0;"
"    overflow: hidden;"
"    display: flex;"
"    justify-content: center;"
"    align-items: center;"
"    height: 100vh;"
"    background-color: #0f0f20;"
"    font-family: Arial, sans-serif;"
"    color: #39ff14;"
"    font-weight: bold;"
"  }"
"  .joystick-container {"
"    display: flex;"
"    justify-content: space-between;"
"    width: 100%;"
"    height: 100%;"
"    align-items: center;"
"    padding: 5vw;"
"  }"
"  .joystick:first-child {"
"    margin-left: -2vw;"
"    margin-top: 14vh;"
"  }"
"  .joystick:last-child {"
"    margin-right: -2vw;"
"    margin-top: 14vh;"
"  }"
"  .joystick {"
"    width: var(--joystick-size);"
"    height: var(--joystick-size);"
"    background-color: rgba(255, 255, 255, 0.1);"
"    border-radius: 12%;"
"    position: relative;"
"    overflow: hidden;"
"    touch-action: none;"
"    border: 2px solid rgba(0, 255, 247, 0.3);"
"  }"
"  .handle {"
"    width: var(--handle-size);"
"    height: var(--handle-size);"
"    background-color: rgba(255, 255, 255, 0.2);"
"    border-radius: 50%;"
"    position: absolute;"
"    top: 50%;"
"    left: 50%;"
"    transform: translate(-50%, -50%);"
"  }"
"  .arm_circle,"
"  .disarm_circle {"
"    width: var(--circle-size);"
"    height: var(--circle-size);"
"    background-color: rgba(255, 255, 255, 0.2);"
"    border-radius: 50%;"
"    position: absolute;"
"    color: rgba(255, 255, 255, 0.8);"
"    text-align: center;"
"    line-height: var(--circle-size);"
"    display: flex;"
"    justify-content: center;"
"    align-items: center;"
"    border: 2px solid rgba(255, 255, 255, 0.3);"
"    font-size: var(--arm-disarm-font-size);"
"    transform: translate(-50%, -50%);"
"  }"
"  .arm_circle {"
"    top: 88.4%;"
"    left: 88.4%;"
"  }"
"  .disarm_circle {"
"    top: 88.4%;"
"    left: 11.5%;"
"  }"

"  #telemetry-display-row {"
"    position: absolute;"
"    top: 4%;"
"    left: 50%;"
"    transform: translateX(-50%);"
"    display: flex;"
"    justify-content: center;"
"    width: 100%;"
"    padding: 1vh 0;"
"  }"
"  .telemetry-row {"
"    color: #f0f0f0;"
"    background-color: rgba(255, 255, 255, 0.1);"
"    font-size: var(--font-size);"
"    margin: 0 1vw;"
"    padding: 1vh;"
"    text-align: center;"
"    border-radius: 10px;"
"    width: 20%;"
"  }"

"  #telemetry-display {"
"    color: #f0f0f0;"
"    position: absolute;"
"    top: 50%;"
"    left: 50%;"
"    transform: translate(-50%, -50%);"
"    display: flex;"
"    flex-direction: column;"
"    align-items: center;"
"    margin-top: 7vh;"
"  }"
"  .telemetry {"
"    background-color: rgba(255, 255, 255, 0.1);"
"    font-size: var(--font-size);"
"    margin: 3vh;"
"    padding: 1vh;"
"    width: 80%;"
"    text-align: center;"
"    border-radius: 10px;"
"  }"
  
"    #telemetry2 {"
"    margin-bottom: 6vh;"
"  }"
  
"  .button {"
"    align-items: center;"
"    background: #FFFFFF;"
"    border: 4px solid #E2E8F0;"
"    box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1),0 2px 4px -1px rgba(0, 0, 0, 0.06);"
"    box-sizing: border-box;"
"    color: #1A202C;"
"    display: inline-flex;"
"    font-family: Inter, sans-serif;"
"    font-size: var(--font-size);"
"    font-weight: 700;"
"    height: var(--button-height);"
"    justify-content: center;"
"    line-height: 24px;"
"    overflow-wrap: break-word;"
"    padding: 2vw;"
"    margin: 1.5vh;"
"    text-decoration: none;"
"    width: var(--button-width);"
"    border-radius: 10px;"
"    cursor: pointer;"
"    user-select: none;"
"    -webkit-user-select: none;"
"    touch-action: manipulation;"
"  }"

"  #landscape-warning {"
"    font-size: 7vw;"
"    display: none;"
"    position: absolute;"
"    top: 0;"
"    left: 0;"
"    width: 100%;"
"    height: 100%;"
"    background-color: rgba(0, 0, 0, 1.0);"
"    background: rgb(1,7,43);"
"    color: #f0f0f0;"
"    display: flex;"
"    justify-content: center;"
"    align-items: center;"
"    text-align: center;"
"    padding: 2vw;"
"    box-sizing: border-box;"
"  }"
"</style>"

"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"

"<script>"

"  var batt = 0;"
"  var rssi = 0;"
"  var pitch = 0;"
"  var roll = 0;"
"  var heading = 0;"
"  var altitude = 0;"
"  var is_headless_on = 0;"
"  var headless_button_value = 0;"
"  var is_flip_on = 0;"
"  var flip_button_value = 0;"
"  var is_alt_hold_on = 0;"
"  var alt_hold_button_value = 0;"
"  var joystickData = {"
"    1: { dx: 0, dy: 0 },"
"    2: { dx: 0, dy: 0 }"
"  };"


"  const wsUri = \"ws://192.168.4.1/ws\";"
"  let websocket = new WebSocket(wsUri);"
"  websocket.binaryType = \"arraybuffer\";"


"  websocket.onmessage = function(evt) {"

"      var buffer = evt.data;"
"      var view = new DataView(buffer);"
"      batt = view.getUint8(0, true) / 50.0;"
"      rssi = view.getInt8(1, true);"
"      pitch = view.getInt8(2, true) / 1.4;"
"      roll = view.getInt8(3, true) / 0.7;"
"      heading = view.getUint8(4, true) / 0.7;"
"      altitude = view.getInt8(5, true) / 10.0;"
"      is_headless_on = view.getUint8(6, true);"
"      is_flip_on = view.getUint8(7, true);"
"      is_alt_hold_on = view.getUint8(8, true);"
"      updateScoreDisplay();"

"  };"


"  function setupJoystick(joystickId, joystickNumber) {"
"    var joystick = document.getElementById(joystickId);"
"    var handle = joystick.querySelector('.handle');"
"    var joystickCenter = { x: joystick.offsetWidth / 2, y: joystick.offsetHeight / 2 };"
"    joystick.addEventListener('touchstart', handleTouch, false);"
"    joystick.addEventListener('touchmove', handleTouch, false);"
"    joystick.addEventListener('touchend', function(e) {"
"        handle.style.left = '50%';"
"        handle.style.top = '50%';"
"        joystickData[joystickNumber] = { dx: 0, dy: 0 };"
"    }, false);"
""
"    function handleTouch(e) {"
"        e.preventDefault();"
"        var touch = e.targetTouches[0];"
"        var rect = joystick.getBoundingClientRect();"
"        var dx = touch.clientX - rect.left - joystickCenter.x;"
"        var dy = touch.clientY - rect.top - joystickCenter.y;"
"        var x = Math.max(Math.min(joystickCenter.x + dx, joystick.offsetWidth - handle.offsetWidth / 2), handle.offsetWidth / 2);"
"        var y = Math.max(Math.min(joystickCenter.y + dy, joystick.offsetHeight - handle.offsetHeight / 2), handle.offsetHeight / 2);"
"        handle.style.left = x + 'px';"
"        handle.style.top = y + 'px';"
"        var x_min = handle.offsetWidth / 2;"
"        var x_max = joystick.offsetWidth - x_min;"
"        var y_min = handle.offsetHeight / 2;"
"        var y_max = joystick.offsetHeight - y_min;"
"        joystickData[joystickNumber] = { dx: mapValue(x, x_min, x_max, -120, 120), dy: mapValue(y, y_min, y_max, 120, -120) };"
"    }"
"}"


"  function mapValue(x, in_min, in_max, out_min, out_max) {"
"  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;"
"}"


"  function headless_func() {"
"    if (is_headless_on == 1){"
"        headless_button_value = 0;"
"    } else{"
"        headless_button_value = 1;"
"    }"
"    navigator.vibrate(200);"
"  }"

"  function flip_func() {"
"    if (flip_button_value == 1){"
"        flip_button_value = 0;"
"    } else{"
"        flip_button_value = 1;"
"    }"
"    navigator.vibrate(200);"
"  }"

"  function hold_func() {"
"    if (alt_hold_button_value == 1){"
"        alt_hold_button_value = 0;"
"    } else{"
"        alt_hold_button_value = 1;"
"    }"
"    navigator.vibrate(200);"
"  }"



"  function sendJoystickData() {"

"    var valueX1 = Math.round(joystickData[1].dx);"
"    var valueY1 = Math.round(joystickData[1].dy);"
"    var valueX2 = Math.round(joystickData[2].dx);"
"    var valueY2 = Math.round(joystickData[2].dy);"
"    var buffer = new ArrayBuffer(7);"
"    var view = new DataView(buffer);"
"    view.setInt8(0, valueX1);"
"    view.setInt8(1, valueY1);"
"    view.setInt8(2, valueX2);"
"    view.setInt8(3, valueY2);"
"    view.setUint8(4, headless_button_value, true);"
"    view.setUint8(5, flip_button_value, true);"
"    view.setUint8(6, alt_hold_button_value, true);"
"    websocket.send(buffer);"
"  }"


"  function checkOrientation() {"
"    var warning = document.getElementById('landscape-warning');"
"    if (window.innerHeight < window.innerWidth) {"
"      warning.style.display = 'none';"
"      if (!document.fullscreenElement) {"
"        document.documentElement.requestFullscreen();"
"      }"
"    } else {"
"      if (document.fullscreenElement) {"
"        document.exitFullscreen();"
"        warning.style.display = 'flex';"
"      }"
"    }"
"  }"



"  function updateScoreDisplay() {"
"    document.getElementById('telemetry1').textContent = 'Bat: ' + batt.toFixed(1) + ' V';"
"    document.getElementById('telemetry2').textContent = rssi.toFixed(0) + ' dBm';"

"    document.getElementById('telemetry3').textContent = 'Pitch: ' + pitch.toFixed(1) + '°';"
"    document.getElementById('telemetry4').textContent = 'Roll: ' + roll.toFixed(1) + '°';"
"    document.getElementById('telemetry5').textContent = 'Yaw: ' + heading.toFixed(1) + '°';"
"    document.getElementById('telemetry6').textContent = 'Altitude: ' + altitude.toFixed(1) + ' m';"

"    if (batt < 3.4) {"
"      document.getElementById('telemetry1').style.color = 'red';"
"    }"
"    else {"
"      document.getElementById('telemetry1').style.color = '#90EE90';"
"    }"
"    if (is_headless_on == 1) {"
"      document.getElementById('headless_button').style.background = '#90EE90';"
"    }"
"    else {"
"       document.getElementById('headless_button').style.background = '#FFFFFF';"
"    }"
"    if (is_flip_on == 1) {"
"      document.getElementById('flip_button').style.background = '#90EE90';"
"    }"
"    else {"
"       document.getElementById('flip_button').style.background = '#FFFFFF';"
"    }"
"    if (is_alt_hold_on == 1) {"
"      document.getElementById('alt_hold_button').style.background = '#90EE90';"
"    }"
"    else {"
"       document.getElementById('alt_hold_button').style.background = '#FFFFFF';"
"    }"

"  }"


"  window.addEventListener('load', function() {"
"    setupJoystick('joystick1', 1);"
"    setupJoystick('joystick2', 2);"
"    document.getElementById('headless_button').addEventListener('touchstart', headless_func);"
"    document.getElementById('flip_button').addEventListener('touchstart', flip_func);"
"    document.getElementById('alt_hold_button').addEventListener('touchstart', hold_func);"
"    checkOrientation();"
"    window.addEventListener('resize', checkOrientation);"
"    window.screen.orientation.addEventListener('change', checkOrientation);"
"    setInterval(sendJoystickData, 50);"
"    updateScoreDisplay();"
"  });"

"</script>"




"</head>"
"<body>"

"  <div id=\"telemetry-display-row\">"
"    <div class=\"telemetry-row\" id=\"telemetry3\">Veri 3</div>"
"    <div class=\"telemetry-row\" id=\"telemetry4\">Veri 4</div>"
"    <div class=\"telemetry-row\" id=\"telemetry5\">Veri 5</div>"
"    <div class=\"telemetry-row\" id=\"telemetry6\">Veri 6</div>"
    
"  </div>"

"  <div class=\"joystick-container\">"
"    <div id=\"joystick1\" class=\"joystick\">"
"      <div class=\"handle\"></div>"
"      <div class=\"arm_circle\">ARM</div>"
"      <div class=\"disarm_circle\">DISARM</div>"
"    </div>"

"    <div id=\"telemetry-display\">"
"       <div class=\"telemetry\" id=\"telemetry1\">Veri 1</div>"
"       <div class=\"telemetry\" id=\"telemetry2\">Veri 2</div>"
"       <div class=\"button\" id=\"headless_button\">Headless</div>"
"       <div class=\"button\" id=\"flip_button\">Flip</div>"
"       <div class=\"button\" id=\"alt_hold_button\">Hold</div>"
"     </div>"
     
"    <div id=\"joystick2\" class=\"joystick\">"
"      <div class=\"handle\"></div>"
"    </div>"
"  </div>"
"  <div id=\"landscape-warning\">"
"    Lütfen Cihazınızı Yatay Konuma Getirin"
"  </div>"

"</body>"
"</html>"
;



static esp_err_t send_recv_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) 
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    esp_err_t ret;
    static uint8_t buf[8];
    static size_t expected_lengh = 7;

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, expected_lengh);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }

    gamepad_ptr->analog_LX = (int8_t)(ws_pkt.payload[0]);
    gamepad_ptr->analog_LY = (int8_t)(ws_pkt.payload[1]);
    gamepad_ptr->analog_RX = (int8_t)(ws_pkt.payload[2]);
    gamepad_ptr->analog_RY = (int8_t)(ws_pkt.payload[3]);

    gamepad_ptr->button_A = (uint8_t)(ws_pkt.payload[4]); // headless_button
    gamepad_ptr->button_B = (uint8_t)(ws_pkt.payload[5]); // flip_button
    gamepad_ptr->button_X = (uint8_t)(ws_pkt.payload[6]); // alt_hold_button

    //printf("%d, %d, %d, %d, %.2f, %.2f\n", gamepad_ptr->analog_LX, gamepad_ptr->analog_LY, gamepad_ptr->analog_RX, gamepad_ptr->analog_RY, is_headless_on, is_flip_on);

    // Send telemetry

    static uint8_t buff[sizeof(telemetry_small_integer_t)];
    telemetry_small_integer_t t_int;

    t_int.battery_voltage = (uint8_t)(telem_small_p->battery_voltage * 50.0f);
    t_int.rssi = telem_small_p->rssi;
    t_int.pitch = (int8_t)(telem_small_p->pitch * 1.4f);
    t_int.roll = (int8_t)(telem_small_p->roll * 0.7f);
    t_int.heading = (uint8_t)(telem_small_p->heading * 0.7f);
    t_int.altitude = (int8_t)(telem_small_p->altitude * 10.0f);

    t_int.is_flip_on = telem_small_p->is_flip_on;
    t_int.is_headless_on = telem_small_p->is_headless_on;
    t_int.is_alt_hold_on = telem_small_p->is_alt_hold_on;

    memcpy(buff, &t_int, sizeof(telemetry_small_integer_t));

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    ws_pkt.payload = buff;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.len = sizeof(telemetry_small_integer_t);

    ret = httpd_ws_send_frame(req, &ws_pkt);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    }

    return ret;
}



esp_err_t get_handler(httpd_req_t *req) 
{
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


static const httpd_uri_t ws = 
{
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = send_recv_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

static const httpd_uri_t uri_get = 
{
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = get_handler,
    .user_ctx  = NULL
};


static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws);
        httpd_register_uri_handler(server, &uri_get);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}


void web_comm_init(gamepad_t *gmp, telemetry_small_t *tels)
{
    gamepad_ptr = gmp;
    telem_small_p = tels;

    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "IKARUS",
            .ssid_len = strlen("IKARUS"),
            .channel = 1,
            .password = "",
            .max_connection = 1,
            .authmode = WIFI_AUTH_OPEN
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_handler, &server));

    server = start_webserver();
}
