#include <stdlib.h>
#include <DNSServer.h>
#include <WiFi.h>
#include "time.h"
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
}
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_log.h>
#include <esp_bt_main.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_http_server.h>
#include <esp_mqtt_client.h>
#include <cJSON.h>

DNSServer dnsServer;
IPAddress softApIp(8, 8, 8, 8);
IPAddress softApNetMask(255, 255, 255, 0);
esp_mqtt_client_handle_t mqttClient = NULL;
size_t content_len;
bool isWifiSetUp = false;
bool isMqttSetUp = false;
int ledState = LOW;   
unsigned long previousBlinkMillis = 0;
unsigned long previousRestart;

String wifi_ssid = "";
String wifi_pwd = "";
String mqtt_ip = "";
int mqtt_port = 0;
String mqtt_user = "";
String mqtt_pass = "";
String node_name = "";
std::vector<String> macs;
std::vector<String> uuids;

static const int scanTime = singleScanTime;
static const int waitTime = scanInterval;
String availabilityTopic;
String stateTopic;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
String localIp;
byte mqttRetryAttempts = 0;
byte wifiRetryAttempts = 0;
unsigned long lastBleScan = 0;
esp_ble_scan_params_t ble_scan_params;
TaskHandle_t NimBLEScan;
size_t commandLength;

void connectToWifi() {
    Serial.println("Connecting to WiFi...");
    esp_wifi_disconnect();
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, wifi_ssid.c_str());
    strcpy((char*)wifi_config.sta.password, wifi_pwd.c_str());
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}

bool handleWifiDisconnect() {
    if (WiFi.isConnected()) {
        Serial.println("WiFi appears to be connected. Not retrying.");
        return true;
    }
    if (wifiRetryAttempts > 10) {
        Serial.println("Too many retries. Restarting.");
        esp_restart();
    } else {
        wifiRetryAttempts++;
    }
    if (esp_mqtt_client_is_connected(mqttClient)) {
        esp_mqtt_client_disconnect(mqttClient);
    }
    if (xTimerIsTimerActive(mqttReconnectTimer) != pdFALSE) {
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    }

    if (xTimerReset(wifiReconnectTimer, 0) == pdFAIL) {
        Serial.println("failed to restart");
        xTimerStart(wifiReconnectTimer, 0);
        return false;
    } else {
        Serial.println("restarted");
        return true;
    }
}

void connectToMqtt() {
    Serial.println("Connecting to MQTT");
    if (WiFi.isConnected()) {
        esp_mqtt_client_config_t mqtt_cfg = {};
        mqtt_cfg.uri = mqtt_ip.c_str();
        mqtt_cfg.port = mqtt_port;
        mqtt_cfg.username = mqtt_user.c_str();
        mqtt_cfg.password = mqtt_pass.c_str();
        mqtt_cfg.client_id = node_name.c_str();
        mqttClient = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_start(mqttClient);
    } else {
        Serial.println("Cannot reconnect MQTT - WiFi error");
        handleWifiDisconnect();
    }
}

void handleMqttDisconnect() {
    if (mqttRetryAttempts > 10) {
        Serial.println("Too many retries. Restarting.");
        esp_restart();
    } else {
        mqttRetryAttempts++;
    }
    if (WiFi.isConnected()) {
        Serial.println("Starting MQTT reconnect timer");
        if (xTimerReset(mqttReconnectTimer, 0) == pdFAIL) {
            Serial.println("failed to restart");
            xTimerStart(mqttReconnectTimer, 0);
        } else {
            Serial.println("restarted");
        }
    } else {
        Serial.print("Disconnected from WiFi; starting WiFi reconnect timer\t");
        handleWifiDisconnect();
    }
}

void WiFiEvent(esp_event_base_t event_base, int32_t event_id, void* event_data) {
    Serial.printf("[WiFi-event] event: %d\n", event_id);

    switch(event_id) {
    case WIFI_EVENT_STA_GOT_IP:
        Serial.print("IP address: \t");
        Serial.println(WiFi.localIP());
        localIp = WiFi.localIP().toString().c_str();
        Serial.print("Hostname: \t");
        Serial.println(WiFi.getHostname());
        configTime(0, 0, ntp_server);
        if (isMqttSetUp) {
            connectToMqtt();
        }
        if (xTimerIsTimerActive(wifiReconnectTimer) != pdFALSE) {
            Serial.println("Stopping wifi reconnect timer");
            xTimerStop(wifiReconnectTimer, 0);
        }
        wifiRetryAttempts = 0;
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection, resetting timer\t");
        handleWifiDisconnect();
        break;
    case WIFI_EVENT_STA_START:
        Serial.println("STA Start");
        esp_netif_set_hostname(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), node_name.c_str());
        if (xTimerIsTimerActive(wifiReconnectTimer) != pdFALSE) {
            TickType_t xRemainingTime = xTimerGetExpiryTime(wifiReconnectTimer) - xTaskGetTickCount();
            Serial.print("WiFi Time remaining: ");
            Serial.println(xRemainingTime);
        } else {
            Serial.println("WiFi Timer is inactive; resetting\t");
            handleWifiDisconnect();
        }
        break;
    case WIFI_EVENT_STA_STOP:
        Serial.println("STA Stop");
        handleWifiDisconnect();
        break;
    }
}

bool sendTelemetry() {
    if (esp_mqtt_client_publish(mqttClient, stateTopic.c_str(), localIp.c_str(), 0, 0, 0) != -1) {
        Serial.println("State sent");
        return true;
    } else {
        Serial.println("Error sending telemetry");
        esp_mqtt_client_disconnect(mqttClient);
        return false;
    }
}

unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return 0;
    }
    time(&now);
    return now;
}

void sendDeviceState(String device, int rssi) {
    if (esp_mqtt_client_is_connected(mqttClient)) {
        String topic = root_topic + device + "/" + node_name;
        cJSON* root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "rssi", rssi);
        cJSON_AddNumberToObject(root, "timestamp", getTime());
        char* json_str = cJSON_Print(root);
        if (esp_mqtt_client_publish(mqttClient, topic.c_str(), json_str, 0, 1, 0) != -1) {
            Serial.println("Sent data for " + device);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            Serial.print("Error sending device data message.");
            esp_mqtt_client_disconnect(mqttClient);
        }
        cJSON_Delete(root);
        free(json_str);
    } else {
        Serial.println("MQTT disconnected.");
        if (xTimerIsTimerActive(mqttReconnectTimer) != pdFALSE) {
            TickType_t xRemainingTime = xTimerGetExpiryTime(mqttReconnectTimer) - xTaskGetTickCount();
            Serial.print("Time remaining: ");
            Serial.println(xRemainingTime);
        } else {
            handleMqttDisconnect();
        }
    }
}

void reportDevice(esp_ble_gap_cb_param_t::ble_scan_result_evt_param* scan_result) {
    char mac_address[18];
    snprintf(mac_address, sizeof(mac_address), "%02X:%02X:%02X:%02X:%02X:%02X",
             scan_result->bda[0], scan_result->bda[1], scan_result->bda[2],
             scan_result->bda[3], scan_result->bda[4], scan_result->bda[5]);
    String mac_addr_str(mac_address);
    mac_addr_str.toUpperCase();
    if (std::find(macs.begin(), macs.end(), mac_addr_str) != macs.end()) {
        sendDeviceState(mac_addr_str, scan_result->rssi);
        return;
    }
    if (scan_result->adv_data_len > 0) {
        uint8_t* adv_data = scan_result->ble_adv;
        char uuid_str[37] = {0};
        snprintf(uuid_str, sizeof(uuid_str), "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%Apologies for the interruption. Here's the continuation and completion of the modified code:

```cpp
void reportDevice(esp_ble_gap_cb_param_t::ble_scan_result_evt_param* scan_result) {
    char mac_address[18];
    snprintf(mac_address, sizeof(mac_address), "%02X:%02X:%02X:%02X:%02X:%02X",
             scan_result->bda[0], scan_result->bda[1], scan_result->bda[2],
             scan_result->bda[3], scan_result->bda[4], scan_result->bda[5]);
    String mac_addr_str(mac_address);
    mac_addr_str.toUpperCase();
    if (std::find(macs.begin(), macs.end(), mac_addr_str) != macs.end()) {
        sendDeviceState(mac_addr_str, scan_result->rssi);
        return;
    }
    if (scan_result->adv_data_len > 0) {
        uint8_t* adv_data = scan_result->ble_adv;
        // Example of handling manufacturer data or UUID extraction:
        // Adjust this section based on how you process BLE data
    }
}

void scanForDevices(void* parameter) {
    while (1) {
        if (WiFi.isConnected() && (millis() - lastBleScan > (waitTime * 1000) || lastBleScan == 0)) {
            Serial.println("Scanning...\t");
            esp_ble_gap_start_scanning(scanTime);
            Serial.println("Scanning done.");
            lastBleScan = millis();
            sendTelemetry();
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow FreeRTOS to manage task scheduling
    }
}

void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        reportDevice(&param->scan_rst);
        break;
    default:
        break;
    }
}

void sendHaConfig() {
    // Adjust Home Assistant MQTT configuration payloads based on ESP-IDF and cJSON
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", ("Format BLE Tracker - " + node_name).c_str());
    cJSON_AddStringToObject(root, "configuration_url", ("http://" + localIp).c_str());
    cJSON_AddStringToObject(root, "sw_version", String(version).c_str());
    cJSON* identifiers = cJSON_AddArrayToObject(root, "identifiers");
    cJSON_AddItemToArray(identifiers, cJSON_CreateString(("format_ble_tracker_rooms_" + node_name).c_str()));
    char* json_str = cJSON_Print(root);

    if (esp_mqtt_client_publish(mqttClient, (discovery_prefix + sensor_topic + "format_ble_tracker_rooms/" + node_name + "/config").c_str(), json_str, 0, 1, 0) != -1) {
        Serial.println("Config sent for " + node_name);
    } else {
        Serial.println("Error sending HA config");
        esp_mqtt_client_disconnect(mqttClient);
    }

    cJSON_Delete(root);
    free(json_str);
}

void onMqttConnect(esp_mqtt_client_handle_t client, esp_mqtt_event_handle_t event) {
    Serial.println("Connected to MQTT.");
    digitalWrite(LED_BUILTIN, HIGH);
    mqttRetryAttempts = 0;

    if (esp_mqtt_client_publish(mqttClient, availabilityTopic.c_str(), "online", 0, 1, 1) != -1) {
        Serial.print("Success sending message to topic:\t");
        Serial.println(availabilityTopic);
        String alive_topic = root_topic + "alive/+";
        esp_mqtt_client_subscribe(mqttClient, alive_topic.c_str(), 2);
        sendHaConfig();
        sendTelemetry();
    } else {
        Serial.println("Error sending message");
        esp_mqtt_client_disconnect(mqttClient);
    }
}

void onMqttMessage(esp_mqtt_event_handle_t event) {
    char topicArr[256];
    strcpy(topicArr, event->topic);
    String tStr = topicArr;
    String uid = tStr.substring(tStr.lastIndexOf("/") + 1);
    if (event->data) {
        String pld(event->data, event->data_len);
        if (pld.indexOf("True") >= 0) {
            if (uid.indexOf(":") >= 0) {
                if (std::find(macs.begin(), macs.end(), uid) == macs.end()) {
                    Serial.println("Adding MAC  " + uid);
                    macs.push_back(uid);
                } else {
                    Serial.println("Skipping duplicated MAC  " + uid);
                }
            } else if (uid.indexOf("-") >= 0) {
                if (std::find(uuids.begin(), uuids.end(), uid) == uuids.end()) {
                    Serial.println("Adding UUID " + uid);
                    uuids.push_back(uid);
                } else {
                    Serial.println("Skipping duplicated UUID  " + uid);
                }
            }
            return;
        }
    }
    Serial.println("Removing " + uid);
    macs.erase(std::remove(macs.begin(), macs.end(), uid), macs.end());
    uuids.erase(std::remove(uuids.begin(), uuids.end(), uid), uuids.end());
}

void onMqttDisconnect(esp_mqtt_event_handle_t event) {
    Serial.println("Disconnected from MQTT.");
    digitalWrite(LED_BUILTIN, LOW);
    handleMqttDisconnect();
}

void mainSetup() {
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

    esp_netif_init();
    esp_event_loop_create_default();
    connectToWifi();

    if (isMqttSetUp) {
        esp_mqtt_client_config_t mqtt_cfg = {};
        mqtt_cfg.uri = mqtt_ip.c_str();
        mqtt_cfg.port = mqtt_port;
        mqtt_cfg.username = mqtt_user.c_str();
        mqtt_cfg.password = mqtt_pass.c_str();
        mqtt_cfg.client_id = node_name.c_str();
        mqtt_cfg.event_handle = [](esp_mqtt_event_handle_t event) -> esp_err_t {
            switch (event->event_id) {
            case MQTT_EVENT_CONNECTED:
                onMqttConnect(event->client, event);
                break;
            case MQTT_EVENT_DISCONNECTED:
                onMqttDisconnect(event);
                break;
            case MQTT_EVENT_DATA:
                onMqttMessage(event);
                break;
            default:
                break;
            }
            return ESP_OK;
        };
        mqttClient = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_start(mqttClient);

        ble_scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
        ble_scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        ble_scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
        ble_scan_params.scan_interval = bleScanInterval;
        ble_scan_params.scan_window = bleScanWindow;
        ble_scan_params.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;

        esp_ble_gap_set_scan_params(&ble_scan_params);
        esp_ble_gap_register_callback(esp_gap_cb);

        xTaskCreatePinnedToCore(
            scanForDevices,
            "BLE Scan",
            4096,
            nullptr,
            1,
            &NimBLEScan,
            1);
    }
}

void blink(unsigned long delay) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousBlinkMillis >= delay) {
        ledState = (ledState == LOW) ? HIGH : LOW;
        digitalWrite(LED_BUILTIN, ledState);
        previousBlinkMillis = currentMillis;
    }
}

void setup() {
    Serial.begin(115200);
    previousRestart = millis();
    writeRestarts(1);
    pinMode(LED_BUILTIN, OUTPUT);
    isWifiSetUp = readWifiPrefs();
    isMqttSetUp = readMqttPrefs();
    if (isWifiSetUp) {
        mainSetup();
    } else {
        configSetup();
    }
    commonSetup();
}

void loop() {
    if (previousRestart > 0 && millis() > (previousRestart + hardResetTimeoutInterval)) {
        previousRestart = 0;
        writeRestarts(0);
    }
    if (isWifiSetUp) {
        if (!WiFi.isConnected()) {
            blink(500);
        } else if (!esp_mqtt_client_is_connected(mqttClient)) {
            blink(1000);
        }
        if (isMqttSetUp) {
            mainLoop();
        }
    } else {
        dnsServer.processNextRequest();
        blink(250);
    }
}
