////////////////////////////////////////////////////////////////////////
// SPDX-License-Identifier: AGPL-3.0-only
// (C) Copyright 2022-2024 - formatBCE

#include "esp32_ble_presense.h"

#include "esphome/core/log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

#define bleScanInterval 0x80 // Adjusted for ESP-IDF
#define bleScanWindow 0x40    // Adjusted for ESP-IDF

// Undo the default loose definitions in our file only
#pragma GCC diagnostic error "-Wdeprecated-declarations"
#pragma GCC diagnostic error "-Wsign-compare"
#pragma GCC diagnostic error "-Wunused-but-set-variable"
#pragma GCC diagnostic error "-Wunused-function"
#pragma GCC diagnostic error "-Wunused-parameter"
#pragma GCC diagnostic error "-Wunused-variable"
#pragma GCC diagnostic error "-Wformat"

using namespace esphome;

namespace ESP32_BLE_Presense {

static esp_ble_scan_params_t ble_scan_params;

class BleAdvertisedDeviceCallbacks {

    ESP32_BLE_Presense& parent_;

public:
    BleAdvertisedDeviceCallbacks(ESP32_BLE_Presense& parent) : parent_(parent) {}

    void onResult(esp_ble_gap_cb_param_t::ble_scan_result_evt_param *scan_result) {
        if (!scan_result)
            return;

        std::string mac_address = esp_ble_gap_get_static_rnd_addr(scan_result->bda);
        parent_.reportDevice(mac_address, scan_result->rssi, std::string((char*)scan_result->ble_adv, scan_result->adv_data_len));
    }
};

static std::string capitalizeString(const std::string& s) {
    std::string ret;
    ret.reserve(s.size());

    std::transform(s.begin(), s.end(), std::back_inserter(ret),
                   [](unsigned char c){ return std::toupper(c); });
    return ret;
}

ESP32_BLE_Presense::ESP32_BLE_Presense()
:  esphome::PollingComponent(5000) {
}

void ESP32_BLE_Presense::update() {
    if (!esp_ble_gap_start_scanning(0)) {
        ESP_LOGD("format_ble", "Start scanning...");
    }
    ESP_LOGD("format_ble", "BLE scan heartbeat");
}

void ESP32_BLE_Presense::setup() {
    PollingComponent::setup();

    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);

    ble_scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
    ble_scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    ble_scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
    ble_scan_params.scan_interval = bleScanInterval;
    ble_scan_params.scan_window = bleScanWindow;
    ble_scan_params.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;

    esp_ble_gap_set_scan_params(&ble_scan_params);
    esp_ble_gap_register_callback(esp_gap_cb);

    subscribe("format_ble_tracker/alive/+", &ESP32_BLE_Presense::on_alive_message);
}

void ESP32_BLE_Presense::reportDevice(const std::string& macAddress,
                                    int rssi,
                                    const std::string& manufacturerData) {

    std::string mac_address = capitalizeString(macAddress);
    time_t time = rtc->timestamp_now();
    if (std::find(macs.begin(), macs.end(), mac_address) != macs.end()) {
        ESP_LOGD("format_ble", "Sending for '%s': %ddBm", mac_address.c_str(), rssi);
        publish_json("format_ble_tracker/" + mac_address + "/" + name, [=](JsonObject root) {
            root["rssi"] = rssi;
            root["timestamp"] = time;
        }, 1, true);
        return;
    }

    static const size_t UUID_INDEX = 4;
    static const size_t UUID_LEN = 16;
    if (manufacturerData.length() >= UUID_INDEX + UUID_LEN) {
        std::string uuid_str = capitalizeString(NimBLEUUID(reinterpret_cast<const uint8_t*>(&manufacturerData[UUID_INDEX]),
                                                           UUID_LEN, true).toString());
        if (std::find(uuids.begin(), uuids.end(), uuid_str) != uuids.end()) {
            ESP_LOGD("format_ble", "Sending for '%s': %ddBm", uuid_str.c_str(), rssi);
            publish_json("format_ble_tracker/" + uuid_str + "/" + name, [=](JsonObject root) {
                root["rssi"] = rssi;
                root["timestamp"] = time;
            }, 1, true);
            return;
        }
    }
}

void ESP32_BLE_Presense::on_alive_message(const std::string &topic, const std::string &payload) {
    std::string uid = capitalizeString(topic.substr(topic.find_last_of("/") + 1));
    if (payload == "True") {
        if (uid.rfind(":") != std::string::npos) {
            if (std::find(macs.begin(), macs.end(), uid) == macs.end()) {
                ESP_LOGD("format_ble", "Adding MAC  %s", uid.c_str());
                macs.push_back(uid);
            } else {
                ESP_LOGD("format_ble", "Skipping duplicated MAC  %s", uid.c_str());
            }
        } else if (uid.rfind("-") != std::string::npos) {
            if (std::find(uuids.begin(), uuids.end(), uid) == uuids.end()) {
                ESP_LOGD("format_ble", "Adding UUID %s", uid.c_str());
                uuids.push_back(uid);
            } else {
                ESP_LOGD("format_ble", "Skipping duplicated UUID  %s", + uid.c_str());
            }
        }
        return;
    } else {
        ESP_LOGD("format_ble", "Removing %s", uid.c_str());
        macs.erase(std::remove(macs.begin(), macs.end(), uid), macs.end());
        uuids.erase(std::remove(uuids.begin(), uuids.end(), uid), uuids.end());
    }
}

}
