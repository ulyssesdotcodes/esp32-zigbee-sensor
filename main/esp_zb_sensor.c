/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_humidity_sensor Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_sensor.h"
#include "sensor_driver.c"
#include "switch_driver.h"
#include "esp_sleep.h"
#include "esp_zigbee_trace.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "time.h"
#include "sys/time.h"


#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

extern void zb_set_ed_node_descriptor(bool power_src, bool rx_on_when_idle, bool alloc_addr);

static RTC_DATA_ATTR struct timeval s_sleep_enter_time;
static esp_timer_handle_t s_oneshot_timer;


static int16_t zb_humidity_to_s16(float humidity)
{
    return (int16_t)(humidity * 10000);
}

static void esp_app_sensor_handler(float humidity, float battery_percentage)
{
    ESP_LOGI(TAG, "Callback from handler");
    int16_t measured_value = zb_humidity_to_s16(humidity);
    int16_t battery_value = (int16_t) trunc(battery_percentage * 100);
    ESP_LOGI(TAG, "Reporting %d battery %d", measured_value, battery_value);
    /* Update humidity sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);

    ESP_LOGI(TAG, "lock acquired - setting attribute");
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &measured_value, false);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &battery_value, false);
    ESP_LOGI(TAG, "lock releasing");
    esp_zb_lock_release();


    /* Send report attributes command */
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

    esp_zb_zcl_report_attr_cmd_t report_attr_batt_cmd = {0};
    report_attr_batt_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_batt_cmd.attributeID = ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
    report_attr_batt_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_batt_cmd.clusterID = ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    report_attr_batt_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_batt_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");

}

static void s_oneshot_timer_callback(void* arg)
{
    /* Enter deep sleep */
    ESP_LOGI(TAG, "Enter deep sleep");
    gettimeofday(&s_sleep_enter_time, NULL);
    esp_deep_sleep_start();
}

static void zb_deep_sleep_init(void)
{
    /* Within this function, we print the reason for the wake-up and configure the method of waking up from deep sleep.
    This example provides support for two wake-up sources from deep sleep: RTC timer and GPIO. */

    /* The one-shot timer will start when the device transitions to the CHILD state for the first time.
    After a 5-second delay, the device will enter deep sleep. */

    const esp_timer_create_args_t s_oneshot_timer_args = {
            .callback = &s_oneshot_timer_callback,
            .name = "one-shot"
    };

    ESP_ERROR_CHECK(esp_timer_create(&s_oneshot_timer_args, &s_oneshot_timer));


    /* Set the methods of how to wake up: */
    /* 1. RTC timer waking-up */
    const int wakeup_time_sec = 30;
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

static void zb_deep_sleep_start(void)
{
    // Print the wake-up reason:
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - s_sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - s_sleep_enter_time.tv_usec) / 1000;
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause) {
    case ESP_SLEEP_WAKEUP_TIMER: {
        ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        ESP_LOGI(TAG, "Not a deep sleep reset");
        break;
    }

    /* Start the one-shot timer */
    const int before_deep_sleep_time_sec = 10;
    ESP_LOGI(TAG, "Start one-shot timer for %ds to enter the deep sleep", before_deep_sleep_time_sec);
    ESP_ERROR_CHECK(esp_timer_start_once(s_oneshot_timer, before_deep_sleep_time_sec * 1000000));
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    ESP_RETURN_ON_ERROR(sensor_init(esp_app_sensor_handler), TAG, "Fail to initialize sensor");
    return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                zb_deep_sleep_start();
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zb_deep_sleep_start();
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        // ESP_LOGI(TAG, "Zigbee can sleep");
        // esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

static esp_zb_cluster_list_t *custom_humidity_sensor_clusters_create(esp_zb_humidity_sensor_cfg_t *humidity_sensor, float battery_percentage)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(humidity_sensor->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(humidity_sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(&(humidity_sensor->humidity_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    uint8_t bat_quantity = 1;
    uint8_t bat_size = 1;
    uint16_t bat_voltage = 37;
    uint16_t battery_percentage_default = 95;
    esp_zb_attribute_list_t *power_config_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    ESP_ERROR_CHECK(
            esp_zb_power_config_cluster_add_attr(power_config_cluster,
                                                 ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
                                                 &bat_voltage));
    ESP_ERROR_CHECK(
            esp_zb_power_config_cluster_add_attr(power_config_cluster,
                                                 ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID,
                                                 &bat_size));
    ESP_ERROR_CHECK(
            esp_zb_power_config_cluster_add_attr(power_config_cluster,
                                                 ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID,
                                                 &bat_quantity));
    ESP_ERROR_CHECK(
            esp_zb_power_config_cluster_add_attr(power_config_cluster,
                                                 ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                                 &battery_percentage_default));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(cluster_list, power_config_cluster,
                                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_ep_list_t *custom_humidity_sensor_ep_create(uint8_t endpoint_id, esp_zb_humidity_sensor_cfg_t *humidity_sensor, float battery_percentage)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_humidity_sensor_clusters_create(humidity_sensor, battery_percentage), endpoint_config);
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    ESP_EARLY_LOGI(TAG, "start zigbee stack");
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
        // esp_zb_sleep_enable(true);
    esp_zb_init(&zb_nwk_cfg);
    

    ESP_EARLY_LOGI(TAG, "humidity sensor");
    /* Create customized humidity sensor endpoint */
    esp_zb_humidity_sensor_cfg_t sensor_cfg = ESP_ZB_DEFAULT_HUMIDITY_SENSOR_CONFIG();
    /* Set (Min|Max)MeasuredValure */
    sensor_cfg.humidity_meas_cfg.min_value = zb_humidity_to_s16(ESP_HUMIDITY_SENSOR_MIN_VALUE);
    sensor_cfg.humidity_meas_cfg.max_value = zb_humidity_to_s16(ESP_HUMIDITY_SENSOR_MAX_VALUE);
    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_humidity_sensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &sensor_cfg, 200.0);

    ESP_EARLY_LOGI(TAG, "register device");
    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    ESP_EARLY_LOGI(TAG, "reporting info");
    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 10,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 10,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    ESP_EARLY_LOGI(TAG, "update reporting info");
    esp_zb_zcl_update_reporting_info(&reporting_info);

    // esp_zb_zcl_reporting_info_t battery_reporting_info = {
    //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
    //     .ep = HA_ESP_SENSOR_ENDPOINT,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .attr_id = ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
    //     .u.send_info.min_interval = 10,
    //     .u.send_info.max_interval = 0,
    //     .u.send_info.delta.u8 = 0x00,
    //     .u.send_info.reported_value.u8 = 0,
    //     .u.send_info.def_min_interval = 10,
    //     .u.send_info.def_max_interval = 0,
    // };
    // esp_zb_zcl_update_reporting_info(&battery_reporting_info);

    // esp_zb_core_action_handler_register(esp_zb_app_signal_handler);

    ESP_EARLY_LOGI(TAG, "network channel");
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    // esp_zb_set_rx_on_when_idle(false);
    ESP_ERROR_CHECK(esp_zb_start(false));

    zb_set_ed_node_descriptor(0,0,0);

    ESP_EARLY_LOGI(TAG, "main loop");
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_zb_power_save_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    zb_deep_sleep_init();

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}