#include "esp_zigbee_core.h"
#include "zboss_api.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT          3      /* esp humidity sensor device endpoint, used for humidity measurement */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

#define ESP_TEMP_SENSOR_UPDATE_INTERVAL (1)     /* Local sensor update interval (second) */
#define ESP_HUMIDITY_SENSOR_MIN_VALUE       (0)   /* Local sensor min measured value (degree Celsius) */
#define ESP_HUMIDITY_SENSOR_MAX_VALUE       (100)    /* Local sensor max measured value (degree Celsius) */

/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER                "\x07"CONFIG_IDF_TARGET


typedef struct esp_zb_humidity_sensor_cfg_s {
    esp_zb_basic_cluster_cfg_t basic_cfg;       /*!<  Basic cluster configuration, @ref esp_zb_basic_cluster_cfg_s */
    esp_zb_identify_cluster_cfg_t identify_cfg; /*!<  Identify cluster configuration, @ref esp_zb_identify_cluster_cfg_s */
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg; /*!<  Identify cluster configuration, @ref esp_zb_identify_cluster_cfg_s */
} esp_zb_humidity_sensor_cfg_t;
/**
 * @brief Zigbee HA standard humidity sensor device default config value.
 *
 */
#define ESP_ZB_DEFAULT_HUMIDITY_SENSOR_CONFIG()                                                  \
    {                                                                                               \
        .basic_cfg =                                                                                \
            {                                                                                       \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,                        \
            },                                                                                      \
        .identify_cfg =                                                                             \
            {                                                                                       \
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   \
            },                                                                                      \
        .humidity_meas_cfg =                                                                            \
            {                                                                                       \
                .measured_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT, \
                .min_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT,                \
                .max_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT,                \
            }                                                                                      \
    }



/**
 * @brief  Create a standard HA humidity sensor cluster list.
 *
 * @note This contains basic, identify cluster and humidity measurement as server side. Identify cluster as client side.
 * @param[in] humidity_sensor  Configuration parameters for this cluster lists defined by @ref esp_zb_humidity_sensor_cfg_s
 *
 * @return Pointer to cluster list  @ref esp_zb_cluster_list_s
 *
 */
esp_zb_cluster_list_t  *esp_zb_humidity_sensor_clusters_create(esp_zb_humidity_sensor_cfg_t *humidity_sensor, float battery_percentage);

/**
 * @brief  Create a standard single HA humidity sensor endpoint.
 *
 * @param[in] endpoint_id The specific endpoint
 * @param[in] humidity_sensor  Configuration parameters for this cluster lists defined by @ref esp_zb_humidity_sensor_cfg_s
 *
 * @note This function adds a single endpoint to a cluster list.
 *
 * @return Pointer to esp_zb_ep_list_t @ref esp_zb_ep_list_s
 *
 */
esp_zb_ep_list_t *esp_zb_humidity_sensor_ep_create(uint8_t endpoint_id, esp_zb_humidity_sensor_cfg_t *humidity_sensor, float battery_percentage);

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }