#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/param.h>
#include <time.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "mpu6050.h"


// Configuration
#define SDA_GPIO              21
#define SCL_GPIO              22
#define MPU1_ADDR             0x68
#define MPU2_ADDR             0x69
#define SAMPLE_RATE_HZ        200
#define MQTT_PUB_INTERVAL_MS  100

static const char *TAG = "MPU6050_MQTT";

// MQTT Configuration
#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipseprojects_io_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
#endif
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

// Global handles
static i2c_master_bus_handle_t bus_handle;
static mpu6050_handle_pair_t mpu_pair;
static dual_acce_value_t dual_acce;
static dual_gyro_value_t dual_gyro;
static esp_mqtt_client_handle_t mqtt_client;
static volatile bool mqtt_connected = false;

// WiFi event handler
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  static int retry_count = 0;

  switch (event_id)
  {
    case WIFI_EVENT_STA_START:
      printf("WiFi connecting ... \n");
      break;
    case WIFI_EVENT_STA_CONNECTED:
      printf("WiFi connected ... \n");
      retry_count = 0; // Reset retry count on successful connection
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      printf("WiFi lost connection ... \n");
      if (retry_count < 5) {
        esp_wifi_connect();
        retry_count++;
        ESP_LOGI(TAG, "Retrying Wi-Fi connection (%d/5)", retry_count);
      } else {
        ESP_LOGE(TAG, "Max Wi-Fi retries reached");
      }
      break;
    case IP_EVENT_STA_GOT_IP:
      printf("WiFi got IP ... \n\n");
      break;
    default:
      break;
  }
}

// Initialize WiFi
static void wifi_init(void) {
  // 1 - WiFi/LwIP Init Phase
  ESP_ERROR_CHECK(esp_netif_init()); // TCP/IP Initialization
  ESP_ERROR_CHECK(esp_event_loop_create_default()); // event loop
  esp_netif_create_default_wifi_sta(); // WiFi station
  wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_initiation));

  // 2 - WiFi configuration phase
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
  wifi_config_t wifi_configuration = {
    .sta = {
      .ssid = CONFIG_WIFI_SSID,
      .password = CONFIG_WIFI_PASSWORD,
    }
  };
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration));

  // 3 - WiFi start phase
  ESP_ERROR_CHECK(esp_wifi_start());

  // 4 - WiFi Connect Phase
  esp_wifi_connect();
}

// Publish MPU6050 data through MQTT
void mqtt_publish_task(void *pvParameters) {
  while (1) {
    if (mqtt_connected) { // Only publish when connected
      esp_err_t ret = mpu6050_get_dual_data(mpu_pair, &dual_acce, &dual_gyro);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data: %s", esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(100)); // Retry after delay
        continue;
      }

      // Get HHMMSSmmm timestamp
      time_t now = time(NULL);
      struct tm *tm_info = localtime(&now);

      int64_t micros_since_boot = esp_timer_get_time();
      int millis = micros_since_boot % 1000000 / 1000;  // Get ms within the current second

      char timestamp[11];  // HHMMSSmmm + null terminator

      // Format the timestamp directly into the buffer
      snprintf(timestamp, sizeof(timestamp), "%02d%02d%02d%03d", tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, millis);
      char payload[512];
      snprintf(payload, sizeof(payload) - 1,
               "{\"time\":\"%s\","
               "\"ax1\":%.2f,\"ay1\":%.2f,\"az1\":%.2f,"
               "\"gx1\":%.2f,\"gy1\":%.2f,\"gz1\":%.2f,"
               "\"ax2\":%.2f,\"ay2\":%.2f,\"az2\":%.2f,"
               "\"gx2\":%.2f,\"gy2\":%.2f,\"gz2\":%.2f}",
               timestamp,
               dual_acce.acce_1.acce_x, dual_acce.acce_1.acce_y, dual_acce.acce_1.acce_z,
               dual_gyro.gyro_1.gyro_x, dual_gyro.gyro_1.gyro_y, dual_gyro.gyro_1.gyro_z,
               dual_acce.acce_2.acce_x, dual_acce.acce_2.acce_y, dual_acce.acce_2.acce_z,
               dual_gyro.gyro_2.gyro_x, dual_gyro.gyro_2.gyro_y, dual_gyro.gyro_2.gyro_z);
      payload[sizeof(payload) - 1] = '\0';

      int msg_id = esp_mqtt_client_publish(mqtt_client, "sensors1635/mpu6050", payload, 0, 1, 0);
      if (msg_id >= 0) {
        ESP_LOGI(TAG, "Published successfully: %s", payload);
      } else {
        ESP_LOGE(TAG, "Failed to publish MQTT message");
      }
    } else {
      ESP_LOGW(TAG, "MQTT client not connected");
    }
  }
}


// MQTT Event Handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      xTaskCreate(mqtt_publish_task, "mqtt_pub", 4096, NULL, 5, NULL);
      mqtt_connected = true;
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      mqtt_connected = false;
      break;
    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_PUBLISHED:
      // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT_EVENT_DATA");
      printf("\nTOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      break;
    case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      break;
    default:
      ESP_LOGI(TAG, "Other event id:%d", event->event_id);
      break;
  }
}

static void mqtt_app_start(void) {
  // Configure MQTT client
  const esp_mqtt_client_config_t mqtt_cfg = {
    .broker = {
      .address.uri = CONFIG_BROKER_URI,
      .verification.certificate = (const char *)mqtt_eclipseprojects_io_pem_start
    },
  };
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);
}

// I2C initialization
static void i2c_init(void) {
  i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = SCL_GPIO,
    .sda_io_num = SDA_GPIO,
    .glitch_ignore_cnt = 7,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
}

static void mpu6050_init_pair(void) {
  // Create a pair of MPU6050 sensors
  mpu_pair = mpu6050_create_pair(bus_handle, MPU1_ADDR, MPU2_ADDR);

  // Check if either sensor failed to initialize
  if (!mpu_pair.mpu1 || !mpu_pair.mpu2) {
    ESP_LOGE("MPU6050", "Initialization failed! MPU1: %s, MPU2: %s",
             mpu_pair.mpu1 ? "OK" : "FAILED",
             mpu_pair.mpu2 ? "OK" : "FAILED");
    return;
  }

  // Configure both sensors with desired settings
  ESP_LOGI("MPU6050", "Initializing MPU1 at address 0x68...");
  ESP_ERROR_CHECK(mpu6050_config(mpu_pair.mpu1, ACCE_FS_16G, GYRO_FS_1000DPS));
  ESP_ERROR_CHECK(mpu6050_wake_up(mpu_pair.mpu1));

  ESP_LOGI("MPU6050", "Initializing MPU2 at address 0x69...");
  ESP_ERROR_CHECK(mpu6050_config(mpu_pair.mpu2, ACCE_FS_16G, GYRO_FS_1000DPS));
  ESP_ERROR_CHECK(mpu6050_wake_up(mpu_pair.mpu2));
}

static void dual_data_read() {
  esp_err_t ret = mpu6050_get_dual_data(mpu_pair, &dual_acce, &dual_gyro);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MPU6050 data: %s", esp_err_to_name(ret));
    return;
  }
}

static void i2c_scanner(void) {
  uint8_t address;
  for (address=1; address<127; address++) {
    if (i2c_master_probe(bus_handle, address, 100) == ESP_OK) {
      ESP_LOGI("I2C", "Found device at 0x%02X", address);
    }
  }
}

void app_main(void) {
  // Initialize NVS
  ESP_ERROR_CHECK(nvs_flash_init());

  // Initialize WiFi
  wifi_init();
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  // Initialize the I2C bus
  i2c_init();

  // I2C Scanner for verification of addresses
  i2c_scanner();

  // Initialize both MPU6050 sensors as a pair
  mpu6050_init_pair();

  const esp_timer_create_args_t timer_args = {
    .callback = &dual_data_read,
    .name = "mpu6050_timer"
  };
  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / SAMPLE_RATE_HZ));

  // Start MQTT
  mqtt_app_start();
}
