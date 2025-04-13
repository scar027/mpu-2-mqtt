#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "mpu6050.h"

// Configuration
#define SDA_GPIO        21
#define SCL_GPIO        22
#define MPU1_ADDR       0x68
#define MPU2_ADDR       0x69
#define SAMPLE_RATE_HZ  200

static i2c_master_bus_handle_t bus_handle;
static mpu6050_handle_pair_t mpu_pair;
static SemaphoreHandle_t data_mutex;
static dual_acce_value_t dual_acce;
static dual_gyro_value_t dual_gyro;

// Timer callback for 200Hz sampling
static void IRAM_ATTR mpu6050_timer_cb(void *arg) {
  if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
    if (mpu6050_get_dual_data(mpu_pair, &dual_acce, &dual_gyro) != ESP_OK) {
      ESP_LOGE("MPU6050", "Failed to read data from sensors");
    }
    xSemaphoreGive(data_mutex);
  }
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
  if (!mpu_pair.mpu1) {
    ESP_LOGE("MPU6050", "MPU1 initialization failed!");
    return;
  }
  // Configure both sensors with desired settings
  ESP_LOGI("MPU6050", "Initializing MPU1 at address 0x68...");
  ESP_ERROR_CHECK(mpu6050_config(mpu_pair.mpu1, ACCE_FS_16G, GYRO_FS_1000DPS));
  ESP_ERROR_CHECK(mpu6050_wake_up(mpu_pair.mpu1));

  if (!mpu_pair.mpu2) {
    ESP_LOGE("MPU6050", "MPU2 initialization failed!");
    return;
  }
  ESP_LOGI("MPU6050", "Initializing MPU2 at address 0x69...");
  ESP_ERROR_CHECK(mpu6050_config(mpu_pair.mpu2, ACCE_FS_16G, GYRO_FS_1000DPS));
  ESP_ERROR_CHECK(mpu6050_wake_up(mpu_pair.mpu2));
}

void app_main(void) {
  // Initialize the I2C bus
  i2c_init();

  uint8_t address;
  for (address=1; address<127; address++) {
    if (i2c_master_probe(bus_handle, address, 100) == ESP_OK) {
      ESP_LOGI("I2C", "Found device at 0x%02X", address);
    }
  }

  // Initialize both MPU6050 sensors as a pair
  mpu6050_init_pair();

  // Create mutex for shared data access
  data_mutex = xSemaphoreCreateMutex();

  const esp_timer_create_args_t timer_args = {
    .callback = &mpu6050_timer_cb,
    .name = "mpu6050_timer"
  };

  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / SAMPLE_RATE_HZ));
  while (1) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    printf("ax1:%.2f, ay1:%.2f, az1:%.2f, gx1:%.2f, gy1:%.2f, gz1:%.2f, "
           "ax2:%.2f, ay2:%.2f, az2:%.2f, gx2:%.2f, gy2:%.2f, gz2:%.2f\n",
           dual_acce.acce_1.acce_x, dual_acce.acce_1.acce_y, dual_acce.acce_1.acce_z,
           dual_gyro.gyro_1.gyro_x, dual_gyro.gyro_1.gyro_y, dual_gyro.gyro_1.gyro_z,
           dual_acce.acce_2.acce_x, dual_acce.acce_2.acce_y, dual_acce.acce_2.acce_z,
           dual_gyro.gyro_2.gyro_x, dual_gyro.gyro_2.gyro_y, dual_gyro.gyro_2.gyro_z);
    xSemaphoreGive(data_mutex);
    vTaskDelay(pdMS_TO_TICKS(50));  // Update console at ~20Hz frequency
  }
}
