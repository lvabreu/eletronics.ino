
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

   
static constexpr uint8_t ADXL345_ADDR = 0x53;
static constexpr uint8_t ITG3205_ADDR = 0x68;
static constexpr uint8_t HMC5883L_ADDR = 0x1E;
static constexpr int I2C_TIMEOUT_MS = 100;
static const char *TAG - "IMU_I2C";

IMU_I2C:: ~ IMU_I2C()
{
}

IMU_I2C:: ~ IMU_I2C(){
    if(m_initialized)
    {
    i2c_master_bus_rm_device(i2c_master_dev_handle_t);
    i2c_del_master_bus(m_bus_handle);
    }
}

esp_err_t init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
{
    if(m_initialized)
        return ESP_ERR_INVALID_STATE
}    

// --- Inicialização ---
esp_err_t IMU_I2C::init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    if (m_initialized) 
        return ESP_ERR_INVALID_STATE;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &m_bus_handle);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Falha ao criar bus I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configura os dispositivos
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = 100000,//100khz 
    };
    

