
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
    

//     // ADXL345
//     dev_cfg.device_address = ADXL345_ADDR;
//     ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &adxl_handle);
//     if (ret != ESP_OK) goto fail;
//     i2c_write_byte(adxl_handle, 0x2D, 0x08); // Power on (measure)

//     // ITG3205
//     dev_cfg.device_address = ITG3205_ADDR;
//     ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &itg_handle);
//     if (ret != ESP_OK) goto fail;
//     i2c_write_byte(itg_handle, 0x3E, 0x00); // Power management

//     // HMC5883L
//     dev_cfg.device_address = HMC5883L_ADDR;
//     ret = i2c_master_bus_add_device(m_bus_handle, &dev_cfg, &hmc_handle);
//     if (ret != ESP_OK) goto fail;
//     i2c_write_byte(hmc_handle, 0x00, 0x70); // Config A
//     i2c_write_byte(hmc_handle, 0x01, 0xA0); // Config B
//     i2c_write_byte(hmc_handle, 0x02, 0x00); // Continuous mode

//     m_initialized = true;
//     ESP_LOGI(TAG, "GY-85 inicializado com sucesso!");
//     return ESP_OK;

// fail:
//     i2c_del_master_bus(m_bus_handle);
//     return ret;
// }

// // --- Leitura de sensores ---
// esp_err_t IMU_I2C ::read_accel(int16_t &x, int16_t &y, int16_t &z) {
//     uint8_t buf[6];
//     esp_err_t ret = i2c_read_bytes(adxl_handle, 0x32, buf, 6);
//     if (ret == ESP_OK) {
//         x = (buf[1] << 8) | buf[0];
//         y = (buf[3] << 8) | buf[2];
//         z = (buf[5] << 8) | buf[4];
//     }
//     return ret;
// }

// esp_err_t IMU_I2C ::read_gyro(int16_t &x, int16_t &y, int16_t &z) {
//     uint8_t buf[6];
//     esp_err_t ret = i2c_read_bytes(itg_handle, 0x1D, buf, 6);
//     if (ret == ESP_OK) {
//         x = (buf[0] << 8) | buf[1];
//         y = (buf[2] << 8) | buf[3];
//         z = (buf[4] << 8) | buf[5];
//     }
//     return ret;
// }

// esp_err_t IMU_I2C ::read_mag(int16_t &x, int16_t &y, int16_t &z) {
//     uint8_t buf[6];
//     esp_err_t ret = i2c_read_bytes(hmc_handle, 0x03, buf, 6);
//     if (ret == ESP_OK) {
//         x = (buf[0] << 8) | buf[1];
//         z = (buf[2] << 8) | buf[3];
//         y = (buf[4] << 8) | buf[5];
//     }
//     return ret;
// }

// // --- Destruição ---
// void IMU_I2C ::deinit() {
//     if (!m_initialized) return;
//     i2c_master_bus_rm_device(adxl_handle);
//     i2c_master_bus_rm_device(itg_handle);
//     i2c_master_bus_rm_device(hmc_handle);
//     i2c_del_master_bus(m_bus_handle);
//     m_initialized = false;
// }
   

//     esp_err_t read_accel(int16_t &x, int16_t &y, int16_t &z);
//     esp_err_t read_gyro(int16_t &x, int16_t &y, int16_t &z);
//     esp_err_t read_mag(int16_t &x, int16_t &y, int16_t &z);
//     void deinit();


//     i2c_master_bus_handle_t m_bus_handle;
//     i2c_master_dev_handle_t adxl_handle;
//     i2c_master_dev_handle_t itg_handle;
//     i2c_master_dev_handle_t hmc_handle;
//     bool m_initialized;

//     esp_err_t i2c_write_byte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);
//     esp_err_t i2c_read_bytes(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);
// };

// const char* IMU_I2C ::TAG = "IMU_I2C ";

//   0x35  53 DATAY1   R   00000000  Y-Axis Data 1.
//   0x36  54 DATAZ0   R   00000000  Z-Axis Data 0.
//   0x37  55 DATAZ1   R   00000000  Z-Axis Data 1.
// */
#define SENSOR 0x53 // Device add in which is also included the 8th bit for selectting the mode, read in this case
#define Power_Register 0x2D
#define X_Axis_Register_DATAX0 0x32 // Hexadecima address for the DATAX0 internal register.
#define X_Axis_Register_DATAX1 0x33 // Hexadecima address for the DATAX1 internal register.
#define Y_Axis_Register_DATAY0 0x34
#define Y_Axis_Register_DATAY1 0x35
#define Z_Axis_Register_DATAZ0 0x36
#define Z_Axis_Register_DATAZ1 0x37

int  DataReturned_x0, DataReturned_x1, DataModified_x_out;
float x;

void setup(){
                                        // Initialize the wire library baudrate: 9600
  Wire.begin(9600);
  Serial.begin(9600);
  delay(100);
                                        // 1 - Which device (address) are you interest in ?                                   
  Wire.beginTransmission(SENSOR);       // Begin transmission with the chosen chip
                                        // 2 - Which Register do you want to talk to ?
  Wire.write(Power_Register);           // Power-saving features control
                                        // Register 0x2D POWER_CTL is 8 bits: [unused:unused:Link:AUTO_SLEEP:Measure:Sleep:Wakeup1:Wakeup0]
                                        // 3 - What do you want to transmit? Enable measurement
  Wire.write(8);                        // bit D3 high for measuring enable: 8d - 00001000b
  
  Wire.endTransmission();               // end Transmission
}

void loop() {
  
 //-------------------------------------REAPEAT THIS PIECE OF CODE FOR EACH AXIS---------------------------------------//
  
  Wire.beginTransmission(SENSOR);       // begin transmission
                                        // 4 -What do you want to ask about the specific register ?
  Wire.write(X_Axis_Register_DATAX0);   // Requiring Register DATA_0 
  Wire.write(X_Axis_Register_DATAX1);   // Requiring Register DATA_1
                                        
  Wire.endTransmission();               // end Transmission
                                        // 5 - Now wait for the data looping in a while ...
  Wire.requestFrom(SENSOR, 2);
  if (Wire.available() <= 2) {
    DataReturned_x0 = Wire.read();
    DataReturned_x1 = Wire.read();
                                        // 6 - Now how do you want the configuration of the three axes ?
                                        //  now read the data and do as the datasheet says.
    DataReturned_x1    = DataReturned_x1 << 8;
    DataModified_x_out = DataReturned_x0 + DataReturned_x1;
    x = DataModified_x_out / 256.0;
    
    //----------------------------------------------------------------------------------------------------------------//
  
  }
                                        // Prints the data on the Serial Monitor
  Serial.print("x = ");
  Serial.println(x);
}
