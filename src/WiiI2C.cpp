#include "Arduino.h"
#include "WiiI2C.h"
#include <driver/i2c.h>
#include <string.h>

const uint8_t WiiI2C::data_init1[] = {0xF0, 0x55};
const uint8_t WiiI2C::data_init2[] = {0xFB, 0x00};
const uint8_t WiiI2C::data_req_ident[] = {0xFA};
const uint8_t WiiI2C::data_req_data[] = {0x00};

i2c_port_t WiiI2C::wii_i2c_port_num;
uint8_t WiiI2C::read_data[6];

#if WII_I2C_ENABLE_MULTI_CORE
TaskHandle_t WiiI2C::read_state_task_handle;
int WiiI2C::read_state_task_delay;
SemaphoreHandle_t WiiI2C::read_mutex;
uint8_t WiiI2C::shared_read_data[6];
volatile uint8_t WiiI2C::shared_copy_data[6];
volatile uint32_t WiiI2C::shared_copy_ready;
#endif

esp_err_t WiiI2C::setupI2C(i2c_port_t i2c_port_num, int sda_pin, int scl_pin)
{
    this->wii_i2c_port_num = i2c_port_num;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100KHz

    Serial.println("sda_pin");
    Serial.println(sda_pin);
    Serial.println("scl_pin");
    Serial.println(scl_pin);
    Serial.println("conf.sda_io_num");
    Serial.println(conf.sda_io_num);
    Serial.println(conf.scl_io_num);
    i2c_param_config(i2c_port_num, &conf);
    return i2c_driver_install(i2c_port_num, conf.mode, 0, 0, 0);
}

esp_err_t WiiI2C::write(const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(this->wii_i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t WiiI2C::read(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(this->wii_i2c_port_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int WiiI2C::init(int i2c_port, int sda_pin, int scl_pin)
{
    this->wii_i2c_port_num = i2c_port;
    this->address = 0x52; // Wii Nunchuk I2C address

    if (setupI2C(i2c_port, sda_pin, scl_pin) != ESP_OK)
        return 1;
    if (write(data_init1, sizeof(data_init1)) != ESP_OK)
        return 1;
    if (write(data_init2, sizeof(data_init2)) != ESP_OK)
        return 1;
    return 0;
}

int WiiI2C::requestState()
{
    if (write(data_req_data, sizeof(data_req_data)) != ESP_OK)
        return 1;
    return 0;
}

const unsigned char *WiiI2C::readIdent()
{
    if (write(data_req_ident, sizeof(data_req_ident)) != ESP_OK)
        return NULL;
    if (read(read_data, sizeof(read_data)) != ESP_OK)
        return NULL;
    return read_data;
}

const unsigned char *WiiI2C::readState()
{
    if (read(read_data, sizeof(read_data)) != ESP_OK)
        return NULL;
    return read_data;
}

unsigned int WiiI2C::decodeIdent(const unsigned char *ident)
{
    if (!ident)
        return WII_I2C_IDENT_NONE;
    return (((uint32_t)ident[0] << 24) | ((uint32_t)ident[1] << 16) | ((uint32_t)ident[2] << 8) | ident[3]);
}

void WiiI2C::decodeNunchuk(const unsigned char *data, WiiI2CNunchukState *state)
{
    state->acc_x = (data[2] << 2) | ((data[5] >> 2) & 0x03);
    state->acc_y = (data[3] << 2) | ((data[5] >> 4) & 0x03);
    state->acc_z = (data[4] << 2) | ((data[5] >> 6) & 0x03);
    state->x = data[0];
    state->y = data[1];
    state->c = !(data[5] & 0x02);
    state->z = !(data[5] & 0x01);
}

void WiiI2C::decodeClassic(const unsigned char *data, WiiI2CClassicState *state)
{
    state->lx = ((data[0] & 0x3F) << 2) | ((data[2] >> 4) & 0x03);
    state->ly = (data[1] & 0x3F);
    state->rx = ((data[0] & 0xC0) >> 3) | ((data[1] & 0xC0) >> 5) | (data[2] & 0x03);
    state->ry = data[2] >> 2 & 0x0F;
    state->a_lt = data[3] & 0x1F;
    state->a_rt = data[4] & 0x1F;
    state->d_lt = !!(data[4] & 0x20);
    state->d_rt = !!(data[4] & 0x40);
    state->up = !!(data[5] & 0x01);
    state->down = !!(data[5] & 0x02);
    state->left = !!(data[5] & 0x04);
    state->right = !!(data[5] & 0x08);
    state->a = !!(data[5] & 0x10);
    state->b = !!(data[5] & 0x20);
    state->x = !!(data[5] & 0x40);
    state->y = !!(data[5] & 0x80);
    state->zr = !!(data[6] & 0x01);
    state->zl = !!(data[6] & 0x02);
    state->home = !!(data[6] & 0x04);
    state->plus = !!(data[6] & 0x08);
    state->minus = !!(data[6] & 0x10);
}

#if WII_I2C_ENABLE_MULTI_CORE

void WiiI2C::read_state_task_func(void *pvParameters)
{
    WiiI2C *wiiI2C = static_cast<WiiI2C*>(pvParameters);
    while (true)
    {
        if (wiiI2C->requestState() == 0)
        {
            const unsigned char *data = wiiI2C->readState();
            if (data)
            {
                xSemaphoreTake(wiiI2C->getReadMutex(), portMAX_DELAY);
                wiiI2C->setSharedCopyData(data);
                wiiI2C->setSharedCopyReady(1);
                xSemaphoreGive(wiiI2C->getReadMutex());
            }
        }
        vTaskDelay(wiiI2C->getReadStateTaskDelay() / portTICK_PERIOD_MS);
    }
}

int WiiI2C::startReadTask(int cpu_num, int delay)
{
    read_state_task_delay = delay;
    read_mutex = xSemaphoreCreateMutex();
    if (!read_mutex)
    {
        return 1;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(read_state_task_func, "wiiI2CTask", 1024, this, 0, &read_state_task_handle, cpu_num);
    if (ret != pdPASS)
    {
        vSemaphoreDelete(read_mutex);
        return 1;
    }
    return 0;
}

const unsigned char *WiiI2C::readDataFromTask()
{
    if (!shared_copy_ready)
        return NULL;

    xSemaphoreTake(read_mutex, portMAX_DELAY);
    memcpy(read_data, (unsigned char *)shared_copy_data, sizeof(read_data));
    shared_copy_ready = 0;
    xSemaphoreGive(read_mutex);

    return (const unsigned char *)read_data;
}

#endif /* WII_I2C_ENABLE_MULTI_CORE */
