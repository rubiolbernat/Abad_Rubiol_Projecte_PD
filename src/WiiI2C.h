#ifndef WII_I2C_H
#define WII_I2C_H

#include <stdint.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define WII_I2C_ENABLE_MULTI_CORE 1 // 0=disable, 1=enable

#define WII_I2C_IDENT_NONE 0
#define WII_I2C_IDENT_NUNCHUK 0xa4200000
#define WII_I2C_IDENT_CLASSIC 0xa4200101

struct WiiI2CNunchukState
{
    // accelerometer
    int acc_x;
    int acc_y;
    int acc_z;

    // analog stick:
    int x;
    int y;

    // buttons:
    char c;
    char z;
};

struct WiiI2CClassicState
{
    // analog sticks:
    signed char lx;
    signed char ly;
    signed char rx;
    signed char ry;

    // triggers (a_ is the analog part, d_ is the click bit):
    unsigned char a_lt;
    unsigned char a_rt;
    char d_lt;
    char d_rt;

    // d-pad:
    char up;
    char down;
    char left;
    char right;

    // buttons:
    char a;
    char b;
    char x;
    char y;

    // bumpers:
    char zr;
    char zl;

    // face buttons:
    char home;
    char plus;
    char minus;
};

class WiiI2C
{
private:
    uint8_t address;
    i2c_config_t conf;
    static const uint8_t data_init1[];
    static const uint8_t data_init2[];
    static const uint8_t data_req_ident[];
    static const uint8_t data_req_data[];

    static i2c_port_t wii_i2c_port_num;
    static uint8_t read_data[6];

#if WII_I2C_ENABLE_MULTI_CORE
    static TaskHandle_t read_state_task_handle;
    static int read_state_task_delay;
    static SemaphoreHandle_t read_mutex;
    static uint8_t shared_read_data[6];
    static volatile uint8_t shared_copy_data[6];
    static volatile uint32_t shared_copy_ready;
#endif

public:
    // void init_v(uint8_t &init1, uint8_t &init2, uint8_t &req_ident, uint8_t &req_data);
    esp_err_t setupI2C(i2c_port_t i2c_port_num, int sda_pin, int scl_pin);
    esp_err_t write(const uint8_t *data, size_t len);
    esp_err_t read(uint8_t *data, size_t len);
    int init(int i2c_port_num, int sda_pin, int scl_pin);
    const unsigned char *readIdent();
    int requestState();
    const unsigned char *readState();
    unsigned int decodeIdent(const unsigned char *ident);
    void decodeNunchuk(const unsigned char *data, WiiI2CNunchukState *state);
    void decodeClassic(const unsigned char *data, WiiI2CClassicState *state);

#if WII_I2C_ENABLE_MULTI_CORE
    int startReadTask(int cpu_num, int delay);
    const unsigned char *readDataFromTask();
    SemaphoreHandle_t getReadMutex() { return read_mutex; }
    void setSharedCopyData(const unsigned char *data)
    {
        for (int i = 0; i < sizeof(shared_copy_data); ++i)
        {
            shared_copy_data[i] = data[i];
        }
    }
    const unsigned char *getSharedCopyData() { return (const unsigned char *)shared_copy_data; }
    void setSharedCopyReady(uint32_t ready) { shared_copy_ready = ready; }
    uint32_t getSharedCopyReady() { return shared_copy_ready; }
    int getReadStateTaskDelay() { return read_state_task_delay; }
#endif

#if WII_I2C_ENABLE_MULTI_CORE
    static void read_state_task_func(void *pvParameters);
#endif
};

#if WII_I2C_ENABLE_MULTI_CORE
void read_state_task_func(void *pvParameters);
#endif

#endif /* WII_I2C_H */
