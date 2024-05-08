#include "Arduino.h"
#include "wii_i2c.h"
/*
// Pins connectats als nunchuks:
#define PIN_SDA_1 21 // Data nunchuk 1
#define PIN_SCL_1 22 // Clock nunchuk 1
#define PIN_SDA_2 23 // Data nunchuk 2
#define PIN_SCL_2 24 // Clock nunchuk 2

// ESP32 I2C port (0 o 1):
#define WII_I2C_PORT 0

unsigned int controller_type_1 = 0;
unsigned int controller_type_2 = 0;

void show_nunchuk(const unsigned char *data, int nunchuk_number)
{
  wii_i2c_nunchuk_state state;
  wii_i2c_decode_nunchuk(data, &state);
  
  Serial.printf("Nunchuk %d:\n", nunchuk_number);
  Serial.printf("giroscopi = (%5d,%5d,%5d)\n", state.acc_x, state.acc_y, state.acc_z);
  Serial.printf("pos joyst = (%5d,%5d)\n", state.x, state.y);
  Serial.printf("c=%d, z=%d\n", state.c, state.z);
}

void setup()
{
  Serial.begin(115200);
  Serial.printf("Starting...\n");

  if (wii_i2c_init(WII_I2C_PORT, PIN_SDA_1, PIN_SCL_1) != 0) {
    Serial.printf("ERROR initializing wii i2c controller for nunchuk 1\n");
    return;
  }
  if (wii_i2c_init(WII_I2C_PORT, PIN_SDA_2, PIN_SCL_2) != 0) {
    Serial.printf("ERROR initializing wii i2c controller for nunchuk 2\n");
    return;
  }

  // Llegeix els identificadors dels dos nunchuks
  const unsigned char *ident_1 = wii_i2c_read_ident();
  const unsigned char *ident_2 = wii_i2c_read_ident();
  
  if (!ident_1 || !ident_2) {
    Serial.printf("no ident :(\n");
    return;
  }
  
  controller_type_1 = wii_i2c_decode_ident(ident_1);
  controller_type_2 = wii_i2c_decode_ident(ident_2);
  
  if (controller_type_1 == WII_I2C_IDENT_NUNCHUK) {
    Serial.printf("-> nunchuk 1 detected\n");
  } else {
    Serial.printf("-> unknown controller detected for nunchuk 1: 0x%06x\n", controller_type_1);
  }
  
  if (controller_type_2 == WII_I2C_IDENT_NUNCHUK) {
    Serial.printf("-> nunchuk 2 detected\n");
  } else {
    Serial.printf("-> unknown controller detected for nunchuk 2: 0x%06x\n", controller_type_2);
  }
  
  wii_i2c_request_state();
}

void loop()
{
  // Llegeix les dades del primer nunchuk
  const unsigned char *data_1 = wii_i2c_read_state();
  if (data_1) {
    show_nunchuk(data_1, 1);
  } else {
    Serial.printf("no data for nunchuk 1 :(\n");
  }
  
  // Llegeix les dades del segon nunchuk
  const unsigned char *data_2 = wii_i2c_read_state();
  if (data_2) {
    show_nunchuk(data_2, 2);
  } else {
    Serial.printf("no data for nunchuk 2 :(\n");
  }
  
  wii_i2c_request_state();
  delay(400);
}
*/