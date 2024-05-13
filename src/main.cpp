#include <Arduino.h>
#include "ServoController.cpp"
#include "MyWebServer.cpp"
#include "wii_i2c.h"

// Inicialitzem una instància de ServoController amb els pins dels servos
ServoController servoController(12, 13, 14, 15);

// PIN 27 reservat per a senyal interna
// Pins connectats als nunchuks:
#define PIN_SDA_1 21 // dataN nunchuk 1   2
#define PIN_SCL_1 22 // Clock nunchuk 1
#define PIN_SDA_2 29 // dataN nunchuk 2
#define PIN_SCL_2 23 // Clock nunchuk 2
// ESP32 I2C port (0 o 1):
#define WII_I2C_PORT 0
unsigned int controller_type_1 = 0;
unsigned int controller_type_2 = 0;

// dades
SDades dades;

MyWebServer webServer("OPPO BernArtNet", "12345678");

// Funció setup() d'Arduino
void startnunchuk()
{
    // Inicialitzar i llegir identificador del primer nunchuk
    if (wii_i2c_init(WII_I2C_PORT, PIN_SDA_1, PIN_SCL_1) != 0)
    {
        Serial.printf("ERROR initializing wii i2c controller for nunchuk 1\n");
        return;
    }
    const unsigned char *ident_1 = wii_i2c_read_ident();
    if (!ident_1)
    {
        Serial.printf("no ident for nunchuk 1 :(\n");
        return;
    }
    controller_type_1 = wii_i2c_decode_ident(ident_1);

    /*// Inicialitzar i llegir identificador del segon nunchuk
    if (wii_i2c_init(WII_I2C_PORT, PIN_SDA_2, PIN_SCL_2) != 0)
    {
        Serial.printf("ERROR initializing wii i2c controller for nunchuk 2\n");
        return;
    }
    const unsigned char *ident_2 = wii_i2c_read_ident();
    if (!ident_2)
    {
        Serial.printf("no ident for nunchuk 2 :(\n");
        return;
    }
    controller_type_2 = wii_i2c_decode_ident(ident_2);
*/
    // Mostrar la detecció dels nunchuks
    if (controller_type_1 == WII_I2C_IDENT_NUNCHUK)
    {
        Serial.printf("-> nunchuk 1 detected\n");
    }
    else
    {
        Serial.printf("-> unknown controller detected for nunchuk 1: 0x%06x\n", controller_type_1);
    }
    /*
        if (controller_type_2 == WII_I2C_IDENT_NUNCHUK)
        {
            Serial.printf("-> nunchuk 2 detected\n");
        }
        else
        {
            Serial.printf("-> unknown controller detected for nunchuk 2: 0x%06x\n", controller_type_2);
        }*/

    // Solicitar l'estat del primer nunchuk
    wii_i2c_request_state();
}

void setup()
{
    // Inicialitzem Arduino
    Serial.begin(115200);

    // Inicialitzem el controlador de servos
    servoController.setup();
    servoController.attachServos();

    // Inicialitzem el servidor web
    webServer.begin();

    // Iniciem nunchakus 1 i 2
    startnunchuk();
}

void xynunchuktoservo(int x, int y, float &retx, float &rety)
{
    if (x > 5 || x < -5)
    {
        retx = x/8;
    }
    if (y > 5 || y < -5)
    {
        rety = y/8;
    }
}

// Funció loop() d'Arduino
void loop()
{
    
    // Si es control per nunchuk
    if (webServer.getnunchukStatus())
    {
        //Serial.println("tthjk");
       // webServer.set_handle(dades);
        //  Llegir les dades del primer nunchuk
        const unsigned char *dataN_1 = wii_i2c_read_state();
        if (dataN_1)
        {
            //Serial.println("testttt");
            wii_i2c_request_state();
            wii_i2c_nunchuk_state state;
            wii_i2c_decode_nunchuk(dataN_1, &state);
            float retx=0, rety=0;
            xynunchuktoservo(static_cast<int>(state.x), static_cast<int>(state.y), retx, rety);
            if (dades.positions[1] <= 180 && dades.positions[1] >= -180)
            {
                dades.positions[1] += static_cast<float>(retx);
            }else if(dades.positions[1] > 180){
                dades.positions[1] = 180;
            }else if(dades.positions[1] < -180){
                dades.positions[1] = -180;
            }
            if (dades.positions[2] <= 180 && dades.positions[2] >= -180)
            {
                dades.positions[2] += static_cast<float>(rety);
            }else if(dades.positions[2] > 180){
                dades.positions[2] = 180;
            }else if(dades.positions[2] < -180){
                dades.positions[2] = -180;
            }
            /*Serial.println("desdenunch");
            Serial.println(retx);
            Serial.println(dades.positions[1]);
            Serial.println(dades.positions[2]);*/
            /*
            Serial.printf("Nunchuk 1:\n");
            Serial.printf("giroscopi = (%5d,%5d,%5d)\n", state.acc_x, state.acc_y, state.acc_z);
            Serial.printf("pos joyst = (%5d,%5d)\n", state.x, state.y);
            Serial.printf("c=%d, z=%d\n", state.c, state.z);*/
        }
        else
        {
            Serial.printf("No dataN for nunchuk 1 :(\n");
        }

        // Pausa breu entre lectures per evitar col·lisions
        delay(10);
/*
        // Llegir les dades del segon nunchuk
        const unsigned char *dataN_2 = wii_i2c_read_state();
        if (dataN_2)
        {
            wii_i2c_request_state();
            wii_i2c_nunchuk_state state;
            wii_i2c_decode_nunchuk(dataN_2, &state);
            /*
                        Serial.printf("Nunchuk 2:\n");
                        Serial.printf("giroscopi = (%5d,%5d,%5d)\n", state.acc_x, state.acc_y, state.acc_z);
                        Serial.printf("pos joyst = (%5d,%5d)\n", state.x, state.y);
                        Serial.printf("c=%d, z=%d\n", state.c, state.z);*
        }
        else
        {
            Serial.printf("No dataN for nunchuk 2 :(\n");
        }
*/
        servoController = dades.positions;
    }
    else
    {
        webServer.handle(dades);
        //Serial.printf("holaaaaaaaaaaaaaaaa");
        // Si no es control per nunchuk, imprimeix les dades emmagatzemades
        Serial.println(dades.positions[1]);
        Serial.println(dades.positions[2]);
        Serial.println(dades.positions[3]);
        Serial.println(dades.positions[4]);

        servoController = dades.positions;

        delay(500);
    }

    // Pausa entre iteracions del bucle principal
    delay(10);
}
