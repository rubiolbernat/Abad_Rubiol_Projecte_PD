#include <Arduino.h>
#include "ServoController.cpp"
#include "MyWebServer.cpp"
#include "wii_i2c.h"

#include "ESP32Wiimote.h"
ESP32Wiimote wiimote;

// Inicialitzem una instància de ServoController amb els pins dels servos
ServoController servoController(12, 13, 14, 15);

// PIN 27 reservat per a senyal interna
// Pins connectats als nunchuks:
#define PIN_SDA_1 21 // dataN nunchuk 1
#define PIN_SCL_1 22 // Clock nunchuk 1
// ESP32 I2C port (0 o 1):
#define WII_I2C_PORT 0
unsigned int controller_type_1 = 0;
unsigned int controller_type_2 = 0;

// dades
SDades dades;

// const char* ssid     = "MOVISTAR_6B6C"; OPPO BernArtNet
// const char* password = "592X7x5ySxb222FTr7uU";12345678
MyWebServer webServer("MOVISTAR_6B6C", "592X7x5ySxb222FTr7uU");

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

    // Mostrar la detecció dels nunchuks
    if (controller_type_1 == WII_I2C_IDENT_NUNCHUK)
    {
        Serial.printf("-> nunchuk 1 detected\n");
    }
    else
    {
        Serial.printf("-> unknown controller detected for nunchuk 1: 0x%06x\n", controller_type_1);
    }

    // Solicitar l'estat del primer nunchuk
    wii_i2c_request_state();
}

void setup()
{
    // Inicialitzem Arduino
    Serial.begin(115200);
    wiimote.init();
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
    if (x > 6 || x < -6)
    {
        retx = x / 64;
    }
    if (y > 6 || y < -6)
    {
        rety = y / 64;
    }
}

// Funció loop() d'Arduino
void loop()
{

    // Si es control per nunchuk
    if (webServer.getnunchukStatus())
    {
        // Serial.println("tthjk");
        // webServer.set_handle(dades);
        //  Llegir les dades del primer nunchuk
        const unsigned char *dataN_1 = wii_i2c_read_state();
        if (dataN_1)
        {
            // Serial.println("testttt");
            wii_i2c_request_state();
            wii_i2c_nunchuk_state state;
            wii_i2c_decode_nunchuk(dataN_1, &state);
            float retx = 0, rety = 0;
            xynunchuktoservo(static_cast<int>(state.x), static_cast<int>(state.y), retx, rety);

            // Servo 1 base
            float newPosition = dades.positions[1] + retx;
            if (state.c && state.z)
            {
                if (newPosition <= 180 && newPosition >= 0)
                {
                    dades.positions[1] = newPosition;
                }
            }

            // Servo 2 braç
            if (!state.c && !state.z)
            {
                // Assegurar que no s'excedeixen els límits abans de sumar
                if (dades.positions[2] + rety > 180)
                {
                    rety = 180 - dades.positions[2];
                }
                else if (dades.positions[2] + rety < 0)
                {
                    rety = -dades.positions[2];
                }

                // Actualitzar la posició
                dades.positions[2] += rety;
            }

            // Servo 3 braç
            if (!state.c && !state.z)
            {
                // Assegurar que no s'excedeixen els límits abans de sumar
                if (dades.positions[3] + retx > 180)
                {
                    retx = 180 - dades.positions[3];
                }
                else if (dades.positions[3] + retx < 0)
                {
                    retx = -dades.positions[3];
                }

                // Actualitzar la posició
                dades.positions[3] += retx;
            }

            // Servo 4 pinça
            if (!state.c || !state.z)
            {
                if (state.c)
                {
                    if (dades.positions[4] < 180) // Comprovar que no s'excedeix el límit superior
                    {
                        dades.positions[4] += 2;
                    }
                }
                else if (state.z)
                {
                    if (dades.positions[4] > 0) // Comprovar que no s'excedeix el límit inferior
                    {
                        dades.positions[4] -= 2;
                    }
                }
            }
        }
        else
        {
            Serial.printf("No dataN for nunchuk 1 :(\n");
        }

        // Pausa breu entre lectures per evitar col·lisions
        delay(10);
        Serial.print("WEB 1: ");
        Serial.print(dades.positions[1]);
        Serial.print(" 2: ");
        Serial.print(dades.positions[2]);
        Serial.print(" 3: ");
        Serial.print(dades.positions[3]);
        Serial.print(" 4: ");
        Serial.println(dades.positions[4]);

        servoController = dades.positions;
        // delay(100);
    }
    else if (webServer.getbtStatus())
    {
        wiimote.task();
        if (wiimote.available() > 0)
        {
            // Suposem que wiimote.getButtonState() retorna aquest valor
            uint16_t button = wiimote.getButtonState();

            // Array de booleans per emmagatzemar l'estat de cada botó
            bool buttonPressed[16];

            // Comprovar l'estat de cada botó
            for (uint8_t i = 0; i < 16; ++i)
            {
                buttonPressed[i] = (button & (1 << i)) != 0;
            }
            // Mostrar l'estat de cada botó
            /*for (uint8_t i = 0; i < 16; ++i)
            {
                Serial.print("Botó ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(buttonPressed[i] ? "Premut" : "No premut");
            }*/

            NunchukState nunchuk = wiimote.getNunchukState();

            /*Serial.printf("%04x\n", button);
            if (button == ESP32Wiimote::BUTTON_A)
            {
                Serial.println("A button");
            }*/

            float retx = 0, rety = 0;
            xynunchuktoservo(static_cast<int>(nunchuk.xStick - 127), static_cast<int>(nunchuk.yStick - 127), retx, rety);

            // Servo 1 base
            float newPosition = dades.positions[1] + retx;
            if (newPosition <= 180 && newPosition >= 0)
            {
                dades.positions[1] = newPosition;
            }

            // Servo 2 braç
            // Assegurar que no s'excedeixen els límits abans de sumar
            if (dades.positions[2] + rety > 180)
            {
                rety = 180 - dades.positions[2];
            }
            else if (dades.positions[2] + rety < 0)
            {
                rety = -dades.positions[2];
            }
            // Actualitzar la posició
            dades.positions[2] += rety;

            xynunchuktoservo(static_cast<int>(nunchuk.xStick - 127), static_cast<int>(nunchuk.yStick - 127), retx, rety);
            // Servo 3 braç
            if ((button & (wiimote.BUTTON_UP || wiimote.BUTTON_RIGHT)) || (wiimote.BUTTON_DOWN || wiimote.BUTTON_LEFT))
            {
                // Serial.printf("%04x\n", button);
                if (button & wiimote.BUTTON_UP)
                {
                    if (dades.positions[3] < 180) // Comprovar que no s'excedeix el límit superior
                    {
                        dades.positions[3] += 2;
                    }
                }
                else if (button & wiimote.BUTTON_DOWN)
                {
                    if (dades.positions[3] > 0) // Comprovar que no s'excedeix el límit inferior
                    {
                        dades.positions[3] -= 2;
                    }
                }
            }

            // Servo 4 pinça
            if ((button & wiimote.BUTTON_A) || (button & wiimote.BUTTON_B))
            {
                // Serial.printf("%04x\n", button);
                if (button & wiimote.BUTTON_A)
                {
                    if (dades.positions[4] < 180) // Comprovar que no s'excedeix el límit superior
                    {
                        dades.positions[4] += 2;
                    }
                }
                else if (button & wiimote.BUTTON_B)
                {
                    if (dades.positions[4] > 0) // Comprovar que no s'excedeix el límit inferior
                    {
                        dades.positions[4] -= 2;
                    }
                }
            }
        }
        Serial.print("BLUETOOTH 1: ");
        Serial.print(dades.positions[1]);
        Serial.print(" 2: ");
        Serial.print(dades.positions[2]);
        Serial.print(" 3: ");
        Serial.print(dades.positions[3]);
        Serial.print(" 4: ");
        Serial.println(dades.positions[4]);

        servoController = dades.positions;
        // delay(10);
    }
    else
    {
        webServer.handle(dades);
        // Serial.printf("holaaaaaaaaaaaaaaaa");
        //  Si no es control per nunchuk, imprimeix les dades emmagatzemades

        Serial.print("WEB 1: ");
        Serial.print(dades.positions[1]);
        Serial.print(" 2: ");
        Serial.print(dades.positions[2]);
        Serial.print(" 3: ");
        Serial.print(dades.positions[3]);
        Serial.print(" 4: ");
        Serial.println(dades.positions[4]);

        servoController = dades.positions;

        // delay(10);
    }

    // Pausa entre iteracions del bucle principal
    delay(4);
}