#include <Arduino.h>
#include "ServoController.cpp"
#include "MyWebServer.cpp"
// #include "wii_i2c.h"
#include "WiiI2C.h"

// Inicialitzem una instància de ServoController amb els pins dels servos
ServoController servoController(12, 13, 14, 15);

// PIN 27 reservat per a senyal interna
// Pins connectats als nunchuks:
#define PIN_SDA_1 21 // dataN nunchuk 1
#define PIN_SCL_1 22 // Clock nunchuk 1
#define PIN_SDA_2 23 // dataN nunchuk 2
#define PIN_SCL_2 18 // Clock nunchuk 2
// ESP32 I2C port (0 o 1):
#define WII_I2C_PORT 0

// dades
SDades dades;
MyWebServer webServer("OPPO BernArtNet", "12345678");
WiiI2C wii1, wii2;

void show_nunchuk1(const unsigned char *data)
{
    WiiI2CNunchukState state;
    wii1.decodeNunchuk(data, &state);

    Serial.printf("Nunchuk 1:\n");
    Serial.printf("giroscopi = (%5d,%5d,%5d)\n", state.acc_x, state.acc_y, state.acc_z);
    Serial.printf("pos joyst = (%5d,%5d)\n", state.x, state.y);
    Serial.printf("c=%d, z=%d\n", state.c, state.z);
}

void show_nunchuk2(const unsigned char *data)
{
    WiiI2CNunchukState state;
    wii2.decodeNunchuk(data, &state);

    Serial.printf("Nunchuk 2:\n");
    Serial.printf("giroscopi = (%5d,%5d,%5d)\n", state.acc_x, state.acc_y, state.acc_z);
    Serial.printf("pos joyst = (%5d,%5d)\n", state.x, state.y);
    Serial.printf("c=%d, z=%d\n", state.c, state.z);
}

void startnunchuk()
{
    // Nunchuk 1
    if (wii1.init(WII_I2C_PORT, PIN_SDA_1, PIN_SCL_1) != 0)
    {
        Serial.printf("ERROR initializing wii i2c controller for nunchuk 1\n");
        return;
    }
    const unsigned char *ident_1 = wii1.readIdent();
    if (!ident_1)
    {
        Serial.printf("no ident for nunchuk 1 :(\n");
        return;
    }
    unsigned int controller_type_1 = wii1.decodeIdent(ident_1);

    // Mostrar la detecció dels nunchuks
    if (controller_type_1 == WII_I2C_IDENT_NUNCHUK)
    {
        Serial.printf("-> nunchuk 1 detected\n");
    }
    else
    {
        Serial.printf("-> unknown controller detected for nunchuk 1: 0x%06x\n", controller_type_1);
    }

    // Nunchuk 2

    if (wii2.init(WII_I2C_PORT, PIN_SDA_2, PIN_SCL_2) != 0)
    {
        Serial.printf("ERROR initializing wii i2c controller for nunchuk 2\n");
        return;
    }
    const unsigned char *ident_2 = wii2.readIdent();
    if (!ident_2)
    {
        Serial.printf("no ident for nunchuk 2 :(\n");
        return;
    }
    unsigned int controller_type_2 = wii2.decodeIdent(ident_2);

    // Mostrar la detecció dels nunchuks
    if (controller_type_2 == WII_I2C_IDENT_NUNCHUK)
    {
        Serial.printf("-> nunchuk 2 detected\n");
    }
    else
    {
        Serial.printf("-> unknown controller detected for nunchuk 2: 0x%06x\n", controller_type_2);
    }

    // Solicitar l'estat dels nunchuk
    wii1.requestState();
    wii2.requestState();
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
    if (x > 6 || x < -6)
    {
        retx = x / 32.0;
    }
    if (y > 6 || y < -6)
    {
        rety = y / 32.0;
    }
}

// Funció loop() d'Arduino
void loop()
{
    // Si es control per nunchuk
    if (!webServer.getnunchukStatus())
    {
        // Llegir les dades del primer nunchuk
        const unsigned char *dataN_1 = wii1.readState();
        if (dataN_1)
        {
            wii1.requestState();
            WiiI2CNunchukState state;
            wii1.decodeNunchuk(dataN_1, &state);
            float retx = 0, rety = 0;
            xynunchuktoservo(static_cast<int>(state.x), static_cast<int>(state.y), retx, rety);

            show_nunchuk1(dataN_1);
            // Servo 1 base
            float newPosition = dades.positions[1] + retx;
            if (newPosition <= 180 && newPosition >= 0)
            {
                dades.positions[1] = newPosition;
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
        }
        else
        {
            Serial.printf("No dataN for nunchuk 1 :(\n");
        }

        const unsigned char *dataN_2 = wii2.readState();
        if (dataN_2)
        {
            wii2.requestState();
            // wii_i2c_nunchuk_state state;
            WiiI2CNunchukState state;
            wii2.decodeNunchuk(dataN_2, &state);
            float retx = 0, rety = 0;
            xynunchuktoservo(static_cast<int>(state.x), static_cast<int>(state.y), retx, rety);

            show_nunchuk2(dataN_2);
            delay(1000);
            // Servo 3 braç 2
            float newPosition = dades.positions[3] + retx;
            if (newPosition <= 180 && newPosition >= 0)
            {
                dades.positions[3] = newPosition;
            }

            // Servo 4 pinça
            if (!state.c && !state.z)
            {
                // Assegurar que no s'excedeixen els límits abans de sumar
                if (dades.positions[4] + rety > 180)
                {
                    rety = 180 - dades.positions[4];
                }
                else if (dades.positions[4] + rety < 0)
                {
                    rety = -dades.positions[4];
                }

                // Actualitzar la posició
                dades.positions[4] += rety;
            }
        }
        else
        {
            Serial.printf("No dataN for nunchuk 2 :(\n");
        }

        // Pausa breu entre lectures per evitar col·lisions
        delay(1000);

        Serial.println("    ");
        Serial.print("servo 1: ");
        Serial.println(dades.positions[1]);
        Serial.print("servo 2: ");
        Serial.println(dades.positions[2]);
        Serial.print("servo 3: ");
        Serial.println(dades.positions[3]);
        Serial.print("servo 4: ");
        Serial.println(dades.positions[4]);
        servoController = dades.positions;
    }
    else
    {
        webServer.handle(dades);
        // Si no es control per nunchuk, imprimeix les dades emmagatzemades
        Serial.println(dades.positions[1]);
        Serial.println(dades.positions[2]);
        Serial.println(dades.positions[3]);
        Serial.println(dades.positions[4]);

        servoController = dades.positions;

        delay(500);
    }

    // Pausa entre iteracions del bucle principal
    delay(200);
}
