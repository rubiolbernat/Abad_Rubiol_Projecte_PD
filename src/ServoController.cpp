#include <Arduino.h>
#include <map>
#include <ESP32Servo.h>

class ServoController
{
private:
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    int servo1Pin;
    int servo2Pin;
    int servo3Pin;
    int servo4Pin;
    int minUs;
    int maxUs;
    int pos;
    ESP32PWM pwm;

public:
    ServoController(int s1Pin, int s2Pin, int s3Pin, int s4Pin) : servo1Pin(s1Pin), servo2Pin(s2Pin), servo3Pin(s3Pin), servo4Pin(s4Pin),
                                                                  minUs(500), maxUs(2500), pos(0) {}

    void setup()
    {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
        servo1.setPeriodHertz(50);
        servo2.setPeriodHertz(50);
        servo3.setPeriodHertz(50);
        servo4.setPeriodHertz(50);
        Serial.begin(115200);
    }

    void attachServos()
    {
        servo1.attach(servo1Pin, minUs, maxUs);
        servo2.attach(servo2Pin, minUs, maxUs);
        servo3.attach(servo3Pin, minUs, maxUs);
        servo4.attach(servo4Pin, minUs, maxUs);
    }

    void detachServos()
    {
        servo1.detach();
        servo2.detach();
        servo3.detach();
        servo4.detach();
    }

    void attachPWM(int pwmPin, int frequency)
    {
        pwm.attachPin(pwmPin, frequency);
    }

    void detachPWM(int pwmPin)
    {
        pwm.detachPin(pwmPin);
    }

    void sweepServo()
    {
        for (pos = 0; pos <= 180; pos += 1)
        {
            servo1.write(pos);
            Serial.println(pos);
            delay(100);
        }
        servo1.write(90);
    }

    void set_S1Pos(float rot)
    {
        servo1.write(rot);
    }
    void set_S2Pos(float rot)
    {
        servo2.write(rot);
    }
    void set_S3Pos(float rot)
    {
        servo3.write(rot);
    }
    void set_S4Pos(float rot)
    {
        servo4.write(rot);
    }

    // Operador d'assignació
    ServoController &operator=(const std::map<int, float> &positionsMap)
    {
        for (const auto &pair : positionsMap)
        {
            int servoID = pair.first;
            float rotation = pair.second;
            // Assignem la rotació al servo corresponent
            switch (servoID)
            {
            case 1:
                set_S1Pos(rotation);
                break;
            case 2:
                set_S2Pos(rotation);
                break;
            case 3:
                set_S3Pos(rotation);
                break;
            case 4:
                set_S4Pos(rotation);
                break;
            default:
                // Maneig de l'error si la ID del servo no és vàlida
                break;
            }
        }
        return *this; // Retorna una referència a l'objecte actual
    }
};