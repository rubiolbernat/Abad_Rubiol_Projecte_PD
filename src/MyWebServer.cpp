#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif

#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "SPIFFS.h"
#include <map>

// #include "automation.h"

struct SDades
{
    // bool nunchukStatus;
    bool record;
    std::map<int, float> positions;
};

class MyWebServer
{
private:
    const char *ssid;
    const char *password;
    AsyncWebServer server;

    SDades dades;

    // Variables per emmagatzemar l'estat del dimmer i dels botons
    int dimmerValue;
    bool nunchukStatus;
    bool socketStatus;

    // Processor for index page template
    String indexPageProcessor(const String &var)
    {
        String status = "";
        if (var == "NUNCHUK_BUTTON_STATUS")
        {
            if (nunchukStatus)
            {
                status = "checked";
            }
        }
        else if (var == "SOCKET_TEMPLATE_STATUS")
        {
            status = "checked";
        }
        else if (var == "DIMMER_VALUE")
        {
            status = String(dimmerValue);
        }
        return status;
    }

    void notFound(AsyncWebServerRequest *request)
    {
        request->send(404, "text/plain", "Not found");
    }

public:
    MyWebServer(const char *ssid, const char *password) : ssid(ssid), password(password), server(80), dimmerValue(0), socketStatus(false) {}

    void begin()
    {
        Serial.begin(115200);
        Serial.println("Connecting to ");
        Serial.println(ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        if (WiFi.waitForConnectResult() != WL_CONNECTED)
        {
            Serial.printf("WiFi Failed!\n");
            return;
        }

        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        // Initialize SPIFFS
        if (!SPIFFS.begin(true))
        {
            Serial.println("An Error has occurred while mounting SPIFFS");
            return;
        }

        // initialize our automation
        server.on("/", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
            Serial.println("Requesting index page...");
            request->send(SPIFFS, "/index.html", "text/html", false, std::bind(&MyWebServer::indexPageProcessor, this, std::placeholders::_1)); });

        // Route to load entireframework.min.css file
        server.on("/css/entireframework.min.css", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(SPIFFS, "/css/entireframework.min.css", "text/css"); });

        // Route to load toggle-switchy.css file
        server.on("/css/toggle-switchy.css", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(SPIFFS, "/css/toggle-switchy.css", "text/css"); });

        // Route to load custom.css file
        server.on("/css/custom.css", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(SPIFFS, "/css/custom.css", "text/css"); });

        // Route to load custom.js file
        server.on("/js/custom.js", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(SPIFFS, "/js/custom.js", "text/javascript"); });

        // server toggle request for "nunchuk"
        server.on("/toggle/nunchuk-button", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
                      StaticJsonDocument<100> data;
                      if (request->hasParam("status"))
                      {
                          String status = request->getParam("status")->value();
                          bool nunchukStatus = (status == "true");
                          this->nunchukStatus = nunchukStatus;
                          data["success"] = "true";
                          data["current_nunchuk_status"] = /*dades.*/this->nunchukStatus;
                      }
                      else
                      {
                          data["isError"] = "true";
                          data["error_description"] = "No nunchuk status parameter was sent by the client!";
                      }

                      AsyncResponseStream *response = request->beginResponseStream("application/json");
                      serializeJson(data, *response);
                      request->send(response); });

        // server toggle request for "socket"
        server.on("/toggle/socket", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
                      StaticJsonDocument<100> data;
                      if (request->hasParam("status"))
                      {
                          String status = request->getParam("status")->value();
                          bool socketStatus = (status == "true");
                          this->socketStatus = socketStatus;
                          data["success"] = "true";
                          data["current_socket_status"] = this->socketStatus;
                      }
                      else
                      {
                          data["isError"] = "true";
                          data["error_description"] = "No socket status parameter was sent by the client!";
                      }
                      Serial.println("socket toggle ");
                      AsyncResponseStream *response = request->beginResponseStream("application/json");
                      serializeJson(data, *response);
                      request->send(response); });

        // server toggle request for "servo1"
        server.on("/servo1/change", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
                      StaticJsonDocument<100> data;
                      if (request->hasParam("value"))
                      {
                          String value = request->getParam("value")->value();
                          this -> dades.positions[1] = value.toInt();
                          data["success"] = "true";
                          data["current_servo1_value"] = this->dades.positions[1];
                          //Serial.print("Servo1 Value: ");
                          //Serial.println(value);
                      }
                      else
                      {
                          data["isError"] = "true";
                          data["error_description"] = "No dimmer value parameter was sent by the client!";
                      }

                      AsyncResponseStream *response = request->beginResponseStream("application/json");
                      serializeJson(data, *response);
                      request->send(response); });

        // server toggle request for "servo2"
        server.on("/servo2/change", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
                      StaticJsonDocument<100> data;
                      if (request->hasParam("value"))
                      {
                          String value = request->getParam("value")->value();
                          this -> dades.positions[2] = value.toInt();
                          data["success"] = "true";
                          data["current_servo2_value"] = this->dades.positions[2];
                          //Serial.print("Servo2 Value: ");
                          //Serial.println(value);
                      }
                      else
                      {
                          data["isError"] = "true";
                          data["error_description"] = "No dimmer value parameter was sent by the client!";
                      }

                      AsyncResponseStream *response = request->beginResponseStream("application/json");
                      serializeJson(data, *response);
                      request->send(response); });

        // server toggle request for "servo3"
        server.on("/servo3/change", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
                      StaticJsonDocument<100> data;
                      if (request->hasParam("value"))
                      {
                          String value = request->getParam("value")->value();
                          this -> dades.positions[3] = value.toInt();
                          data["success"] = "true";
                          data["current_servo3_value"] = this->dades.positions[3];
                          //Serial.print("Servo3 Value: ");
                          //Serial.println(value);
                      }
                      else
                      {
                          data["isError"] = "true";
                          data["error_description"] = "No dimmer value parameter was sent by the client!";
                      }

                      AsyncResponseStream *response = request->beginResponseStream("application/json");
                      serializeJson(data, *response);
                      request->send(response); });

        // server toggle request for "servo4"
        server.on("/servo4/change", HTTP_GET, [&](AsyncWebServerRequest *request)
                  {
                      StaticJsonDocument<100> data;
                      if (request->hasParam("value"))
                      {
                          String value = request->getParam("value")->value();
                          this -> dades.positions[4] = value.toInt();
                          data["success"] = "true";
                          data["current_servo4_value"] = this->dades.positions[4];
                          //Serial.print("Servo4 Value: ");
                          //Serial.println(value);
                      }
                      else
                      {
                          data["isError"] = "true";
                          data["error_description"] = "No dimmer value parameter was sent by the client!";
                      }

                      AsyncResponseStream *response = request->beginResponseStream("application/json");
                      serializeJson(data, *response);
                      request->send(response); });

        server.onNotFound(std::bind(&MyWebServer::notFound, this, std::placeholders::_1));

        server.begin();
    }

    // Mètodes per a obtenir l'estat del dimmer i dels botons
    int getDimmerValue() const
    {
        return dimmerValue;
    }

    bool getnunchukStatus() const
    {
        return nunchukStatus;
    }
    bool getSocketStatus() const
    {
        return socketStatus;
    }

    void handle(SDades &outdades)
    {
        outdades = dades;
        // Pots posar aquí el teu codi per actualitzar l'estat del dimmer i dels botons
        // Exemple:
        // dimmerValue = proto.getDimmerValue();
        // tubeStatus = proto.isTubeOn();
        // bulbStatus = proto.isBulbOn();
        // socketStatus = proto.isSocketOn();
        /*Serial.print("ps 2 value: ");
        Serial.println(dades.positions[2]);
        /*
        Serial.print("Button 11 value: ");
        Serial.println(dades.nunchukStatus);
        Serial.print("Button 2 value: ");
        Serial.println(tubeStatus);
        Serial.print("Slider value: ");
        Serial.println(bulbStatus);*/
    }
    void set_handle(SDades &outdades)
    {
        dades = outdades;

        // Construeix un missatge SSE amb els valors dels servos
        String sseMessage = "data: ";
        for (int i = 1; i <= 4; ++i)
        {
            sseMessage += "servo" + String(i) + "=" + String(dades.positions[i]) + "\n";
        }
        sseMessage += "\n";

        // Envia el missatge SSE a tots els clients connectats
        server.on("/updates", HTTP_GET, [sseMessage](AsyncWebServerRequest *request)
                  { request->send(200, "text/event-stream", sseMessage); });
    }
};