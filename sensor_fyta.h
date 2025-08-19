#ifndef _SENSOR_FYTA_H
#define _SENSOR_FYTA_H

#include "OpenSprinkler.h"

using ArduinoJson::JsonDocument;
using ArduinoJson::DeserializationError;

/**
 * @brief FYTA Public API Client
 * https://fyta-io.notion.site/FYTA-Public-API-d2f4c30306f74504924c9a40402a3afd
 * 
 */
class FytaApi {
public:
    FytaApi(const String& email, const String& password)
        : userEmail(email), userPassword(password) { 
            allocClient();
            authenticate();
        }
    ~FytaApi() {
        freeClient();
    }

    // Authenticate and store token
    bool authenticate();
    // Query sensor values
    bool getSensorData(int plantId, JsonDocument& doc);
    // Get plant list
    bool getPlantList(JsonDocument& doc);
   private:
    void allocClient();
    void freeClient();
    String userEmail;
    String userPassword;
    String authToken;
#if defined(ESP8266)
    WiFiClient *client;
#else
    EthernetClient *client;
#endif
};

#endif // _SENSOR_FYTA_H