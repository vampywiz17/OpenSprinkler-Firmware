#ifndef _SENSOR_FYTA_H
#define _SENSOR_FYTA_H

#if defined(ESP8266) || defined(OSPI)

#if defined(ESP8266)
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#elif defined(OSPI)
#include "naett.h"
#endif

#include "ArduinoJson.hpp"
#include "OpenSprinkler.h"

using ArduinoJson::JsonDocument;
using ArduinoJson::DeserializationError;

#define FYTA_URL "https://web.fyta.de"
#define FYTA_URL_LOGIN "https://web.fyta.de/api/auth/login"
#define FYTA_URL_USER_PLANT "https://web.fyta.de/api/user-plant"
#define FYTA_URL_USER_PLANTF "https://web.fyta.de/api/user-plant/%lu"

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
    bool getSensorData(ulong plantId, JsonDocument& doc);
    // Get plant list
    bool getPlantList(JsonDocument& doc);
private:
    void allocClient();
    void freeClient();
#if defined(ESP8266)
    String userEmail;
    String userPassword;
    String authToken;
    WiFiClient *client;
#else
    std::string userEmail;
    std::string userPassword;
    std::string authToken;
#endif
};

#endif // defined(ESP8266) || defined(OSPI)
#endif // _SENSOR_FYTA_H