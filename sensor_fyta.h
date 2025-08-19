#ifndef _SENSOR_FYTA_H
#define _SENSOR_FYTA_H

#if defined(ESP8266)
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#elif defined(OSPI)
#define CPPHTTPLIB_OPENSSL_SUPPORT
#include "httplib.h"
#endif

#include "ArduinoJson.hpp"
#include "OpenSprinkler.h"

using ArduinoJson::JsonDocument;
using ArduinoJson::DeserializationError;

#define FYTA_URL "https://web.fyta.de"
#if defined(ESP8266)
#define FYTA_URL_LOGIN "https://web.fyta.de/api/auth/login"
#define FYTA_URL_USER_PLANT "https://web.fyta.de/api/user-plant"
#define FYTA_URL_USER_PLANT2 "https://web.fyta.de/api/user-plant/"
#elif defined(OSPI)
#define FYTA_URL_LOGIN "/api/auth/login"
#define FYTA_URL_USER_PLANT "/api/user-plant"
#define FYTA_URL_USER_PLANT2 "/api/user-plant/"
#endif

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
#endif
};

#endif // _SENSOR_FYTA_H