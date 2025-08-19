#include "sensor_fyta.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.hpp>

using ArduinoJson::JsonDocument;
using ArduinoJson::StaticJsonDocument;
using ArduinoJson::DeserializationError;

/**
 * @brief FYTA Public API Client
 * https://fyta-io.notion.site/FYTA-Public-API-d2f4c30306f74504924c9a40402a3afd
 * 
 */
bool FytaApi::authenticate() {
    HTTPClient http;
    String url = "https://web.fyta.de/api/auth/login";
    http.begin(*client, url);
    http.addHeader("Content-Type", "application/json");
    JsonDocument payload;
    payload["email"] = userEmail;
    payload["password"] = userPassword;
    char requestBody[128];
    serializeJson(payload, requestBody, sizeof(requestBody));
    authToken = "";

    int res = http.POST(requestBody);
    if (res == 200) {
        JsonDocument responseDoc;
        DeserializationError error = deserializeJson(responseDoc, http.getString());
        if (!error && responseDoc.containsKey("token")) {
            authToken = responseDoc["token"].as<String>();
            http.end();
            return true;
        }
    }
    http.end();
    return false;
}

// Query sensor values
bool FytaApi::getSensorData(int plantId, JsonDocument& doc) {
    if (authToken.isEmpty()) return false;
    HTTPClient http;
    String url = "https://web.fyta.de/api/user-plant/" + plantId;
    http.begin(*client, url);
    http.addHeader("Authorization", "Bearer " + authToken);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.GET();
    if (httpCode == 200) {
        DeserializationError error = deserializeJson(doc, http.getString());
        http.end();
        return !error;
    }
    http.end();
    return false;
}

bool FytaApi::getPlantList(JsonDocument& doc) {
    if (authToken.isEmpty()) return false;      
    HTTPClient http;
    String url = "https://web.fyta.de/api/user-plant";
    http.begin(*client, url);
    http.addHeader("Authorization", "Bearer " + authToken);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.GET();
    if (httpCode == 200) {
        DeserializationError error = deserializeJson(doc, http.getString());   
        if (!error && doc.containsKey("plants")) {
            http.end();
            return true;    
        }
    }
    http.end();
    return false;
}

void FytaApi::allocClient() {
#if defined(ESP8266)
    WiFiClientSecure *_c = new WiFiClientSecure();
    _c->setInsecure();
    _c->setBufferSizes(512, 512); 
    client = _c;
#else
	EthernetClient *client = new EthernetClientSsl();
#endif   
}

void FytaApi::freeClient() {
    if (client) {
        delete client;
        client = nullptr;
    }   
}