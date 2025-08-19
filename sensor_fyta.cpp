#include "sensor_fyta.h"

using ArduinoJson::JsonDocument;
using ArduinoJson::StaticJsonDocument;
using ArduinoJson::DeserializationError;

/**
 * @brief FYTA Public API Client
 * https://fyta-io.notion.site/FYTA-Public-API-d2f4c30306f74504924c9a40402a3afd
 *
 * httplib: https://github.com/yhirose/cpp-httplib
 * 
 */
bool FytaApi::authenticate() {
    JsonDocument payload;
    payload["email"] = userEmail;
    payload["password"] = userPassword;
    char requestBody[128];
    serializeJson(payload, requestBody, sizeof(requestBody));
    authToken = "";

#if defined(ESP8266)
    HTTPClient http;
    http.begin(*client, FYTA_URL_LOGIN);
    http.addHeader("Content-Type", "application/json");
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
#elif defined(OSPI)
    httplib::Client http(FYTA_URL, 443);
    auto res = http.Post(FYTA_URL_LOGIN, requestBody, "application/json");
    if (res && res->status == 200) {
        JsonDocument responseDoc;
        DeserializationError error = deserializeJson(responseDoc, res->body);   
        if (!error && responseDoc.containsKey("token")) {
            authToken = responseDoc["token"].as<String>();
            return true;
        }
    }
#endif
    return false;
}

// Query sensor values
bool FytaApi::getSensorData(int plantId, JsonDocument& doc) {

#if defined(ESP8266)
    if (authToken.isEmpty()) return false;
    HTTPClient http;
    String url = FYTA_URL_USER_PLANT2 + String(plantId);
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
#elif defined(OSPI)
    if (authToken.empty()) return false;
    httplib::Client http(FYTA_URL, 443);
    httplib::Headers headers = {
        {"Authorization", "Bearer " + authToken},
        {"Content-Type", "application/json"}
    };
    std::string url = FYTA_URL_USER_PLANT2 + std::to_string(plantId);
    auto res = http.Get(url, headers);
    if (res && res->status == 200) {
        DeserializationError error = deserializeJson(doc, res->body);
        if (!error) {
            return true;
        }
    }
#endif
    return false;
}

bool FytaApi::getPlantList(JsonDocument& doc) {

#if defined(ESP8266)
    if (authToken.isEmpty()) return false;      
    HTTPClient http;
    http.begin(*client, FYTA_URL_USER_PLANT);
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
#elif defined(OSPI)
    if (authToken.empty()) return false;      
    httplib::Client http(FYTA_URL, 443);
    httplib::Headers headers = {
        {"Authorization", "Bearer " + authToken},
        {"Content-Type", "application/json"}
    };
    auto res = http.Get(FYTA_URL_USER_PLANT, headers);
    if (res && res->status == 200) {
        DeserializationError error = deserializeJson(doc, res->body);
        if (!error && doc.containsKey("plants")) {
            return true;
        }
    }
#endif
    return false;
}

void FytaApi::allocClient() {
#if defined(ESP8266)
    WiFiClientSecure *_c = new WiFiClientSecure();
    _c->setInsecure();
    _c->setBufferSizes(512, 512); 
    client = _c;
#endif   
}

void FytaApi::freeClient() {
#if defined(ESP8266)
    if (client) {
        delete client;
        client = nullptr;
    }   
#endif
}