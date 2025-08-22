#include "sensor_fyta.h"

#if defined(ESP8266) || defined(OSPI)

using ArduinoJson::JsonDocument;
using ArduinoJson::DeserializationError;

#if defined(OSPI)
static bool fyta_init = false;
#endif

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
#if defined(ESP8266) 
    String requestBody;
#else
    std::string requestBody;
#endif
    serializeJson(payload, requestBody);
    authToken = "";
    DEBUG_PRINTLN("FYTA AUTH");
    DEBUG_PRINTLN(requestBody.c_str());

#if defined(ESP8266)
    HTTPClient http;
    http.begin(*client, FYTA_URL_LOGIN);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("accept", "application/json");
    int res = http.POST(requestBody.c_str());
    if (res == 200) {
        JsonDocument responseDoc;
        DeserializationError error = deserializeJson(responseDoc, http.getString());
        if (!error && responseDoc.containsKey("access_token")) {
            authToken = responseDoc["access_token"].as<String>();
            http.end();
            return true;
        }
    }
    http.end();
#elif defined(OSPI)
    naettReq* req =
        naettRequest(FYTA_URL_LOGIN,
            naettMethod("POST"),
            naettHeader("accept", "application/json"),
            naettHeader("Content-Type", "application/json"),
            naettBody(requestBody.c_str(), requestBody.length()));

    naettRes* res = naettMake(req);
    while (!naettComplete(res)) {
        usleep(100 * 1000);
    }

    if (naettGetStatus(res) < 0) {
        DEBUG_PRINTLN("Request failed");
        naettFree(req);
        return false;
    }

    int bodyLength = 0;
    const char* body = (char*)naettGetBody(res, &bodyLength);
    JsonDocument responseDoc;
    DeserializationError error = deserializeJson(responseDoc, body, bodyLength);
    if (!error && responseDoc.containsKey("access_token")) {
        authToken = responseDoc["access_token"].as<String>();
    }
    naettClose(res);
    naettFree(req);
#endif
    DEBUG_PRINTLN("AUTH-TOKEN:");
    DEBUG_PRINTLN(authToken.c_str());
    return true;
}

// Query sensor values
bool FytaApi::getSensorData(ulong plantId, JsonDocument& doc) {
    DEBUG_PRINTLN("FYTA getSensorData");
#if defined(ESP8266)
    if (authToken.isEmpty()) return false;
    HTTPClient http;
    char url[50];
    sprintf(url, FYTA_URL_USER_PLANTF, plantId);
    DEBUG_PRINTLN(url);
    http.begin(*client, url);
    http.addHeader("Authorization", "Bearer " + authToken);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("accept", "application/json");
    int httpCode = http.GET();
    if (httpCode == 200) {
        DeserializationError error = deserializeJson(doc, http.getString());
        http.end();
        return !error;
    }
    http.end();
    return false;
#elif defined(OSPI)
    if (authToken.empty()) return false;
    std::string auth = "Bearer " + authToken;
    char url[50];
    sprintf(url, FYTA_URL_USER_PLANTF, plantId);
    DEBUG_PRINTLN(url);
    naettReq* req =
        naettRequest(url,
            naettMethod("GET"),
            naettHeader("accept", "application/json"),
            naettHeader("Content-Type", "application/json"),
            naettHeader("Authorization", auth.c_str()));

    naettRes* res = naettMake(req);
    while (!naettComplete(res)) {
        usleep(100 * 1000);
    }

    int bodyLength = 0;
    const char* body = (char*)naettGetBody(res, &bodyLength);
    DeserializationError error = deserializeJson(doc, body, bodyLength);
    if (naettGetStatus(res) < 0 || !body || !bodyLength || error) {
        DEBUG_PRINTLN("FYTA Request failed");
        naettClose(res);
        naettFree(req);
        return false;
    }
    DEBUG_PRINTLN("FYTA getSensorData OK");
    naettClose(res);
    naettFree(req);
    return true;
#endif
}

bool FytaApi::getPlantList(JsonDocument& doc) {
    DEBUG_PRINTLN("FYTA getPlantList");
#if defined(ESP8266)
    if (authToken.isEmpty()) return false;
    HTTPClient http;
    http.begin(*client, FYTA_URL_USER_PLANT);
    http.addHeader("Authorization", "Bearer " + authToken);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("accept", "application/json");

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
#elif defined(OSPI)
    if (authToken.empty()) return false;
    std::string auth = "Bearer " + authToken;
    naettReq* req =
        naettRequest(FYTA_URL_USER_PLANT,
            naettMethod("GET"),
            naettHeader("accept", "application/json"),
            naettHeader("Content-Type", "application/json"),
            naettHeader("Authorization", auth.c_str()));

    naettRes* res = naettMake(req);
    while (!naettComplete(res)) {
        usleep(100 * 1000);
    }

    int bodyLength = 0;
    const char* body = (char*)naettGetBody(res, &bodyLength);
    DEBUG_PRINTLN(body);
    DeserializationError error = deserializeJson(doc, body, bodyLength);
    if (naettGetStatus(res) < 0 || !body || !bodyLength || error) {
        DEBUG_PRINTLN("FYTA Request failed!");
        naettClose(res);
        naettFree(req);
        return false;
    }

    DEBUG_PRINTLN("FYTA getPlantList OK");

    naettClose(res);
    naettFree(req);

    return true;
#endif
}

bool FytaApi::getPlantThumb(ulong plantId, JsonDocument& doc) {
    DEBUG_PRINTLN("FYTA getPlantThumb");
#if defined(ESP8266)
    if (authToken.isEmpty()) return false;
    HTTPClient http;
    DEBUG_PRINTLN(doc["thumb_path"]);
    http.begin(*client, doc["thumb_path"]);
    http.addHeader("Authorization", "Bearer " + authToken);
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.GET();
    if (httpCode == 200) {
        doc["thumb"] = http.getString();
        http.end();
        return true;
    }
    http.end();
    return false;
#elif defined(OSPI)
    if (authToken.empty()) return false;
    std::string auth = "Bearer " + authToken;
    DEBUG_PRINTLN(doc["thumb_path"]);
    naettReq* req =
        naettRequest(doc["thumb_path"],
            naettMethod("GET"),
            naettHeader("Content-Type", "application/json"),
            naettHeader("Authorization", auth.c_str()));

    naettRes* res = naettMake(req);
    while (!naettComplete(res)) {
        usleep(100 * 1000);
    }

    int bodyLength = 0;
    const char* body = (char*)naettGetBody(res, &bodyLength);
    doc["thumb"] = std::string(body, bodyLength);
    if (naettGetStatus(res) < 0 || !body || !bodyLength || error) {
        DEBUG_PRINTLN("FYTA Request failed");
        naettClose(res);
        naettFree(req);
        return false;
    }
    DEBUG_PRINTLN("FYTA getSensorData OK");
    naettClose(res);
    naettFree(req);
    return true;
#endif
}

void FytaApi::allocClient() {
#if defined(ESP8266)
    WiFiClientSecure *_c = new WiFiClientSecure();
    _c->setInsecure();
    _c->setBufferSizes(512, 512);
    client = _c;
#elif defined(OSPI)
    if (!fyta_init) {
        fyta_init = true;
        naettInit(NULL);
    }
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

#endif // defined(ESP8266) || defined(OSPI)