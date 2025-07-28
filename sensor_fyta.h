#pragma once

#include <ArduinoJson.hpp>
#include <HTTPService.h>

class FytaApi {
public:
    FytaApi(const String& email, const String& password)
        : userEmail(email), userPassword(password) {}

    // Authenticate and store token
    bool authenticate() {
        HTTPService http(new ConnectionInfo{
            .serverUrl = "https://web.fyta.de",
            .authToken = "",
            .dbVersion = 2,
            .httpOptions = HTTPOptions()
        });
        const int capacity = 4 * JSON_OBJECT_SIZE(3);
        StaticJsonBuffer<capacity> payload;
        JsonObject& obj = payload.createObject();
        payload["email"] = userEmail;
        payload["password"] = userPassword;
        char requestBody[128];
        obj.printTo(requestBody, sizeof(requestBody));

        int httpCode = http.doPOST("/api/auth/login", requestBody,
        if (httpCode == 200) {
            StaticJsonDocument<512> responseDoc;
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
    bool getSensorData(const String& sensorId, JsonDocument& doc) {
        if (authToken.isEmpty()) return false;
        HTTPClient http;
        String url = "https://api.fyta.com/v1/sensors/" + sensorId;
        http.begin(url);
        http.addHeader("Authorization", "Bearer " + authToken);

        int httpCode = http.GET();
        if (httpCode == 200) {
            DeserializationError error = deserializeJson(doc, http.getString());
            http.end();
            return !error;
        }
        http.end();
        return false;
    }

private:
    String userEmail;
    String userPassword;
    String authToken;
};