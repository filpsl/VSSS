/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        communication.cpp                            Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/

#include "communication.h"

/**
 * @brief Constructor of the Communication class.
 * 
 * @param ssid WiFi network name (SSID).
 * @param password WiFi network password.
 * @param port Port used for TCP/IP communication.
 */
Communication::Communication(const char* ssid, const char* password, uint32_t port)
    : ssid_(ssid), password_(password), port_(port), server_(port) {}

/**
 * @brief Initializes WiFi communication and starts the TCP/IP server.
 * 
 * Attempts to connect the ESP32 to the provided WiFi network. Upon successful connection,
 * the TCP/IP server is started on the specified port.
 */
void Communication::begin() {
    WiFi.begin(ssid_, password_);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    server_.begin();
}

/**
 * @brief Receives data from a client connected to the server.
 * 
 * This function checks if a client is connected to the server. If connected,
 * it reads the data sent by the client. If there is no client or no data available,
 * it returns the value 0xFFFFFFFF to indicate the absence of data.
 * 
 * @return uint32_t Data received from the client. If no data is available, returns 0xFFFFFFFF.
 */
uint32_t Communication::receiveData() {
    if (!client_ || !client_.connected()) {
        client_ = server_.available();
        if (!client_) {
            return 0xFFFFFFFF; // Invalid value, no client connected
        }
    }
    if (client_.available()) {
        uint32_t value = 0;
        client_.read(reinterpret_cast<uint8_t*>(&value), sizeof(value));
        return value;
    }
    return 0xFFFFFFFF; // Invalid value, no data available
}

uint32_t Communication::waitData() {
    if (!client_ || !client_.connected()) {
        client_ = server_.available();
        if (!client_) {
            return 0xFFFFFFFF; // Invalid value, no client connected
        }
    }
    if (client_.available()) {
        uint32_t value = 0;
        while(!client_.read(reinterpret_cast<uint8_t*>(&value), sizeof(value))){
          delayMicroseconds(1);
        }
        return value;
    }
    return 0xFFFFFFFF; // Invalid value, no data available
}

/**a
 * @brief Sends data to a client connected to the server.
 * 
 * Checks if a client is connected. If a client is connected, the data is sent
 * to the client via TCP/IP.
 * 
 * @param value Data to be sent to the client.
 */
void Communication::sendData(uint32_t value) {
    if (client_ && client_.connected()) {
        client_.write(reinterpret_cast<uint8_t*>(&value), sizeof(value));
    }
}
