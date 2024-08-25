/* 
 *   Copyright (C) 2024 dingmos. All rights reserved.
 */

#include "hal/hal.h"
#include "wireless_tune.h"
#include <Arduino.h>

#define WL_TUNE_MAX_PACKETSIZE  255
// const uint8_t *probe_rsp = (const uint8_t *)"STUDIO_YES";
// const char *probe_msg = "STUDIO_PROBE";
CommanderWirelessGlue::CommanderWirelessGlue( uint16_t localPort) 
  : _localPort(localPort), _packetSize(0), _packetIndex(0), _remotePort(UINT16_MAX)
{
}

CommanderWirelessGlue::~CommanderWirelessGlue() 
{
    // Destructor for cleanup
}

void CommanderWirelessGlue::begin(const char* ssid, const char* password)
{
    // Connect to WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.printf("Connected to WiFi.\n");
    Serial.printf("IP address: %s", WiFi.localIP().toString());
    
    _server = WiFiServer(_localPort);
    _server.begin(_localPort);

}

int CommanderWirelessGlue::handleClientDisconnect() {
    if (_client && !_client.connected()) {

        Serial.printf("client disconnected, %s:%d\n", _client.remoteIP().toString(), 
                            _client.remotePort());
        _client.stop(); // Close the client connection
        _client = WiFiClient(); // Reset the client
        _packetSize = 0;
        _packetIndex = 0;


        return true;
    }
    return false;
}

void CommanderWirelessGlue::acceptClient() {
    if (!_client || !_client.connected()) {
        _client = _server.available(); // Accept a new _client
        if (!_client.connected()) {
            // invalid client
            return;
        }
        Serial.printf("accept new conn, %s:%d\n", _client.remoteIP().toString(), 
                            _client.remotePort());
    }
}

void CommanderWirelessGlue::receivePacket() {

    handleClientDisconnect(); // Check if the client is still connected
    acceptClient();  // Ensure we have a client connected before checking for data

    if (_client && _client.connected()) {
        if (_client.available()) {
            _packetSize = _client.read(_packetBuffer, sizeof(_packetBuffer));
            _packetIndex = 0;
        }
    }
}

int CommanderWirelessGlue::available() {
    if (handleClientDisconnect()) {
        return 0;
    }
    acceptClient();  // Ensure we have a client connected before writing data
    
    if (_packetIndex >= _packetSize) {
        receivePacket();  // Check for a new packet if the current one is fully read
    }
    return _packetSize - _packetIndex;
}

int CommanderWirelessGlue::read() {
    if (_packetIndex < _packetSize) {
        return _packetBuffer[_packetIndex++];
    } else {
        return -1; // No more data to read
    }
}

int CommanderWirelessGlue::peek() {
    if (_packetIndex < _packetSize) {
        return _packetBuffer[_packetIndex];
    } else {
        return -1; // No more data to peek at
    }
}

void CommanderWirelessGlue::flush() {
    if (_client) {
        _client.flush();
    }
}

size_t CommanderWirelessGlue::write(uint8_t data) {
    if (handleClientDisconnect()) {
        return 0;
    }
    acceptClient();  // Ensure we have a client connected before writing data

    if (_client && _client.connected()) {
        return _client.write(data);
    } else {
        return 0; // No _client to send data to
    }
}

size_t CommanderWirelessGlue::write(const uint8_t* buffer, size_t size) {

    if (handleClientDisconnect()) {
        return 0;
    }
    acceptClient();  // Ensure we have a client connected before writing data

    if (_client && _client.connected()) {
        return _client.write(buffer, size);
    } else {
        return 0; // No _client to send data to
    }
}
