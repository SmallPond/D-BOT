#include "udp_comm.h"

namespace iot {

UDPComm::UDPComm(const std::string& ssid, const std::string& password, uint16_t localPort)
    : ssid_(ssid), password_(password), local_port_(localPort) {}

UDPComm::~UDPComm() {
    udp_.stop();
}

int UDPComm::Init() {
    // Check if WiFi is already connected
    if (WiFi.status() == WL_CONNECTED) {
        log_i("WiFi already connected. Starting UDP on port %d...", local_port_);
        udp_.begin(local_port_);
        return 0;
    }
    log_i("netwrok is not connected, skip udp comm init");
    return -1;
    /*
    // Attempt to connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_.c_str(), password_.c_str());

    log_i("Connecting to WiFi...");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > 10000) { // Timeout after 10 seconds
            Serial.println("\nFailed to connect to WiFi");
            return -1;
        }
        delay(500);
        Serial.print(".");
    }

    log_i("\nWiFi connected");
    log_i("IP Address: %s", WiFi.localIP().toString().c_str());

    // Start UDP
    udp_.begin(local_port_);
    */

}

int UDPComm::Send(const JsonDocument& json) {
    char buffer[256];
    serializeJson(json, buffer);

    udp_.beginPacket(udp_.remoteIP(), udp_.remotePort());
    udp_.write(reinterpret_cast<const uint8_t*>(buffer), strlen(buffer));
    udp_.endPacket();
    return 0;
}

void UDPComm::SetRecvCallback(RecvCallback callback) {
    recvCallback_ = callback;
}

void UDPComm::Loop() {
    int packetSize = udp_.parsePacket();
    if (packetSize) {
        char buffer[packetSize + 1];
        udp_.read(buffer, packetSize);
        buffer[packetSize] = '\0'; // Ensure null-terminated string
        log_i("receved udp msg: %s", buffer);
        JsonDocument jsonDoc;
        deserializeJson(jsonDoc, buffer);

        if (recvCallback_) {
            recvCallback_(jsonDoc);
        }
    }
}

} // namespace iot