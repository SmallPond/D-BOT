#include "hal/hal.h"

#if XK_MQTT

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "config.h"
#include "hal/nvs.h"

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
int mqtt_last_connect_time = 0;
const char *g_topic_prefix = "dingmos";

const char *HAL::mqtt_get_topic_frefix(void)
{
    return g_topic_prefix;
}

int HAL::setup_wifi(void) 
{
    int retry_cnt = 0;
    int max_retry_cnt = 30;
    const char *ssid = HAL::get_wifi_ssid().c_str();
    if (WiFi.status() == WL_CONNECTED) {
        log_i("WiFi already connected.\n");
        return 0;
    }

    log_i("connecting to WiFi..: %s\n", ssid);
    WiFi.begin(ssid, HAL::get_wifi_passwd().c_str());
    
    while (WiFi.status() != WL_CONNECTED && retry_cnt < max_retry_cnt) {
        Serial.print(".");
        retry_cnt++;
        delay(100);
    }

   if (WiFi.status() == WL_CONNECTED) {
        log_i("Connected to network %s\n", ssid);
   } else {
        log_e("WiFi connection timeout: %s\n", ssid);
        return -1;
   }
   return 0;
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "Received mqtt callback for topic %s, length %u", topic, length);
    Serial.printf(buf);
}

void connectMQTT(void)
{
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", "");
    String id = "DBOT_" + macAddress.substring(macAddress.length() - 6);
    bool result;

    struct mqtt_config conf;
    if (nvs_get_mqtt_config(&conf)) {
        return;
    }

    log_i("Attempting MQTT connection...id %s", id.c_str());

    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    // if(conf.mqtt_password.length() > 0){
    //     result = mqtt_client.connect(id.c_str(), conf.mqtt_username.c_str(),
    //                                          conf.mqtt_password.c_str());
    // } else {
    result = mqtt_client.connect(id.c_str());
    // }

    if (!result) {
        printf("MQTT failed rc=%d will try again in 5 seconds\n", mqtt_client.state());
    }
}


TaskHandle_t handleTaskMqtt;
void TaskMqttUpdate(void *pvParameters) {

    while(1) {
        long now = millis();
        if (!mqtt_client.connected() && (now - mqtt_last_connect_time) > 5000) {
            log_e("Reconnecting MQTT");
            mqtt_last_connect_time = now;
            connectMQTT();
        }
        mqtt_client.loop();
        delay(5);
    }
}

int HAL::mqtt_subscribe(const char *topic)
{
    bool ret = mqtt_client.subscribe(topic);
    if (ret) {
        log_i("Subscribe Error to topic:%s\n", topic);
    }
    return ret;
}

int HAL::mqtt_publish(const char *topic, const char *playload)
{
    bool ret = mqtt_client.publish(topic, playload);
    if (ret) {
        Serial.printf("Publish Error: topic:%s, playload:%s\n", topic, playload);
    }
    return ret;
}
#define MQTT_SIZE  50


void HAL::mqtt_init(void) {
    struct mqtt_config conf;
    static char mqtt_host[MQTT_SIZE];
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    if (nvs_get_mqtt_config(&conf)) {
        log_e("no mqtt conf");
        return;
    }

    g_topic_prefix = strdup(conf.mqtt_topic_prefix.c_str());
    
    log_i("MQTT connect host |%s|:|%d|\n", conf.mqtt_host.c_str(), conf.mqtt_port);
    snprintf(mqtt_host, MQTT_SIZE, conf.mqtt_host.c_str());

    mqtt_client.setServer(mqtt_host, conf.mqtt_port);
    mqtt_client.setCallback(mqttCallback);
    xTaskCreatePinnedToCore(
        TaskMqttUpdate,
        "MqttThread",
        4096,
        nullptr,
        2,
        &handleTaskMqtt,
        ESP32_RUNNING_CORE
    );
}
#else 
void HAL::mqtt_init(void) {

}

int HAL::mqtt_publish(const char *topic, const char *playload) {
    return 0;
}

int HAL::mqtt_subscribe(const char *topic){

}
#endif