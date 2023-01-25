#ifndef _mqtt_H_
#define _mqtt_H_

#include "config.h"
#include "util.h"
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

AsyncMqttClient mqttClient;

TimerHandle_t mqttReconnectTimer;

void connectToMqtt() {
  logln("Connecting to MQTT...");
  mqttClient.connect();
}

void publish_available() {
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_AVAILABILITY, 1, true, String("online").c_str());
}

void publish_mqtt(const char* topic, const char* msg, uint8_t qos, bool retain) {
  uint16_t packetIdPub1 = mqttClient.publish(topic, qos, retain, msg);
  logln(msg);
}

void onMqttPublish(uint16_t packetId) {

}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (String(topic) == MQTT_PUB_COMMAND) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, len); 
    serializeJsonPretty(doc, Serial);
    
    if (doc["pga_trigger"] != nullptr) {
      float pga_request = doc["pga_trigger"];
      pga_trigger_changed = pga_request;
    }

    if (doc["reset"] != nullptr) {
      if (doc["reset"]) {
        logln("MQTT Restart Request Received");
        ESP.restart();
      }
    }

    if (doc["update"] != nullptr) {
      if (doc["update"]) {
        logln("MQTT Update Request Received");
        String msg = "{\"x\":\""+ String(ax) + "\",\"y\":\"" + String(ay) + "\",\"z\":\"" + String(az) + "\",\"pga\":\""+ String(pga,4).c_str() +"\"}";
        publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);
      }
    }

    if (doc["speaker_enable"] != nullptr) {
      SPK_HAT = doc["speaker_enable"];
    }

    if (doc["lcd_brightness"] != nullptr) {
      lcd_brightness = doc["lcd_brightness"];
      M5.Axp.ScreenBreath(lcd_brightness);
    }

    if (doc["continuous_graph"] != nullptr) {
      continuous_graph = doc["continuous_graph"];
    }

    if (doc["update_period"] != nullptr) {
      flush_period = doc["update_period"];
    }
    
    pref_update();
  }
}

void onMqttConnect(bool sessionPresent) {
  logln("Connected to MQTT.");
  publish_available();
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_PUB_STATE, 0);
  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_PUB_COMMAND, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  logln("Disconnected from MQTT.");
  if (WiFi.isConnected() && MQTT_active) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void initMqtt() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setWill(MQTT_PUB_AVAILABILITY, 1, true, "offline");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
}

void stopMqttTimer() {
  xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
}
#endif
