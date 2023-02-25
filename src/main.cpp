#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <stdlib.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "MPU6886s.h"
#include "config.h"
#include "util.h"
#include "mqtt.h"
#include "stalta.h"
#include "screen.h"

AsyncWebServer server(80);

MPU6886s seismo;

STA_LTA staLta(trigger_threshold,detrigger_threshold);

screen lcd;

void updateState(const char* state) {
  publish_mqtt(MQTT_PUB_STATE, state, 1, true);
}

String updateEvent(int16_t _x, int16_t _y, int16_t _z, float _pga) {
  String msg = "{\"x\":\""+ String(_x) + "\",\"y\":\"" + String(_y) + "\",\"z\":\"" + String(_z) + "\",\"pga\":\""+ String(_pga,4).c_str() +"\"}";
  return msg;
}

void handleReset(AsyncWebServerRequest *request) {
  request->send(200, "text/html", "Reset OK");    
  logln("MQTT Restart Request Received");
  ESP.restart();
}

void calibrate_MPU() {
  updateState("INIT_MPU");
  seismo.Init();
  if (!calibrating) {
    updateState("WAIT");
    lcd.show_wait();
  }

  lcd.show_calibration();
  updateState("CALIBRATION");

  seismo.calibrateAccel(buffersize,acel_deadzone);

  calibrating = false;
  delay(100);
  seismo.getAccelAdc(&ax, &ay, &az);
  publish_mqtt(MQTT_PUB_EVENT, updateEvent(ax,ay,az,pga).c_str(), 0, true);
  updateState("LISTENING");
  lcd.clear_screen();
  lcd.set_brightness(lcd_brightness);
  lcd.draw_axis();
  lcd.show_title();
}

String config_processor(const String& var){
  String server_details = "";
  if (var == "PGAPLACEHOLDER"){
    server_details = String(pga_trigger,4);
  } else if (var == "BRIPLACEHOLDER"){
    server_details = String(lcd_brightness);
  } else if (var == "PERPLACEHOLDER"){
    server_details = String(flush_period);
  } else if (var == "SPKPLACEHOLDER"){
    server_details = SPK_HAT;
  } else if (var == "CONPLACEHOLDER"){
    server_details = continuous_graph;
  } else if (var == "LOGPLACEHOLDER"){
    server_details = logging;
  } else if (var == "SLTAPLACEHOLDER"){
    server_details = slta;
  } else if (var == "MASTERPLACEHOLDER"){
    server_details = master;
  } else if (var == "SLAVEPLACEHOLDER"){
    server_details = slave;
  } 
  return server_details;
}

void eq_happening() {
  eq_status = true;
  eq_time_count = 0;
  lcd.set_brightness(15); // Full Brightness
  updateState("EARTHQUAKE");
  digitalWrite(M5_LED, LOW);
  if (SPK_HAT) ledcWriteTone(spkChannel, 800);
}

void connectToWifi() {
  logln("Connecting to Wi-Fi...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void onWifiEvent(WiFiEvent_t event) {
}

void checkConnection() {
  if (!WiFi.isConnected() && (conn_stat != 1)) { conn_stat = 0; }
  if (MQTT_active) {
    if (WiFi.isConnected() && !mqttClient.connected() && (conn_stat != 3))  { conn_stat = 2; }
    if (WiFi.isConnected() && mqttClient.connected() && (conn_stat != 5)) { conn_stat = 4;}
  } else {
    if (WiFi.isConnected() && (conn_stat == 1))  { conn_stat = 4; }
  }
  switch (conn_stat) {
    case 0:                                                       // MQTT and WiFi down: start WiFi
      logln("[MQTT] and [WiFi] down: start [WiFi]");
      connectToWifi();
      conn_stat = 1;
      waitCount = 0;
      break;
    case 1:                                                       // WiFi starting, do nothing here
      logln("[WiFi] starting, wait : "+ String(waitCount));
      waitCount++;
      if (waitCount > WIFI_RECONNECT_TIMER) {
        ESP.restart();
      }
      break;
    case 2:                                                       // WiFi up, MQTT down: start MQTT
      logln("[WiFi] up, [MQTT] down: start [MQTT]");
      if (MQTT_active) connectToMqtt();
      conn_stat = 3;
      waitCount = 0;
      break;
    case 3:                                                       // WiFi up, MQTT starting, do nothing here
      logln("[WiFi] up, [MQTT] starting, wait : "+ String(waitCount));
      waitCount++;
      break;
    case 4:                                                       // WiFi up, MQTT up: finish MQTT configuration
      if (MQTT_active) {
        logln("[WiFi] up, [MQTT] up: Connected");
      } else {
        logln("[WiFi] up, [MQTT] button disabled: Connected");
      }
      conn_stat = 5;  
      waitCount = 0;                  
      break;
  }
}

void setup() {
  Serial.begin(115200);   // Initialize serial communication
  M5.begin();

#ifdef SLAVE
  slave = true;
#else
  slave = false;
#endif

  pref_init(); // Read Parameters
  pinMode(M5_LED, OUTPUT); // Setup M5 Red LED
  digitalWrite(M5_LED, HIGH);

  ledcSetup(spkChannel, spkFreq, spkResolution); // Setup SPK_HAT
  ledcAttachPin(SPK_pin, spkChannel);
  ledcWriteTone(spkChannel, 0);
  lcd.show_wifi_connect();  
  initMqtt(); //Initialize MQTT Server Parameters

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  WiFi.setHostname(WIFI_HOSTNAME);
  connectToWifi();

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    waitCount++;
    if (waitCount > (WIFI_RECONNECT_TIMER)) {
      ESP.restart();
    }
  }
  
  if (MQTT_active) connectToMqtt();

  WebSerial.begin(&server);
  webSerialEnabled = true;

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname(WIFI_HOSTNAME);
  ArduinoOTA.begin();
  MDNS.addService("http", "tcp", 80);

  pga_trigger_changed = pga_trigger;

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {request->send_P(200, "text/html", index_html,config_processor);});
  server.on("/rt", HTTP_GET, [](AsyncWebServerRequest *request){handleReset(request);});
  server.on("/recal", HTTP_GET, [](AsyncWebServerRequest *request){
    calibrating = true;
    request->send(200, "text/html", "Recalibrating MPU");
    });
  server.on("/sv", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("new_pga") && (request->getParam("new_pga")->value() != "")) {
      pga_trigger_changed = request->getParam("new_pga")->value().toFloat();
    }
    if (request->hasParam("new_bri") && (request->getParam("new_bri")->value() != "")) {
      lcd_brightness = request->getParam("new_bri")->value().toInt();
      lcd.set_brightness(lcd_brightness);
    }
    if (request->hasParam("new_per") && (request->getParam("new_per")->value() != "")) {
      flush_period = request->getParam("new_per")->value().toInt();
    }
    if (request->hasParam("spk")) {
      if (request->getParam("spk")->value() == "on") SPK_HAT = true; else SPK_HAT = false;
    } else SPK_HAT = false;
    if (request->hasParam("slta")) {
      if (request->getParam("slta")->value() == "on") slta = true; else slta = false;
    } else slta = false;
    if (request->hasParam("con")) {
      if (request->getParam("con")->value() == "on") continuous_graph = true; else continuous_graph = false;
    } else continuous_graph = false;
    if (request->hasParam("lg") && (request->getParam("lg")->value() != "")) {
      logging = (request->getParam("lg")->value().toInt());
    }
    if (request->hasParam("master")) {
      if (request->getParam("master")->value() == "on") master = true; else master = false;
    } else master = false;
    pref_update();
    request->send(200, "text/html", "Settings Changed");
    request->redirect("/");
  });
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});

  server.begin();
  calibrate_MPU();
}

void loop() {
  checkConnection();
  ArduinoOTA.handle();
  if (!calibrating) {
    M5.update();
    seismo.getAccelAdc(&ax, &ay, &az);

    if (slta) {
      lcd.show_method(slta, true);
    } else {
      if (pga_trigger_changed != pga_trigger) {
        pga_trigger = pga_trigger_changed;
        lcd.show_method(false, true);
        updateState("CHANGED_PGA_TRIGGER");
        publish_mqtt(MQTT_PUB_PGA_TRIGGER, String(pga_trigger,4).c_str(), 1, true);
        updateState("LISTENING");
      }
      lcd.show_method(false,false,pga_trigger);
    }

    lcd.show_mqtt(mqttClient.connected());
//    lcd.draw_axis();

    x_vector_mag = ax / gravity - 1.0;
    y_vector_mag = ay / gravity;
    z_vector_mag = az / gravity;

    pga = sqrt(x_vector_mag * x_vector_mag + y_vector_mag * y_vector_mag + z_vector_mag * z_vector_mag)*scale_factor;

// For Helicorder:
// Serial Studio
//    String msg = "/*"+ String(ax) + "," + String(ay) +"," + String(az) + ","+ String(pga,4).c_str() +"*/"; 
// Jamasesis - Select ASCII 4
//    String msg = String(ax) + " , " + String(ay) +" , " + String(az) + " , " + String(int(pga*10000)).c_str() ;
//    Serial.println(msg);

    if ((continuous_graph) || (not continuous_graph && eq_status)){
      if (slta) {
        lcd.draw_slta_graph(staLta.getSTA(), staLta.getLTA(), pga);
      } else {
        lcd.draw_acc_graph(x_vector_mag,y_vector_mag,z_vector_mag);
      }
    } 
//    if ((continuous_graph) || (not continuous_graph && eq_status)) lcd.draw_pga_graph(pga);

    if (slta) {
      staLta.updateData(pga);
      lcd.show_stalta(staLta.getSTA(),staLta.getLTA());
      if (staLta.checkTrigger()) {
        if ((master && slave_eq_state) || !master) {
          eq_happening();
        }
      }
    } else {
      lcd.show_pga(pga);
      if (pga >= pga_trigger) {
        if ((master && slave_eq_state) || !master) {
          eq_happening();
        }
      }
    }

    if (eq_status) { // Earthquake still happening
      eq_time_count++ ;

      if (((slta && !staLta.checkTrigger()) || (!slta && eq_time_count > eq_pet)) || (master && !slave_eq_state)) { // EQ Ends...              
          eq_status = false;
          eq_time_count = 0;

          updateState("LISTENING");

          publish_mqtt(MQTT_PUB_EVENT, updateEvent(ax,ay,az,pga).c_str(), 0, true);

          digitalWrite(M5_LED, HIGH);
          if (SPK_HAT) ledcWriteTone(spkChannel, 0);
          lcd.set_brightness(lcd_brightness);
      } else {
        if (eq_time_count%2) publish_mqtt(MQTT_PUB_EVENT, updateEvent(ax,ay,az,pga).c_str(), 0, true);
      }
    } else {
        if (flush_update) { // Send periodic event update message
        flush_event++;
        if (flush_event > (flush_period*10)) {
          publish_mqtt(MQTT_PUB_EVENT, updateEvent(ax,ay,az,pga).c_str(), 0, true);
          flush_event = 0;
        }
      }
    }

    if (M5.BtnA.wasPressed()) {
      if (mqttClient.connected()) {
        updateState("MQTT_BUTTON_DISCONNECT");
        MQTT_active = false;
        mqttClient.disconnect();
      } else {
        updateState("MQTT_BUTTON_CONNECT");
        MQTT_active = true;
      }
    }

    if (M5.BtnB.wasPressed()) {
      ESP.restart();
    }

    delay(100);    
  } else {
    calibrate_MPU();
  }
}


