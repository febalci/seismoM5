#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <stdlib.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include "FreeFonts.h"
#include "MPU6886s.h"
#include "config.h"

// WiFi
AsyncWebServer server(80); // OTA
size_t content_len;
bool webSerialEnabled = false;

// Seismic Measuring Variables
uint16_t flush_period; //seconds

bool eq_status = false;
int eq_time_count;
int16_t ax, ay, az;
float xy_vector_mag, x_vector_mag, y_vector_mag, z_vector_mag;
float pga;
/* Peak Ground Acceleration
https://en.wikipedia.org/wiki/Peak_ground_acceleration
https://en.wikipedia.org/wiki/Japan_Meteorological_Agency_seismic_intensity_scale
*/
float pga_trigger; // (g - m/s2)
uint16_t flush_event = 0;
bool flush_update = true;
float pga_request;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

Preferences preferences; // To store and read pga_trigger

MPU6886s seismo;

void connectToWifi();
void connectToMqtt();
void publish_available();
void publish_state(String state);
void publish_event(int16_t x_mag, int16_t y_mag, int16_t z_mag, float pga_mag);
void publish_pga();
void onMqttPublish(uint16_t packetId);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

template<typename T>
void logln(T msg) {
  if (logging == 1 || logging == 3) Serial.println(msg);
  if ((logging == 2 || logging == 3) && webSerialEnabled) WebSerial.println(msg);
}

template<typename T>
void log(T msg) {
  if (logging == 1 || logging == 3) Serial.print(msg);
  if ((logging == 2 || logging == 3) && webSerialEnabled) WebSerial.print(msg);
}

void handleUpdate(AsyncWebServerRequest *request) {
  request->send(200, "text/html", update_html);
}

void handleReset(AsyncWebServerRequest *request) {
  request->send(200, "text/html", "Reset OK");    
  logln("MQTT Restart Request Received");
  ESP.restart();
}

void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index){
    logln("Update");
    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
    response->addHeader("Refresh", "20");  
    response->addHeader("Location", "/");
    request->send(response);
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      logln("Update complete");
      Serial.flush();
      ESP.restart();
    }
  }
}

/*
void printProgress(size_t prg, size_t sz) {
  logf("Progress: %d%%\n", (prg*100)/content_len);
}
*/
String config_processor(const String& var){
  if(var == "PGAPLACEHOLDER"){
    String server_details = "";
    server_details = String(pga_trigger,4);
    return server_details;
  }
  return String();
}

void calibrate_MPU() {
  logln("Init MPU6886...");
  publish_state("INIT_MPU");
  seismo.Init();

  publish_state("WAIT");
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setTextSize(2);

  logln("Prepare for Calibration...");
#ifdef STICKC
  M5.Lcd.setCursor(2, 5);
  M5.Lcd.print("PREPARE FOR");
  M5.Lcd.setCursor(17, 35);
  M5.Lcd.print("CALIBRATION");
  M5.Lcd.drawRect(32,60,100,10,YELLOW);
  for (int i = 0; i <= 9; i++) { // Wait 10 secs
    delay(1000);
    M5.Lcd.fillRect(32+(i*10),60,10,10,YELLOW);
  }
#else
  M5.Lcd.setCursor(60, 25);
  M5.Lcd.print("PREPARE FOR");
  M5.Lcd.setCursor(60, 55);
  M5.Lcd.print("CALIBRATION");
  M5.Lcd.drawRect(72,110,100,15,YELLOW);
  for (int i = 0; i <= 9; i++) { // Wait 10 secs
    delay(1000);
    M5.Lcd.fillRect(72+(i*10),110,10,15,YELLOW);
  }
#endif // 160x80-240x135

//  Serial.println("Setting MPU6886 Range +/- 2g");
//  seismo.SetAccelFsr(seismo.AFS_2G);

  logln("Calibrating...");
  M5.Lcd.fillScreen(BLACK);
#ifdef STICKC
  M5.Lcd.setTextSize(1);
#else
  M5.Lcd.setTextSize(2);
#endif
  M5.Lcd.setCursor(2, 5);
  M5.Lcd.print("Calibrating...");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(RED,BLACK);
  M5.Lcd.setCursor(12, 35);
  M5.Lcd.print("DO NOT MOVE!");
  publish_state("CALIBRATION");

  seismo.calibrateAccel(buffersize,acel_deadzone);

  M5.Axp.ScreenBreath(lcd_brightness);
  M5.Lcd.fillScreen(BLACK);
  publish_state("LISTENING");
}

void draw_graph(float x_vector, float y_vector, float z_vector) {
  int y0[3], y1[3];

  float vectors[3] = {x_vector, y_vector, z_vector};
  if ((continuous_graph) || (not continuous_graph && eq_status)) {
    for (int i = 0; i < 3 ; i++) {
      if (previous_graph_y[i] != 0) {
        y0[i] = previous_graph_y[i];
      } else {
        y0[i] = graph_y_axis[i];
      }
    y1[i] = graph_y_axis[i] - (vectors[i] * graph_scale);
    if (y1[i] > (graph_y_axis[i] + graph_y_axis_boundary)) y1[i] = graph_y_axis[i] + graph_y_axis_boundary;
    if (y1[i] < (graph_y_axis[i] - graph_y_axis_boundary)) y1[i] = graph_y_axis[i] - graph_y_axis_boundary;
    previous_graph_y[i] = y1[i];
    }

    M5.Lcd.fillRect(graph_x_start+1, 15, 20, graph_clear_y_height, BLACK);
    M5.Lcd.drawLine(graph_x_start, y0[0], graph_x_start+1, y1[0], RED);
    M5.Lcd.drawLine(graph_x_start, y0[1], graph_x_start+1, y1[1], GREEN);
    M5.Lcd.drawLine(graph_x_start, y0[2], graph_x_start+1, y1[2], BLUE);

    graph_x_start++;
    if (graph_x_start > graph_x_limit) graph_x_start = graph_x_axis + 1;
  }
}

void eq_happening() {
  eq_status = true;
  publish_event(ax, ay, az, pga);
  eq_time_count = 0;
  M5.Axp.ScreenBreath(15); // Full Brightness
  publish_state("EARTHQUAKE");
  logln("EARTHQUAKE");
  digitalWrite(M5_LED, LOW);
  if (SPK_HAT) ledcWriteTone(spkChannel, 800);
}

void change_pga_trigger(float new_trigger) {
  preferences.begin("seismom5", false);
  preferences.putFloat("pga_trigger", new_trigger);
  preferences.end();
  pga_trigger = new_trigger;
  M5.Lcd.setCursor(pga_print_x,pga_print_y);
  M5.Lcd.print("     ");
  publish_state("CHANGED_PGA_TRIGGER");
  publish_state("LISTENING");
  publish_pga();
}

void onWifiEvent(WiFiEvent_t event) {
  switch (event) {
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED:
      logln("Connected or reconnected to WiFi");
      break;
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP:
      logln("");
      log("Connected to ");
      logln(WIFI_SSID);
      log("IP address: ");
      logln(WiFi.localIP());
      if (MQTT_active) connectToMqtt();
      break;
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      logln("WiFi Disconnected. Restarting...");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
//      delay(3000);
//      WiFi.setAutoReconnect(true);
//      ESP.restart();
//      WiFi.reconnect();
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);   // Initialize serial communication
  M5.begin();

  preferences.begin("seismom5", false);
  pga_trigger = preferences.getFloat("pga_trigger", 0.025);
  SPK_HAT = preferences.getBool("spk_hat", false);
  lcd_brightness = preferences.getUInt("brightness", 7);
  continuous_graph = preferences.getBool("continuous", false);
  flush_period = preferences.getUShort("period", 60);
  preferences.end();

  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);
  
  ledcSetup(spkChannel, spkFreq, spkResolution);
  ledcAttachPin(SPK_pin, spkChannel);
  ledcWriteTone(spkChannel, 0);
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setWill(MQTT_PUB_AVAILABILITY, 1, true, "offline");
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  connectToWifi();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    log(".");
  }
  WebSerial.begin(&server);
  webSerialEnabled = true;

  publish_pga();
  publish_event(16384, 0, 0, 0.0000); // Init 1st retain event
  log("PGA TRIGGER:");
  logln(String(pga_trigger,3).c_str());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {request->send_P(200, "text/html", index_html,config_processor);});
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
  server.on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {handleDoUpdate(request, filename, index, data, len, final);}
  );
  server.on("/rt", HTTP_GET, [](AsyncWebServerRequest *request){handleReset(request);});
  server.on("/cp", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("new_pga")) {
      if (request->getParam("new_pga")->value() != "") {
        change_pga_trigger(request->getParam("new_pga")->value().toFloat());
        request->send(200, "text/html", "PGA Trigger Changed");
      }
    }
    request->redirect("/");
  });
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});

  server.begin();
//  Update.onProgress(printProgress);

  calibrate_MPU();

  logln("LISTENING...");
}

void loop() {
  M5.update();
  seismo.getAccelAdc(&ax, &ay, &az);

  // EQ ends
  if (eq_time_count > eq_pet) {
    eq_status = false;
    eq_time_count = 0;

    publish_state("LISTENING");
    publish_event(ax, ay, az, pga);

    digitalWrite(M5_LED, HIGH);
    if (SPK_HAT) ledcWriteTone(spkChannel, 0);
    M5.Axp.ScreenBreath(lcd_brightness);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE,BLACK);
  }

  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(GREEN,BLACK);
  M5.Lcd.setCursor(2, pga_print_y);
  M5.Lcd.print("EARTHQUAKE SENSOR");
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setCursor(pga_print_x,pga_print_y);
  M5.Lcd.printf("%.3f", pga_trigger);

  if (mqttClient.connected()) {
    M5.Lcd.setTextColor(YELLOW,PURPLE);
    M5.Lcd.setCursor(mqtt_print_x,mqtt_print_y);
    M5.Lcd.print("MQTT");
  } else {
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.setCursor(mqtt_print_x,mqtt_print_y);
    M5.Lcd.print("    ");
  }
  
  // Draw Graph Axis
  M5.Lcd.drawFastVLine(7,15,graph_clear_y_height+3,WHITE);
  // Draw Tickmarks
  for (int i = 0; i < 3 ; i++) {
    M5.Lcd.drawFastHLine(5,graph_y_axis[i],2,WHITE);
  }

  x_vector_mag = ax / 16384.0 - 1.0;
  y_vector_mag = ay / 16384.0;
  z_vector_mag = az / 16384.0;
  
  draw_graph(x_vector_mag,y_vector_mag,z_vector_mag);

  pga = sqrt(x_vector_mag * x_vector_mag + y_vector_mag * y_vector_mag + z_vector_mag * z_vector_mag)*scale_factor;

  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setCursor(2, mqtt_print_y);
  M5.Lcd.print("PGA: ");
  M5.Lcd.setCursor(mqtt_print_x-35, mqtt_print_y);
  M5.Lcd.print("(g)");
  M5.Lcd.fillRect(29,mqtt_print_y-7,mqtt_print_x-68,14,BLACK);
  M5.Lcd.setCursor(30, mqtt_print_y+5);
  M5.Lcd.setFreeFont(FSSB9);  // Select Free Sans Serif Bold 9pt font
  M5.Lcd.printf("%.4f", pga);
  M5.Lcd.setTextFont(1);

// EARTHQUAKE!!!
  if (pga >= pga_trigger) {
    eq_happening();
  }

  if (eq_status) {
    publish_event(ax, ay, az, pga);
  } else {
      if (flush_update) {
      flush_event++;
      if (flush_event > (flush_period*10)) {
        publish_event(ax, ay, az, pga);
        flush_event = 0;
      }
    }
  }

  if (M5.BtnA.wasPressed()) {
    if (mqttClient.connected()) {
      publish_state("MQTT_BUTTON_DISCONNECT");
      MQTT_active = false;
      xTimerStop(mqttReconnectTimer, 0);
      mqttClient.disconnect();
    } else {
//      connectToMqtt();
      MQTT_active = true;
      connectToMqtt();
      publish_state("MQTT_BUTTON_CONNECT");
      publish_state("LISTENING");
    }
  }

  if (M5.BtnB.wasPressed()) {
    ESP.restart();
  }

  if (eq_status) eq_time_count++ ;

  delay(100);
}

void connectToWifi() {
  logln("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void connectToMqtt() {
  logln("Connecting to MQTT...");
  mqttClient.connect();
}

void publish_available() {
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_AVAILABILITY, 1, true, String("online").c_str());
}

void publish_state(String state) {
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_STATE, 1, true, state.c_str());
}

void publish_event(int16_t x_mag, int16_t y_mag, int16_t z_mag, float pga_mag) {
  String msg = "{\"x\":\""+ String(x_mag) + "\",\"y\":\"" + String(y_mag) + "\",\"z\":\"" + String(z_mag) + "\",\"pga\":\""+ String(pga_mag,4).c_str() +"\"}";
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_EVENT, 0, true, msg.c_str());
}

void publish_pga() {
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_PGA_TRIGGER, 1, true, String(pga_trigger,3).c_str());
}

void onMqttPublish(uint16_t packetId) {

}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (String(topic) == MQTT_PUB_COMMAND) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, len); 
    serializeJsonPretty(doc, Serial);
    
    if (doc["pga_trigger"] != nullptr) {
      pga_request = doc["pga_trigger"];
      log("MQTT Change PGA Trigger Request Received: ");
      logln(pga_request);
      change_pga_trigger(pga_request);
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
      publish_event(ax, ay, az, pga);
      }
    }

    if (doc["speaker_enable"] != nullptr) {
      SPK_HAT = doc["speaker_enable"];
      preferences.begin("seismom5", false);
      preferences.putBool("spk_hat", SPK_HAT);
      preferences.end();
      log("MQTT Speaker Request Enabled: ");
      logln(SPK_HAT);
    }

    if (doc["lcd_brightness"] != nullptr) {
      lcd_brightness = doc["lcd_brightness"];
      preferences.begin("seismom5", false);
      preferences.putUInt("brightness", lcd_brightness);
      preferences.end();
      log("MQTT LCD Standby Brightness: ");
      logln(lcd_brightness);
      M5.Axp.ScreenBreath(lcd_brightness);
    }

    if (doc["continuous_graph"] != nullptr) {
      continuous_graph = doc["continuous_graph"];
      preferences.begin("seismom5", false);
      preferences.putBool("continuous", continuous_graph);
      preferences.end();
      log("MQTT Continuous Graph Enabled: ");
      logln(continuous_graph);
    }
    
    if (doc["update_period"] != nullptr) {
      flush_period = doc["update_period"];
      preferences.begin("seismom5", false);
      preferences.putUShort("period", flush_period);
      preferences.end();
      log("MQTT Update Period: ");
      log(String(flush_period));
      logln(" secs.");
    }
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