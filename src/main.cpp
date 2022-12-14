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
#include "FreeFonts.h"
#include "MPU6886s.h"
#include "config.h"
#include "util.h"
#include "mqtt.h"

// WiFi
AsyncWebServer server(80); // OTA
size_t content_len;

TimerHandle_t wifiReconnectTimer;

MPU6886s seismo;

void updateState(const char* state) {
  log("State: ");
  logln(state);
  publish_mqtt(MQTT_PUB_STATE, state, 1, true);
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
  }
  return server_details;
}

void calibrate_MPU() {
  updateState("INIT_MPU");
  seismo.Init();
  updateState("WAIT");

  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setTextSize(2);

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
  updateState("CALIBRATION");

  seismo.calibrateAccel(buffersize,acel_deadzone);

  M5.Axp.ScreenBreath(lcd_brightness);
  M5.Lcd.fillScreen(BLACK);
  updateState("LISTENING");
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
  String msg = "{\"x\":\""+ String(ax) + "\",\"y\":\"" + String(ay) + "\",\"z\":\"" + String(az) + "\",\"pga\":\""+ String(pga,4).c_str() +"\"}";
  publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);

  eq_time_count = 0;
  M5.Axp.ScreenBreath(15); // Full Brightness
  updateState("EARTHQUAKE");
  digitalWrite(M5_LED, LOW);
  if (SPK_HAT) ledcWriteTone(spkChannel, 800);
}

void connectToWifi() {
  logln("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
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
      stopMqttTimer();
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

  pref_init(); // Read Parameters

  pinMode(M5_LED, OUTPUT); // Setup M5 Red LED
  digitalWrite(M5_LED, HIGH);
    
  ledcSetup(spkChannel, spkFreq, spkResolution); // Setup SPK_HAT
  ledcAttachPin(SPK_pin, spkChannel);
  ledcWriteTone(spkChannel, 0);
  
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  initMqtt(); //Initialize MQTT Server Parameters

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  connectToWifi();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    log(".");
  }

  WebSerial.begin(&server);
  webSerialEnabled = true;

  publish_mqtt(MQTT_PUB_PGA_TRIGGER, String(pga_trigger,3).c_str(), 1, true);
  String msg = "{\"x\":\"16384\",\"y\":\"0\",\"z\":\"0\",\"pga\":\"0.0000\"}"; // Init 1st retain event
  publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);

  log("PGA TRIGGER:");
  logln(String(pga_trigger,3).c_str());
  pga_trigger_changed = pga_trigger;

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {request->send_P(200, "text/html", index_html,config_processor);});
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
  server.on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {handleDoUpdate(request, filename, index, data, len, final);}
  );
  server.on("/rt", HTTP_GET, [](AsyncWebServerRequest *request){handleReset(request);});
  server.on("/sv", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->getParam("new_pga")->value() != "") {
      pga_trigger_changed = request->getParam("new_pga")->value().toFloat();
    }
    if (request->getParam("new_bri")->value() != "") {
      lcd_brightness = request->getParam("new_bri")->value().toInt();
      M5.Axp.ScreenBreath(lcd_brightness);
    }
    if (request->getParam("new_per")->value() != "") {
      flush_period = request->getParam("new_per")->value().toInt();
    }
    if (request->hasParam("spk")) {
      if (request->getParam("spk")->value() == "on") SPK_HAT = true; else SPK_HAT = false;
    } else SPK_HAT = false;
    if (request->hasParam("con")) {
      if (request->getParam("con")->value() == "on") continuous_graph = true; else continuous_graph = false;
    } else continuous_graph = false;
    if (request->getParam("lg")->value() != "") {
      logging = (request->getParam("lg")->value().toInt());
    }
    pref_update();
    request->send(200, "text/html", "Settings Changed");
    request->redirect("/");
  });
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});

  server.begin();

  calibrate_MPU();

  updateState("LISTENING");
}

void loop() {
  M5.update();
  seismo.getAccelAdc(&ax, &ay, &az);

  // EQ ends
  if (eq_time_count > eq_pet) {
    eq_status = false;
    eq_time_count = 0;

    updateState("LISTENING");

    String msg = "{\"x\":\""+ String(ax) + "\",\"y\":\"" + String(ay) + "\",\"z\":\"" + String(az) + "\",\"pga\":\""+ String(pga,4).c_str() +"\"}";
    publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);

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
  if (pga_trigger_changed != pga_trigger) {
    pga_trigger = pga_trigger_changed;
    M5.Lcd.setCursor(pga_print_x,pga_print_y);
    M5.Lcd.print("     ");
    updateState("CHANGED_PGA_TRIGGER");
    publish_mqtt(MQTT_PUB_PGA_TRIGGER, String(pga_trigger,3).c_str(), 1, true);
    updateState("LISTENING");
  }
  M5.Lcd.setCursor(pga_print_x,pga_print_y);
  M5.Lcd.printf("%.4f", pga_trigger);

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
    String msg = "{\"x\":\""+ String(ax) + "\",\"y\":\"" + String(ay) + "\",\"z\":\"" + String(az) + "\",\"pga\":\""+ String(pga,4).c_str() +"\"}";
    publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);
  } else {
      if (flush_update) {
      flush_event++;
      if (flush_event > (flush_period*10)) {
        String msg = "{\"x\":\""+ String(ax) + "\",\"y\":\"" + String(ay) + "\",\"z\":\"" + String(az) + "\",\"pga\":\""+ String(pga,4).c_str() +"\"}";
        publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);
        flush_event = 0;
      }
    }
  }

  if (M5.BtnA.wasPressed()) {
    if (mqttClient.connected()) {
      updateState("MQTT_BUTTON_DISCONNECT");
      MQTT_active = false;
      stopMqttTimer();
      mqttClient.disconnect();
    } else {
//      connectToMqtt();
      MQTT_active = true;
      connectToMqtt();
      delay(500);
      updateState("MQTT_BUTTON_CONNECT");
      publish_mqtt(MQTT_PUB_PGA_TRIGGER, String(pga_trigger,3).c_str(), 1, true);
      String msg = "{\"x\":\"16384\",\"y\":\"0\",\"z\":\"0\",\"pga\":\"0.0000\"}"; // Init 1st retain event
      publish_mqtt(MQTT_PUB_EVENT, msg.c_str(), 0, true);
      updateState("LISTENING");
    }
  }

  if (M5.BtnB.wasPressed()) {
    ESP.restart();
  }

  if (eq_status) eq_time_count++ ;

  delay(100);
}

