#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif
#include "MPU6886s.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <stdlib.h>
#include "FreeFonts.h"
#include <ArduinoJson.h>

//WiFi Parameters
const char* WIFI_SSID = "XXXXXXXX";
const char* WIFI_PASS = "XXXXXXXX";
WebServer server(80); // OTA

// MQTT Parameters
bool MQTT_active = true; // Enable / Disable MQTT

const char* MQTT_SERVER = "XXXXXXXX";
const char* MQTT_PORT = "1883";
const char* MQTT_USER = "XXXXXXXX";
const char* MQTT_PASS = "XXXXXXXX";

#define MQTT_PUB_TOPIC_MAIN "m5seismo"

#define MQTT_PUB_EVENT (MQTT_PUB_TOPIC_MAIN "/event")
#define MQTT_PUB_STATE  (MQTT_PUB_TOPIC_MAIN "/state")
#define MQTT_PUB_AVAILABILITY (MQTT_PUB_TOPIC_MAIN "/status") // online or offline
#define MQTT_PUB_PGA_TRIGGER (MQTT_PUB_TOPIC_MAIN "/pga_trigger") // change pga_trigger
#define MQTT_PUB_COMMAND (MQTT_PUB_TOPIC_MAIN "/command")

// SPK HAT, default disabled
bool SPK_HAT;
const int SPK_pin   = 26;
int spkChannel      = 0;
int spkFreq         = 50;
int spkResolution   = 10;

// Screen coordinates
int8_t lcd_brightness; // TFT backlight brightness for standby ( value: 7 - 15 )
uint8_t graph_x_axis = 7; // X Coordinate for Vertical axis line
uint8_t graph_x_start = 8; // X Coordinate for where the graph starts (increases)
bool continuous_graph; // false: Draw graph only when EQ happens
uint8_t previous_graph_y[3];
#ifdef STICKC
  // Graph coordinates for main screen: M5StickC: 160x80
  uint8_t graph_y_axis[3] = {25,35,45}; // Y coordinates for X,Y,Z horizontal axis lines
  uint8_t graph_y_axis_boundary = 5; // pixels
  uint8_t graph_x_limit = 155; // X Coordinate limit for graph
  uint8_t graph_scale = 50;
  uint8_t graph_clear_y_height = 37;
  uint8_t pga_print_x = 120;
  uint8_t pga_print_y = 0;
  uint8_t mqtt_print_x = 130; // 120 for text size 1
  uint8_t mqtt_print_y = 70;
#else
  // Graph coordinates for main screen: M5StickCPlus: 240x135 pixels
  uint8_t graph_y_axis[3] = {35,65,95}; // Y coordinates for X,Y,Z horizontal axis lines
  uint8_t graph_y_axis_boundary = 15; // pixels
  uint8_t graph_x_limit = 235; // X Coordinate limit for graph
  uint8_t graph_scale = 150;
  uint8_t graph_clear_y_height = 97;
  uint8_t pga_print_x = 200;
  uint8_t pga_print_y = 2;
  uint8_t mqtt_print_x = 200;
  uint8_t mqtt_print_y = 125;
#endif

// MPU6886 Calibration Parameters
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)

// Seismic Measuring Variables
float scale_factor = 1; // scale_factor
uint8_t eq_pet = 40; // Post Event Time (Around 5 secs)
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

AsyncMqttClient mqttClient;

Preferences preferences; // To store and read pga_trigger

MPU6886s seismo;

/* Server Index Page */
 
const char* serverIndex =
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<title>SeismoM5</title>"
"<h1>SeismoM5 OTA</h1>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
"<label id='file-input' for='file'>   Choose file...</label>"
"<input type='submit' class=btn value='Update'>"
"<br><br>"
"<div id='prg'></div>"
"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
"<script>"
"function sub(obj){"
"var fileName = obj.value.split('\\\\');"
"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
"};"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
"$.ajax({"
"url: '/update',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
"$('#bar').css('width',Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!') "
"},"
"error: function (a, b, c) {"
"}"
"});"
"});"
"</script>"
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}h1{color:white;text-align:center;}</style>";

void publish_available();
void publish_state(String state);
void publish_event(int16_t x_mag, int16_t y_mag, int16_t z_mag, float pga_mag);
void publish_pga();
void onMqttPublish(uint16_t packetId);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

void calibrate_MPU() {
  Serial.println("Init MPU6886...");
  publish_state("INIT_MPU");
  seismo.Init();

  publish_state("WAIT");
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

//  Serial.println("Setting MPU6886 Range +/- 2g");
//  seismo.SetAccelFsr(seismo.AFS_2G);

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
}

void onWifiEvent(WiFiEvent_t event) {
  switch (event) {
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected or reconnected to WiFi");
      break;
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(WIFI_SSID);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("Connecting to MQTT...");
      if (MQTT_active) mqttClient.connect();
      break;
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi Disconnected. Restarting...");
      delay(3000);
//      WiFi.setAutoReconnect(true);
      ESP.restart();
//      WiFi.reconnect();
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);   // Initialize serial communication
  M5.begin();
  preferences.begin("seismom5", true);
  pga_trigger = preferences.getFloat("pga_trigger", 0.025);
  SPK_HAT = preferences.getBool("spk_hat", false);
  lcd_brightness = preferences.getUInt("brightness", 7);
  continuous_graph = preferences.getBool("continuous", false);
  flush_period = preferences.getUShort("period", 300);
  preferences.end();

  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);
  
  ledcSetup(spkChannel, spkFreq, spkResolution);
  ledcAttachPin(SPK_pin, spkChannel);
  ledcWriteTone(spkChannel, 0);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setWill(MQTT_PUB_AVAILABILITY, 1, true, "offline");
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);

  Serial.println("WiFi Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  publish_pga();
  publish_event(16384, 0, 0, 0.0000); // Init 1st retain event
  Serial.print("PGA TRIGGER:");
  Serial.println(String(pga_trigger,3).c_str());
  
// OTA Web Server
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();

  calibrate_MPU();

  Serial.println("LISTENING...");
}

void loop() {
  M5.update();
  server.handleClient();
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

/*
  Serial.println("");
  Serial.print("Vectors: ");
  Serial.printf("%.5f , %.5f , %.5f", x_vector_mag,y_vector_mag,z_vector_mag);
  Serial.println("");

  Serial.print("PGA: ");
  Serial.printf("%.5f", pga);
*/
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
      publish_state("DISCONNECT");
      mqttClient.disconnect();
    } else {
      publish_state("LISTENING");
      mqttClient.connect();
    }
  }

  if (M5.BtnB.wasPressed()) {
    ESP.restart();
  }

  if (eq_status) eq_time_count++ ;

  delay(100);
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
  if (String(topic) == MQTT_PUB_STATE) {
    if (String(payload) == "RESET") {
      Serial.print("MQTT Restart Request Received");
      ESP.restart();
    } else if (String(payload) == "UPDATE") {
      Serial.print("MQTT Update Request Received");
      publish_event(ax, ay, az, pga);
      publish_state("LISTENING");
    }
  } else if (String(topic) == MQTT_PUB_COMMAND) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, len); 
    serializeJsonPretty(doc, Serial);
    Serial.println();
    
    if (doc["pga_trigger"] != nullptr) {
      float pga_request = doc["pga_trigger"];
      Serial.print("MQTT Change PGA Trigger Request Received: ");
      Serial.println(pga_request);
      change_pga_trigger(pga_request);
    }

    if (doc["reset"] != nullptr) {
      if (doc["reset"]) {
        Serial.println("MQTT Restart Request Received");
        ESP.restart();
      }
    }

    if (doc["update"] != nullptr) {
      if (doc["update"]) {
      Serial.println("MQTT Update Request Received");
      publish_event(ax, ay, az, pga);
      }
    }

    if (doc["speaker_enable"] != nullptr) {
      SPK_HAT = doc["speaker_enable"];
      preferences.begin("seismom5", false);
      preferences.putBool("spk_hat", SPK_HAT);
      preferences.end();
      Serial.print("MQTT Speaker Request Enabled: ");
      Serial.println(SPK_HAT);
    }

    if (doc["lcd_brightness"] != nullptr) {
      lcd_brightness = doc["lcd_brightness"];
      preferences.begin("seismom5", false);
      preferences.putUInt("brightness", lcd_brightness);
      preferences.end();
      Serial.print("MQTT LCD Standby Brightness: ");
      Serial.println(lcd_brightness);
      M5.Axp.ScreenBreath(lcd_brightness);
    }

    if (doc["continuous_graph"] != nullptr) {
      continuous_graph = doc["continuous_graph"];
      preferences.begin("seismom5", false);
      preferences.putBool("continuous", continuous_graph);
      preferences.end();
      Serial.print("MQTT Continuous Graph Enabled: ");
      Serial.println(continuous_graph);
    }
    
    if (doc["update_period"] != nullptr) {
      flush_period = doc["update_period"];
      preferences.begin("seismom5", false);
      preferences.putUShort("period", flush_period);
      preferences.end();
      Serial.print("MQTT Update Period: ");
      Serial.print(flush_period);
      Serial.println(" secs.");
    }

  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  publish_available();
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_PUB_STATE, 0);
  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_PUB_COMMAND, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
}
