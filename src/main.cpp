#include <M5StickC.h>
#include "MPU6886s.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>

bool MQTT_active = true; // Enable / Disable MQTT

//WiFi Parameters
const char* WIFI_SSID = "XXXXXXXX";
const char* WIFI_PASS = "XXXXXXXX";

// MQTT Parameters
const char* MQTT_SERVER = "XXXXXXXX";
const char* MQTT_PORT = "1883";
const char* MQTT_USER = "XXXXXXXX";
const char* MQTT_PASS = "XXXXXXXX";
#define MQTT_PUB_EVENT "m5seismo/event"
#define MQTT_PUB_STATE  "m5seismo/state"
#define MQTT_PUB_AVAILABILITY "m5seismo/status" // online or offline

AsyncMqttClient mqttClient;

// SPK HAT, Comment out this line if no SPK HAT is used
#define SPK_HAT  

#ifdef SPK_HAT
  const int SPK_pin   = 26;
  int spkChannel      = 0;
  int spkFreq         = 50;
  int spkResolution   = 10;
# endif

// Graph - 160 x 80 TFT Screen; Coordinates for main screen:
int8_t lcd_brightness=8; // TFT backlight brightness for standby ( value: 7 - 15 )
uint8_t graph_x_axis = 7;
uint8_t graph_y_axis[3] = {25,35,45};
uint8_t graph_y_axis_boundary = 5; // pixels
uint8_t graph_x_start = 8;
uint8_t graph_scale = 50;
bool continuous_graph = false; //Draw only when EQ happens
uint8_t previous_graph_y[3];

// MPU6886 Calibration Parameters
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)

// Seismic Measuring Variables
float scale_factor = 1; // scale_factor 
bool eq_status = false;
int eq_time_count;
uint8_t eq_pet = 40; // Post Event Time (Around 5 secs)
int16_t ax, ay, az;
float xy_vector_mag, x_vector_mag, y_vector_mag, z_vector_mag; 
float pga; // Peak Ground Acceleration
// https://en.wikipedia.org/wiki/Peak_ground_acceleration
// https://en.wikipedia.org/wiki/Japan_Meteorological_Agency_seismic_intensity_scale
float pga_trigger = 0.025; // (g - m/s2)

MPU6886s seismo;

void publish_available();
void publish_state(String state);
void publish_event(String x_mag, String y_mag, String z_mag, String pga_mag);
void onMqttPublish(uint16_t packetId);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

void calibrate_MPU() {
  Serial.println("Init MPU6886...");
  publish_state("INIT_MPU");
  seismo.Init();

  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(2, 5);
  M5.Lcd.print("PREPARE FOR");
  M5.Lcd.setCursor(17, 35);
  M5.Lcd.print("CALIBRATION");
  publish_state("WAIT");
  //Progress bar
  M5.Lcd.drawRect(32,60,100,10,YELLOW);
  for (int i = 0; i <= 9; i++) { // Wait 10 secs
    delay(1000);
    M5.Lcd.fillRect(32+(i*10),60,10,10,YELLOW);
  }

  Serial.println("Setting MPU6886 Range +/- 2g");
  seismo.SetAccelFsr(seismo.AFS_2G);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
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

    M5.Lcd.fillRect(graph_x_start+1, 15, 20, 40, BLACK);
    M5.Lcd.drawLine(graph_x_start, y0[0], graph_x_start+1, y1[0], RED);
    M5.Lcd.drawLine(graph_x_start, y0[1], graph_x_start+1, y1[1], GREEN);
    M5.Lcd.drawLine(graph_x_start, y0[2], graph_x_start+1, y1[2], BLUE);

    graph_x_start++;
    if (graph_x_start > 155) graph_x_start = graph_x_axis + 1;
  }
}

void eq_happening() {
  eq_status = true;
  publish_event(String(ax),String(ay),String(az),String(pga));
  eq_time_count = 0;
  M5.Axp.ScreenBreath(15); // Full Brightness
  publish_state("EARTHQUAKE");
  digitalWrite(M5_LED, LOW);
#ifdef SPK_HAT
  ledcWriteTone(spkChannel, 800);
#endif
}

void setup() {
  Serial.begin(115200);   // Initialize serial communication
  M5.begin();

  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);
#ifdef SPK_HAT
  ledcSetup(spkChannel, spkFreq, spkResolution);
  ledcAttachPin(SPK_pin, spkChannel);
#endif
  Serial.println("WiFi Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Connecting to MQTT...");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setWill(MQTT_PUB_AVAILABILITY, 1, true, "offline");
  mqttClient.setServer(MQTT_SERVER, atoi(MQTT_PORT));
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  if (MQTT_active) mqttClient.connect();

  calibrate_MPU();
}

void loop() {
  M5.update();
  seismo.getAccelAdc(&ax, &ay, &az);

  // EQ ends
  if (eq_time_count > eq_pet) {
    eq_status = false;
    eq_time_count = 0;

    publish_state("LISTENING");

    digitalWrite(M5_LED, HIGH);
#ifdef SPK_HAT
    ledcWriteTone(spkChannel, 0);
#endif
    M5.Axp.ScreenBreath(lcd_brightness); 
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE,BLACK);
  }

  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(GREEN,BLACK);
  M5.Lcd.setCursor(2, 0);
  M5.Lcd.print("EARTHQUAKE SENSOR");
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setCursor(137,0);
  M5.Lcd.print("   ");
  M5.Lcd.setCursor(137,0);
  M5.Lcd.print((int)scale_factor);

  if (mqttClient.connected()) {
    M5.Lcd.setTextColor(YELLOW,PURPLE);
    M5.Lcd.setCursor(132,70);
    M5.Lcd.print("MQTT");
  } else {
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.setCursor(132,70);
    M5.Lcd.print("    ");
  }

  // Draw Graph Axis
  M5.Lcd.drawFastVLine(7,15,40,WHITE);
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
  M5.Lcd.setCursor(2, 60);
  M5.Lcd.printf("PGA: %.5f  (g)", pga);


// EARTHQUAKE!!!
//  if (xy_vector_mag >= 1.0 || z_abs_vector_mag >= 1.0) {
  if (pga >= pga_trigger) {
    eq_happening();
  }

  if (eq_status) {
    publish_event(String(ax),String(ay),String(az),String(pga));
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

void publish_event(String x_mag, String y_mag, String z_mag, String pga_mag) {
    String msg = "{\"x\":\""+x_mag + "\",\"y\":\"" + y_mag + "\",\"z\":\"" + z_mag + "\",\"pga\":\""+ pga_mag +"\"}";
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_EVENT, 0, true, msg.c_str());
}

void onMqttPublish(uint16_t packetId) {

}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if ( (String(topic) == MQTT_PUB_STATE) && (String(payload) == "RESET")) {
      Serial.print("MQTT Restart Request Received");
    ESP.restart();
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  publish_available();
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_PUB_STATE, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
}
