#include "util.h"

bool eq_status = false;
uint16_t flush_event = 0;
bool flush_update = true;
bool calibrating = false;
uint8_t logging = 3;

// Seismic Measuring Variables
int16_t ax, ay, az;
float xy_vector_mag, x_vector_mag, y_vector_mag, z_vector_mag;
float pga;
int eq_time_count;

// Parameters
bool SPK_HAT;
int8_t lcd_brightness; // TFT backlight brightness for standby ( value: 7 - 15 )
float pga_trigger, pga_trigger_changed; // (g - m/s2)
uint16_t flush_period; //seconds
bool continuous_graph; // false: Draw graph only when EQ happens
bool slta; // true: use STA/LTA Method instead of trigger

bool webSerialEnabled; // Disable until WebSerial is loaded

Preferences preferences; // To store and read parameters



void pref_init() {
  preferences.begin("seismom5", false);
  pga_trigger = preferences.getFloat("pga_trigger", 0.025);
  SPK_HAT = preferences.getBool("spk_hat", false);
  lcd_brightness = preferences.getChar("brightness", 7);
  continuous_graph = preferences.getBool("continuous", false);
  flush_period = preferences.getUShort("period", 60);
  slta = preferences.getBool("stalta", false);
  preferences.end();
  logln("---------------  PREFERENCES READ ---------------");
  log("PGA Trigger: "); logln(pga_trigger);
  log("STA/LTA Method: "); logln(slta);
  log("SPK_HAT: "); logln(SPK_HAT);
  log("Lcd Stanby Brightness(7-15): "); logln(lcd_brightness);
  log("Continuous Graph: "); logln(continuous_graph);
  log("Event Values Update Period(secs): "); logln(flush_period);
  logln("-------------------------------------------------");
}

void pref_update() {
  preferences.begin("seismom5", false);
  preferences.putFloat("pga_trigger", pga_trigger_changed);
  preferences.putBool("spk_hat", SPK_HAT);
  preferences.putChar("brightness", lcd_brightness);
  preferences.putBool("continuous", continuous_graph);
  preferences.putUShort("period", flush_period);
  preferences.putBool("stalta", slta);
  preferences.end();
  logln("--------------  PREFERENCES UPDATE --------------");
  log("PGA Trigger: "); logln(pga_trigger_changed);
  log("STA/LTA Method: "); logln(slta);
  log("SPK_HAT: "); logln(SPK_HAT);
  log("Lcd Stanby Brightness(7-15): "); logln(lcd_brightness);
  log("Continuous Graph: "); logln(continuous_graph);
  log("Event Values Update Period(secs): "); logln(flush_period);
  logln("-------------------------------------------------");
}
