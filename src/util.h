#ifndef _util_H_
#define _util_H_

#include "config.h"
#include <WebSerial.h>
#include <Preferences.h>

// Seismic Measuring Variables
int16_t ax, ay, az;
float xy_vector_mag, x_vector_mag, y_vector_mag, z_vector_mag;
float pga;
bool eq_status = false;
int eq_time_count;
/* Peak Ground Acceleration
https://en.wikipedia.org/wiki/Peak_ground_acceleration
https://en.wikipedia.org/wiki/Japan_Meteorological_Agency_seismic_intensity_scale
*/
uint16_t flush_event = 0;
bool flush_update = true;

// Parameters
bool SPK_HAT;
int8_t lcd_brightness; // TFT backlight brightness for standby ( value: 7 - 15 )
float pga_trigger, pga_trigger_changed; // (g - m/s2)
uint16_t flush_period; //seconds
bool continuous_graph; // false: Draw graph only when EQ happens

// Logging
uint8_t logging = 3; // 0: None, 1: Serial only, 2: WebSerial only, 3: Serial and WebSerial

bool webSerialEnabled = false; // Disable until WebSerial is loaded

Preferences preferences; // To store and read parameters

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

void pref_init() {
  preferences.begin("seismom5", false);
  pga_trigger = preferences.getFloat("pga_trigger", 0.025);
  SPK_HAT = preferences.getBool("spk_hat", false);
  lcd_brightness = preferences.getChar("brightness", 7);
  continuous_graph = preferences.getBool("continuous", false);
  flush_period = preferences.getUShort("period", 60);
  preferences.end();
  logln("---------------  PREFERENCES READ ---------------");
  log("PGA Trigger: "); logln(pga_trigger);
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
  preferences.end();
  logln("--------------  PREFERENCES UPDATE --------------");
  log("PGA Trigger: "); logln(pga_trigger_changed);
  log("SPK_HAT: "); logln(SPK_HAT);
  log("Lcd Stanby Brightness(7-15): "); logln(lcd_brightness);
  log("Continuous Graph: "); logln(continuous_graph);
  log("Event Values Update Period(secs): "); logln(flush_period);
  logln("-------------------------------------------------");
}
#endif