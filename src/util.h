#ifndef _util_H_
#define _util_H_

#include <WebSerial.h>
#include <Preferences.h>

// Seismic Measuring Variables
extern int16_t ax, ay, az;
extern float xy_vector_mag, x_vector_mag, y_vector_mag, z_vector_mag;
extern float pga;
extern bool eq_status;
extern int eq_time_count;
/* Peak Ground Acceleration
https://en.wikipedia.org/wiki/Peak_ground_acceleration
https://en.wikipedia.org/wiki/Japan_Meteorological_Agency_seismic_intensity_scale
*/
extern uint16_t flush_event;
extern bool flush_update;
extern bool calibrating;

// Parameters
extern bool SPK_HAT;
extern int8_t lcd_brightness; // TFT backlight brightness for standby ( value: 7 - 15 )
extern float pga_trigger, pga_trigger_changed; // (g - m/s2)
extern uint16_t flush_period; //seconds
extern bool continuous_graph; // false: Draw graph only when EQ happens
extern bool slta; // true: use STA/LTA Method instead of trigger

// Logging
extern uint8_t logging; // 0: None, 1: Serial only, 2: WebSerial only, 3: Serial and WebSerial

extern bool webSerialEnabled; // Disable until WebSerial is loaded

extern Preferences preferences; // To store and read parameters

template<typename T>
void logln(T msg) {
  if (logging == 1 || logging == 3) Serial.println(msg);
  if ((logging == 2 || logging == 3) && webSerialEnabled) WebSerial.println(msg);
}

template<typename T, typename U>
void logln(T msg, U digits) {
  char msgStr[6];
  dtostrf(msg, 6,digits, msgStr);
  if (logging == 1 || logging == 3) Serial.println(msgStr);
  if ((logging == 2 || logging == 3) && webSerialEnabled) WebSerial.println(msgStr);
}

template<typename T>
void log(T msg) {
  if (logging == 1 || logging == 3) Serial.print(msg);
  if ((logging == 2 || logging == 3) && webSerialEnabled) WebSerial.print(msg);
}

template<typename T, typename U>
void log(T msg,U digits) {
  char msgStr[6];
  dtostrf(msg, 6,digits, msgStr);
  if (logging == 1 || logging == 3) Serial.print(msgStr);
  if ((logging == 2 || logging == 3) && webSerialEnabled) WebSerial.print(msgStr);
}

void pref_init();
void pref_update();

#endif