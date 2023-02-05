#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif
#include "screen.h"
#include "FreeFonts.h"

void screen::show_wait() {
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
    for (int i = 0; i <= 19; i++) { // Wait 10 secs
      delay(1000);
      M5.Lcd.fillRect(32+(i*5),60,5,10,YELLOW);
    }
  #else
    M5.Lcd.setCursor(60, 25);
    M5.Lcd.print("PREPARE FOR");
    M5.Lcd.setCursor(60, 55);
    M5.Lcd.print("CALIBRATION");
    M5.Lcd.drawRect(72,110,100,15,YELLOW);
    for (int i = 0; i <= 19; i++) { // Wait 10 secs
      delay(1000);
      M5.Lcd.fillRect(72+(i*5),110,5,15,YELLOW);
    }
  #endif // 160x80-240x135
}

void screen::show_calibration() {
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
}

void screen::show_title() {
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(GREEN,BLACK);
  M5.Lcd.setCursor(2, pga_print_y);
  M5.Lcd.print("EARTHQUAKE SENSOR");
}

void screen::show_method(boolean _stalta, boolean _clear, float _pga_trigger) {
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(YELLOW,BLACK);
  if (_clear) {
    M5.Lcd.setCursor(pga_print_x,pga_print_y);
    M5.Lcd.print("      ");
  }
  M5.Lcd.setCursor(pga_print_x,pga_print_y);
  if (_stalta) M5.Lcd.printf("S/LTA"); else M5.Lcd.printf("%.4f", _pga_trigger);
}

void screen::show_mqtt(boolean _mqtt) {
  M5.Lcd.setTextFont(1);
  if (_mqtt) {
    M5.Lcd.setTextColor(YELLOW,PURPLE);
    M5.Lcd.setCursor(mqtt_print_x,mqtt_print_y);
    M5.Lcd.print("MQTT");
  } else {
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.setCursor(mqtt_print_x,mqtt_print_y);
    M5.Lcd.print("    ");
  }
}

void screen::draw_axis() {
  // Draw Graph Axis
  M5.Lcd.drawFastVLine(7,15,graph_clear_y_height+3,WHITE);
  // Draw Tickmarks
  for (int i = 0; i < 3 ; i++) {
    M5.Lcd.drawFastHLine(5,graph_y_axis[i],2,WHITE);
  }
}

void screen::draw_acc_graph(float _xVector, float _yVector, float _zVector) {
  int y0[3], y1[3];
  float vectors[3] = {_xVector, _yVector, _zVector};

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

void screen::draw_pga_graph(float _pga) {
  int y0, y1;

  if (previous_graph_y[2] != 0) {
    y0 = previous_graph_y[2];
  } else {
    y0 = graph_y_axis[2];
  }
  y1 = graph_y_axis[2] - (_pga * 450);
  if (y1 > (graph_y_axis[2] + 15)) y1 = graph_y_axis[2] + 15;
  if (y1 < (graph_y_axis[2] - 15)) y1 = graph_y_axis[2] - 15;
  previous_graph_y[2] = y1;
    
  M5.Lcd.fillRect(graph_x_start+1, 15, 20, graph_clear_y_height, BLACK);
  M5.Lcd.drawLine(graph_x_start, y0, graph_x_start+1, y1, GREEN);

  graph_x_start++;
  if (graph_x_start > graph_x_limit) graph_x_start = graph_x_axis + 1;
}

void screen::show_pga(float _pga) {
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setTextFont(1);
#ifdef STICK
  M5.Lcd.fillRect(2,mqtt_print_y-7,mqtt_print_x-7,14,BLACK);
#else
  M5.Lcd.fillRect(2,mqtt_print_y-11,mqtt_print_x-7,18,BLACK);
#endif
  M5.Lcd.setCursor(2, mqtt_print_y);
  M5.Lcd.print("PGA: ");
  M5.Lcd.setCursor(mqtt_print_x-40, mqtt_print_y);
  M5.Lcd.print("(g)");
#ifdef STICKC
  M5.Lcd.setCursor(30, mqtt_print_y+5);
  M5.Lcd.setFreeFont(FSSB9);  // Select Free Sans Serif Bold 9pt font
  M5.Lcd.printf("%.4f", _pga);
#else
  M5.Lcd.setCursor(30, mqtt_print_y+5);
  M5.Lcd.setFreeFont(FSSB12);  // Select Free Sans Serif Bold 9pt font
  M5.Lcd.printf("%.4f", _pga);
#endif
  M5.Lcd.setTextFont(1);
}

void screen::show_stalta(float _sta, float _lta) {
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(WHITE,BLACK);
#ifdef STICKC
  M5.Lcd.fillRect(2,mqtt_print_y-7,mqtt_print_x-7,14,BLACK);
  M5.Lcd.setCursor(2, mqtt_print_y);
  M5.Lcd.print("S:");
  M5.Lcd.setCursor(mqtt_print_x-55, mqtt_print_y);
  M5.Lcd.print("L:");
  M5.Lcd.setCursor(14, mqtt_print_y+5);
  M5.Lcd.setFreeFont(FSSB9);  // Select Free Sans Serif Bold 9pt font
  M5.Lcd.printf("%.4f", _sta);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setCursor(mqtt_print_x-43, mqtt_print_y);
#else
  M5.Lcd.fillRect(2,mqtt_print_y-11,mqtt_print_x-7,19,BLACK);
  M5.Lcd.setCursor(2, mqtt_print_y);
  M5.Lcd.print("S:");
  M5.Lcd.setCursor(mqtt_print_x-75, mqtt_print_y);
  M5.Lcd.print("L:");
  M5.Lcd.setCursor(14, mqtt_print_y+5);
  M5.Lcd.setFreeFont(FSSB12);
  M5.Lcd.printf("%.4f", _sta);
  M5.Lcd.setFreeFont(FSSB9);
  M5.Lcd.setCursor(mqtt_print_x-63, mqtt_print_y+4);
#endif
  M5.Lcd.printf("%.4f", _lta);
}

void screen::clear_screen() {
  M5.Lcd.fillScreen(BLACK);
}

void screen::set_brightness(uint8_t _brightness) {
  M5.Axp.ScreenBreath(_brightness); // Full Brightness
}
