#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif
#include "screen.h"
#include "FreeFonts.h"


void screen::show_wifi_connect() {
    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(2, 5);
    M5.Lcd.print("CONNECTING WiFi...");
}

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
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.drawString("EARTHQUAKE SENSOR",2,title_top_y,1);
}

void screen::show_method(boolean _stalta, boolean _clear, float _pga_trigger) {
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(YELLOW,BLACK);
  M5.Lcd.setTextDatum(TR_DATUM);
  if (_clear) {
    M5.Lcd.drawString("      ",title_right_align_x,title_top_y);    
  }
  if (_stalta) M5.Lcd.drawString("S/LTA",title_right_align_x,title_top_y); 
    else M5.Lcd.drawFloat(_pga_trigger,4,title_right_align_x,title_top_y);
}

void screen::show_mqtt(boolean _mqtt) {
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextDatum(BR_DATUM);
  if (_mqtt) {
    M5.Lcd.setTextColor(YELLOW,PURPLE);
    M5.Lcd.drawString("MQTT",title_right_align_x,title_bottom_y);
  } else {
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.drawString("    ",title_right_align_x,title_bottom_y);
  }
}

void screen::draw_axis() {
  // Draw Graph Axis
  M5.Lcd.drawFastVLine(7,15,title_bottom_y-38,WHITE);

  // Draw Tickmarks
  for (int i = 0; i < 3 ; i++) {
    graph_y_axis[i]=(2*i+1)*((title_bottom_y-38)/6)+15;
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
  if (graph_x_start > title_right_align_x) graph_x_start = graph_x_axis + 1;
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
  if (graph_x_start > title_right_align_x) graph_x_start = graph_x_axis + 1;
}

void screen::draw_slta_graph(float _sta, float _lta, float _pga) {
  int y0[3], y1[3];
  float vectors[3] = {_sta, _lta, _pga};

  for (int i = 0; i < 3 ; i++) {
    if (previous_graph_y[i] != 0) {
      y0[i] = previous_graph_y[i];
    } else {
      y0[i] = graph_y_axis[2]+8;
    }
    y1[i] = graph_y_axis[2] - (vectors[i] * 2000);
    if (y1[i] > (graph_y_axis[2] + 8)) y1[i] = graph_y_axis[2] + 8;
    if (y1[i] < graph_y_axis[0]) y1[i] = graph_y_axis[0];
    previous_graph_y[i] = y1[i];
  }

  M5.Lcd.fillRect(graph_x_start+1, 15, 20, graph_clear_y_height, BLACK);

  M5.Lcd.drawLine(graph_x_start, y0[0], graph_x_start+1, y1[0], RED);

  M5.Lcd.drawLine(graph_x_start, y0[1], graph_x_start+1, y1[1], BLUE);

//  M5.Lcd.drawLine(graph_x_start, y0[2], graph_x_start+1, y1[2], BLUE);

  graph_x_start++;
  if (graph_x_start > title_right_align_x) graph_x_start = graph_x_axis + 1;
  
}

void screen::show_pga(float _pga) {
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setTextDatum(BL_DATUM);

#ifdef STICKC
  M5.Lcd.fillRect(2,title_bottom_y-16,title_right_align_x-30,16,BLACK);
#else
  M5.Lcd.fillRect(2,title_bottom_y-20,title_right_align_x-30,20,BLACK);
#endif

  M5.Lcd.setTextFont(1);
  M5.Lcd.drawString("PGA:", 2, title_bottom_y);

#ifdef STICKC
  M5.Lcd.setFreeFont(FSSB9);  // Select Free Sans Serif Bold 9pt font
#else
  M5.Lcd.setFreeFont(FSSB12);  // Select Free Sans Serif Bold 9pt font
#endif

  M5.Lcd.drawFloat(_pga, 4, 30, title_bottom_y+2);
  M5.Lcd.setTextFont(1);
  M5.Lcd.drawString("(g)", (title_right_align_x/2)+10, title_bottom_y);
}

void screen::show_stalta(float _sta, float _lta) {
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.setTextDatum(BL_DATUM);

#ifdef STICKC
  M5.Lcd.fillRect(2,title_bottom_y-16,title_right_align_x-35,16,BLACK);
#else
  M5.Lcd.fillRect(2,title_bottom_y-20,title_right_align_x-35,20,BLACK);
#endif

  M5.Lcd.setTextFont(1);  
  M5.Lcd.setTextColor(RED,BLACK);
  M5.Lcd.drawString("S:", 2, title_bottom_y);
  M5.Lcd.setTextColor(WHITE,BLACK);

#ifdef STICKC
  M5.Lcd.setFreeFont(FSSB9);  // Select Free Sans Serif Bold 9pt font
#else
  M5.Lcd.setFreeFont(FSSB12);  // Select Free Sans Serif Bold 9pt font
#endif

  M5.Lcd.drawFloat(_sta, 4, 14, title_bottom_y+2);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextColor(BLUE,BLACK);
  M5.Lcd.drawString("L:", (title_right_align_x/2), title_bottom_y);
  M5.Lcd.setTextColor(WHITE,BLACK);
  M5.Lcd.drawFloat(_lta, 4, (title_right_align_x/2)+14, title_bottom_y);
}

void screen::clear_screen() {
  M5.Lcd.fillScreen(BLACK);
}

void screen::set_brightness(uint8_t _brightness) {
  M5.Axp.ScreenBreath(_brightness); // Full Brightness
}
