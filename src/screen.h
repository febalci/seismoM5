#ifndef _screen_H_
#define _screen_H_

class screen
{
  public:
    void show_wifi_connect();
    void show_wait();
    void show_calibration();
    void show_title();
    void show_method(boolean _stalta, boolean _clear, float _pga_trigger = 0);
    void show_mqtt(boolean _mqtt);
    void draw_axis();
    void draw_acc_graph(float _xVector, float _yVector, float _zVector);
    void draw_pga_graph(float _pga);
    void draw_slta_graph(float _sta, float _lta, float _pga);
    void show_pga(float _pga);
    void show_stalta(float _sta, float _lta);
    void clear_screen();
    void set_brightness(uint8_t _brightness);
  private:
    // Screen coordinates
    #define graph_x_axis 7 // X Coordinate for Vertical axis line
    uint8_t graph_x_start = 8; // X Coordinate for where the graph starts (increases)
    uint8_t previous_graph_y[3];
    uint8_t graph_y_axis[3]; // Y coordinates for X,Y,Z horizontal axis lines
    #ifdef STICKC
      // Graph coordinates for main screen: M5StickC: 160x80
      #define graph_y_axis_boundary 5 // pixels
      #define graph_scale 50
      #define graph_clear_y_height 37
      #define title_right_align_x 158
      #define title_top_y 0
      #define title_bottom_y 78
    #else
      // Graph coordinates for main screen: M5StickCPlus: 240x135 pixels
      #define graph_y_axis_boundary 15 // pixels
      #define graph_scale 150
      #define graph_clear_y_height 97
      #define title_right_align_x 238
      #define title_top_y 2
      #define title_bottom_y 133
    #endif
};

#endif
