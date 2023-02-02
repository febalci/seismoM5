#ifndef _screen_H_
#define _screen_H_

class screen
{
  public:
    void show_wait();
    void show_calibration();
    void show_title();
    void show_method(boolean _stalta, boolean _clear, float _pga_trigger = 0);
    void show_mqtt(boolean _mqtt);
    void draw_axis();
    void draw_acc_graph(float _xVector, float _yVector, float _zVector);
    void draw_pga_graph(float _pga);
    void show_pga(float _pga);
    void show_stalta(float _sta, float _lta);
    void clear_screen();
    void set_brightness(uint8_t _brightness);
  private:
    // Screen coordinates
    #define graph_x_axis 7 // X Coordinate for Vertical axis line
    uint8_t graph_x_start = 8; // X Coordinate for where the graph starts (increases)
    uint8_t previous_graph_y[3];
    #ifdef STICKC
      // Graph coordinates for main screen: M5StickC: 160x80
      uint8_t graph_y_axis[3] = {25,35,45}; // Y coordinates for X,Y,Z horizontal axis lines
      #define graph_y_axis_boundary 5 // pixels
      #define graph_x_limit 155 // X Coordinate limit for graph
      #define graph_scale 50
      #define graph_clear_y_height 37
      #define pga_print_x 120
      #define pga_print_y 0
      #define mqtt_print_x 130 // 120 for text size 1
      #define mqtt_print_y 70
    #else
      // Graph coordinates for main screen: M5StickCPlus: 240x135 pixels
      uint8_t graph_y_axis[3] = {35,65,95}; // Y coordinates for X,Y,Z horizontal axis lines
      #define graph_y_axis_boundary 15 // pixels
      #define graph_x_limit 235 // X Coordinate limit for graph
      #define graph_scale 150
      #define graph_clear_y_height 97
      #define pga_print_x 200
      #define pga_print_y 2
      #define mqtt_print_x 200
      #define mqtt_print_y 125
    #endif
};

#endif