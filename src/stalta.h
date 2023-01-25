#ifndef _stalta_H_
#define _stalta_H_

const int sta_lta_element_count = 10; // Reads per second
const int STA_WINDOW_SIZE = 5; // 0.5 secs
const int LTA_WINDOW_SIZE = sta_lta_element_count * 30; // secs
const int PEM_WINDOW_SIZE = 8;

class STA_LTA
{
  public:
    STA_LTA(float triggerThreshold, float detriggerThreshold);
    void updateData(float data);
    bool checkTrigger();
    float getSTA();
    float getLTA();
    int getPET();
    int getPEM();
  private:
    float _sta;
    int _sta_window_current;
    int _bufferIndex_sta;
    float _buffer_sta[STA_WINDOW_SIZE];
    bool _fill_array_sta;
    float _lta;
    int _lta_window_current;
    int _bufferIndex_lta;
    float _buffer_lta[LTA_WINDOW_SIZE];
    bool _fill_array_lta;

    float _triggerThreshold;
    float _detriggerThreshold;
    bool _isTriggered;

    int _pem;
    int _pemIndex;
    float _pemBuffer[PEM_WINDOW_SIZE];
    int _pet;
};

#endif
