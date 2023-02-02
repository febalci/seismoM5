#include "stalta.h"

STA_LTA::STA_LTA(float triggerThreshold, float detriggerThreshold) {
  _triggerThreshold = triggerThreshold;
  _detriggerThreshold = detriggerThreshold;

  _sta = 0;
  _bufferIndex_sta = 0;
  _sta_window_current = 0;
  _fill_array_sta = false;
  _lta = 0;
  _bufferIndex_lta = 0;
  _lta_window_current = 0;
  _fill_array_lta = false;

  _isTriggered = false;
  _pem = 0;
  _pemIndex = 0;
  _pet = 0;
}

void STA_LTA::updateData(float data) {

    if(!_fill_array_sta) {
      if (_sta_window_current < STA_WINDOW_SIZE) {
        _sta_window_current = _bufferIndex_sta + 1;
      } else {
        _fill_array_sta = true;
        _sta_window_current = STA_WINDOW_SIZE;
      }
    }

    if(!_fill_array_lta) {
      if (_lta_window_current < LTA_WINDOW_SIZE) {
        _lta_window_current = _bufferIndex_lta + 1;
      } else {
        _fill_array_lta = true;
        _lta_window_current = LTA_WINDOW_SIZE;
      }
    }
      
    // calculate short-term average
    _sta = _sta + data - _buffer_sta[_bufferIndex_sta];
    _buffer_sta[_bufferIndex_sta] = data;
    _bufferIndex_sta = (_bufferIndex_sta + 1) % STA_WINDOW_SIZE;

    // calculate long-term average
    _lta = _lta + data - _buffer_lta[_bufferIndex_lta];
    _buffer_lta[_bufferIndex_lta] = data;
    _bufferIndex_lta = (_bufferIndex_lta + 1) % LTA_WINDOW_SIZE;

    // Check for trigger
    if (!_isTriggered && (_sta / _sta_window_current) > (_lta / _lta_window_current) * _triggerThreshold) {
        _isTriggered = true;
        _pem = 0;
        _pemIndex = 0;
    }

    // Check for detrigger
    if (_isTriggered && (_sta / _sta_window_current) < (_lta / _lta_window_current) * _detriggerThreshold) {
        _isTriggered = false;
        _pet = _pem;
    }

    // Update PEM
    if (_isTriggered) {
        _pemBuffer[_pemIndex] = data;
        _pemIndex = (_pemIndex + 1) % PEM_WINDOW_SIZE;
        _pem++;
    }
}

bool STA_LTA::checkTrigger() {
    return _isTriggered;
}

float STA_LTA::getSTA() {
  return _sta / _sta_window_current;
}

float STA_LTA::getLTA() {
  return _lta / _lta_window_current;
}

int STA_LTA::getPET() {
  return _pet;
}

int STA_LTA::getPEM() {
  return _pem;
}