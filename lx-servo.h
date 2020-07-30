
#ifndef _LX_SERVO_H_
#define _LX_SERVO_H_

#include <Arduino.h>

class HardwareSerial;
class SoftwareSerial;

class LxServo {
  public:
    attach(HardwareSerial &serial, uint8_t id);
    attach(SoftwareSerial &serial, uint8_t id);

    void write(int16_t position, uint16_t time);
    void move(float angle, float speed);
    void stop();
    
    void setID(uint8_t newID);
    void setMode(uint8_t mode, int16_t speed);
    
    void load();
    void unload();

    int angle();
    int read();
    int vin();
    int temperature();

  private:
    HardwareSerial *_hws;
    SoftwareSerial *_sws;
    uint8_t _id;

    void serialWrite(const byte *buf, int len);
    int serialAvailable();
    byte serialRead();
    int receiveHandle(byte *ret);
};

#endif
