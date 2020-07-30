
#ifndef _LX_SERVO_H_
#define _LX_SERVO_H_

#include <Arduino.h>

class HardwareSerial;
class SoftwareSerial;

class LxServo {
  public:
    void attach(HardwareSerial &serial, uint8_t id);
    void attach(SoftwareSerial &serial, uint8_t id);

    void write(int16_t position, uint16_t duration);
    long move(float degrees, float speed);
    void grip(float degrees, float speed, float stopamps, float holdamps);
    void stop();
    
    void setID(uint8_t newID);
    void setMode(uint8_t mode, int16_t speed);
    
    void load();
    void unload();

    float angle();
    int read();
    float vin();
    float temperature();

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
