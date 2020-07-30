
#include "lx-servo.h"
#include <SoftwareSerial.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

#define LOBOT_SERVO_MAX_ANGLE 240.0
#define LOBOT_SERVO_VIN_RESISTANCE 0.5

//#define LOBOT_DEBUG 1  /*调试是用，打印调试数据*/

void LxServo::attach(HardwareSerial &serial, uint8_t id) {
  _hws = &serial;
  _sws = 0;
  _id = id;
  serial.begin(115200);
}
    
void LxServo::attach(SoftwareSerial &serial, uint8_t id) {
  _hws = 0;
  _sws = &serial;
  _id = id;
  serial.begin(115200);
}

byte LobotCheckSum(byte buf[]) {
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LxServo::write(int16_t position, uint16_t duration) {
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(duration);
  buf[8] = GET_HIGH_BYTE(duration);
  buf[9] = LobotCheckSum(buf);
  serialWrite(buf, 10);
}

long LxServo::move(float degrees, float speed) {
  int currentpos = read();
  if (currentpos > -2048) {
    int targetpos = degrees * 1000.0 / LOBOT_SERVO_MAX_ANGLE;
    uint16_t duration = (float)abs(targetpos - currentpos) / ((1000.0 * 360.0 / LOBOT_SERVO_MAX_ANGLE) / 60000.0 * speed);
    write(targetpos, duration);
    return duration;
  }

  return currentpos;
}

void LxServo::grip(float degrees, float speed, float stopamps, float holdamps) {
  // Stop to get a clean VIN measurement at rest
  stop();
  delay(100);
  float vinrest = vin();
  int posrest = read();

  // Start gripping
  long duration = move(degrees, speed);
  if (duration <= -2048)
    return;
  
  // Detect when blocking occurs
  float avgvin = vinrest;
  float amps;
  
  for (unsigned long ts = millis(), endts = ts + min(duration * 2, 30000); ts < endts; ts = millis()) {
    delay(1);
    avgvin = avgvin * 0.95 + (float)vin() * 0.05;
    amps = (vinrest - avgvin) * LOBOT_SERVO_VIN_RESISTANCE;

    // Detect when average current goes over the max
    if (amps >= stopamps) {
      int pos = read();
      if (pos <= -2048)
        return;

#ifdef LOBOT_DEBUG
      Serial.print("Detected blocked servo at amps:");
      Serial.print(amps);
      Serial.print(", position: ");
      Serial.println(pos);
#endif

      // Try to backoff 1 unit at a time
      for (int i = 0; i < 50 && posrest != pos; i++) {
        if (pos <= -2048)
          return;

        pos = pos + (posrest < pos ? 1 : -1);
        write(pos, 1);
        delay(10);

        // .. until the realtime current measurement goes below the hold
        amps = (vinrest - vin()) * LOBOT_SERVO_VIN_RESISTANCE;
        if ((int)(amps * 100) <= (int)(holdamps * 100)) {
#ifdef LOBOT_DEBUG
          Serial.print("Backed off grip to amps:");
          Serial.print(amps);
          Serial.print(", position: ");
          Serial.println(pos);
#endif
          break;
        }
      }

      // Slowly apply torque until holding securely
      for (int i = 0; i < 50 && posrest != pos; i++) {
        if (pos <= -2048)
          return;

        pos = pos + (posrest < pos ? 1 : -1);
        write(pos, 1);
        delay(10);

        // .. until the realtime current measurement goes below the hold
        amps = (vinrest - vin()) * LOBOT_SERVO_VIN_RESISTANCE;
        if ((int)(amps * 100) >= (int)(holdamps * 100)) {
#ifdef LOBOT_DEBUG
          Serial.print("Holding grip at amps:");
          Serial.print(amps);
          Serial.print(", position: ");
          Serial.println(pos + (posrest < pos ? 1 : -1));
#endif
          break;
        }
      }
      
      return;
    }
  }
}

void LxServo::stop() {
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  serialWrite(buf, 6);
}

void LxServo::setID(uint8_t newID) {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  serialWrite(buf, 7);
  
  _id = newID;
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LxServo::setMode(uint8_t mode, int16_t speed) {
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  serialWrite(buf, 10);
}

void LxServo::load() {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  serialWrite(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LxServo::unload() {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  serialWrite(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

float LxServo::angle() {
  int pos = read();
  if (pos > -2048) {
    return (float)pos * LOBOT_SERVO_MAX_ANGLE / 1000.0;
  }
}

int LxServo::read() {
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (serialAvailable())
    serialRead();

  serialWrite(buf, 6);

  while (!serialAvailable()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (receiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

float LxServo::vin() {
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (serialAvailable())
    serialRead();

  serialWrite(buf, 6);

  while (!serialAvailable()) {
    count -= 1;
    if (count < 0)
      return 0;
  }

  if (receiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = 0;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return (float)ret / 1000.0;
}


float LxServo::temperature() {
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = _id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO TEMP READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (serialAvailable())
    serialRead();

  serialWrite(buf, 6);

  while (!serialAvailable()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (receiveHandle(buf) > 0)
    ret = (int8_t)buf[1];
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

void LxServo::serialWrite(const byte *buf, int len) {
  if (_hws) {
    _hws->write(buf, len);
  }
  else {
    _sws->write(buf, len);
  }
}

int LxServo::serialAvailable() {
  if (_hws) {
    return _hws->available();
  }
  else {
    return _sws->available();
  }
}

byte LxServo::serialRead() {
  if (_hws) {
    return _hws->read();
  }
  else {
    return _sws->read();
  }
}

int LxServo::receiveHandle(byte *ret) {
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  long tsend = millis() + 100;
  while (millis() < tsend) {
    if (!serialAvailable()) {
      delayMicroseconds(100);
      continue;
    }
    
    rxBuf = serialRead();
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}
