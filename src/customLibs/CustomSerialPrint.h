#ifndef CUSTOMSERIALPRINT_H_
#define CUSTOMSERIALPRINT_H_

//#define LOG_DEBUG 1 // Uncomment to activate verbose mode when running

#include "../libraries/I2Cdev/I2Cdev.h"

class CustomSerialPrint {
  public:
    static void print(int _number) {
#ifdef LOG_DEBUG
        Serial.print(_number);
#endif
    }

    static void println(int _number) {
#ifdef LOG_DEBUG
        Serial.println(_number);
#endif
    }

    static void print(String _string) {
#ifdef LOG_DEBUG
        Serial.print(_string);
#endif
    }

    static void println(String _string) {
#ifdef LOG_DEBUG
        Serial.println(_string);
#endif
    }
};

#endif // CUSTOMSERIALPRINT_H_
