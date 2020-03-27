#ifndef CUSTOMSERIALPRINT_H_
#define CUSTOMSERIALPRINT_H_

//#define LOG_DEBUG 1 // Uncomment to activate verbose mode when running

#include "../libraries/I2Cdev/I2Cdev.h"

class CustomSerialPrint {
  public:
#ifdef LOG_DEBUG
    static void begin(unsigned long _baudRate) {
        Serial.begin(_baudRate);
    }
    static void print(int _number) {
        Serial.print(_number);
    }

    static void println(int _number) {
        Serial.println(_number);
    }

    static void print(String _string) {
        Serial.print(_string);
    }

    static void println(String _string) {
        Serial.println(_string);
    }

    static void println(double _string, int significantNb) {
        Serial.println(_string, significantNb);
    }
#else
    static void begin(unsigned long _baudRate) {
    }
    static void print(int _number) {
    }

    static void println(int _number) {
    }

    static void print(String _string) {
    }

    static void println(String _string) {
    }

    static void println(double _string, int significantNb) {
    }
#endif
};

#endif // CUSTOMSERIALPRINT_H_
