#ifndef MAGNETOMETERLIB_H
#define MAGNETOMETERLIB_H

#include <Arduino.h>
#include <Wire.h>

class MagnetometerLib {
public:
    MagnetometerLib(byte address = 0x10, long frequency = 100000,int resolution_config = 2, int magnetometer_sample_rate_code = 123);
    void initialise();
    void setConfigMode(int config);
    bool dataReady();
    void readValues(int16_t* data);

private:
    byte _address;
    long _frequency;
    int _resolution_config;
    int _magnetometer_sample_rate_code;
    void writeByteToRegister(byte reg, byte value);
    void readByteArrayFromRegister(byte reg, int nbytes, byte* outArray);
};

#endif
