#include "MagnetometerLib.h"

// Constructor: Initializes the magnetometer with I2C address and frequency
MagnetometerLib::MagnetometerLib(byte address, long frequency, int resolution_config, int magnetometer_sample_rate_code) {
    _address = address;
    _frequency = frequency;
    _resolution_config = resolution_config; // Default to high accuracy
    _magnetometer_sample_rate_code = magnetometer_sample_rate_code;
}

// Initialise the magnetometer: sets up I2C and configures measurement mode
void MagnetometerLib::initialise() {
    Wire.begin();
    Wire.setClock(_frequency);
    /*
    114 = 5Hz
    116 = 15Hz
    123 = 200Hz
    (p27 datasheet)
    */
    writeByteToRegister(14, _magnetometer_sample_rate_code); // Set to continuous measurement mode: 14 is the adress of the mode register, 116 sets continuous mode
    setConfigMode(_resolution_config); // Set to config mode 2 (high accuracy)
}

// Set the configuration mode of the magnetometer
/*
Low Current High Range (CONFIG = 0)
Low Noise High Range (CONFIG = 1)
Low Noise High Sensitivity (CONFIG = 2)
*/
void MagnetometerLib::setConfigMode(int config) {
    if (config == 2) writeByteToRegister(15, 136);
    else if (config == 1) writeByteToRegister(15, 72);
    else if (config == 0) writeByteToRegister(15, 8);
}

// Check if new magnetometer data is available
bool MagnetometerLib::dataReady() {
    Wire.beginTransmission(_address);
    Wire.write(0); // Status register
    Wire.endTransmission(false);
    Wire.requestFrom(_address, 1, true);
    int stat_reg_val = Wire.read();
    return stat_reg_val & 0x01;
}

// Read magnetometer values (X, Y, Z axes)
// data: pointer to int16_t array of size 3
void MagnetometerLib::readValues(int16_t* data) {
    byte rawData[6];
    readByteArrayFromRegister(1, 6, rawData); // Read 6 bytes from data register
    for (int i = 0; i < 6; i += 2) {
        // Combine low and high bytes for each axis
        data[i/2] = (int16_t)(rawData[i] | (rawData[i+1] << 8));
    }
}

// Write a single byte to a register
void MagnetometerLib::writeByteToRegister(byte reg, byte value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

// Read multiple bytes from a register into outArray
void MagnetometerLib::readByteArrayFromRegister(byte reg, int nbytes, byte* outArray) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, nbytes, true);
    for (int i = 0; i < nbytes; i++) {
        outArray[i] = Wire.read();
    }
}
