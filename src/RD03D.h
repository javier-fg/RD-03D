/*
   RD-03D Library
   Copyright (c) 2024 javier-fg

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

#ifndef RD03D_H
#define RD03D_H

#include <Arduino.h>

// #define RD03D_LOGGER_DEBUG   // If defined, some logger information will be sent to Serial.

// The TargetData class encapsulates the parameters of a detected target.
class TargetData {

  public:
  
    uint8_t   idNum;      // target id
    uint32_t  timestamp;  // measurement system timestamp

    int16_t   x;           // X coordinate in mm
    int16_t   y;           // Y coordinate in mm
    int16_t   speed;       // Speed in cm/s
    uint16_t  distanceRes; // Distance resolution in mm
    float     distance;    // Calculated distance (mm)
    float     angle;       // Calculated angle in degrees

   TargetData();

   void clearValues();

   bool setValues(int16_t x, int16_t y, int16_t speed, uint16_t distanceRes);

   bool isValid() { return _isValid; };

   void printInfo();

  private:
    
    bool _isValid;    // flag to mark valid measurement

    const float    MAX_DISTANCE = 10000;  // Max distance in mm ( theoretically module is up to 8 m)
    
};

// The RD03D class encapsulates the radar module interface.
class RD03D {

  public:

    static const uint8_t  MAX_TARGETS = 3;   // Max number of detected targets

    // Define the operating mode for the radar module.
    enum RD03DMode {
      SINGLE_TARGET,
      MULTI_TARGET
    };

    // Constructor: you can specify the RX pin, TX pin, baud rate (default 256000),
    // buffer size (default 1000) and operation mode (default SINGLE_TARGET)
    RD03D(uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 256000, 
      HardwareSerial* serial = &Serial1, size_t bufferSize = 1000);

    // Initializes the hardware serial interface (for example, Serial1)
    bool initialize( RD03DMode mode = RD03DMode::SINGLE_TARGET );

    // Task method to be called regularly in loop() to read and process frames.
    bool tasks();

    // Retrieve operational mode
    RD03DMode   getMode(){ return _mode; };

    // Retrieve the last detected target (for single-target mode)
    TargetData* getTarget( uint8_t target_num = 0);

    // For multi-target mode, get the number of targets parsed
    uint8_t getTargetCount() { return _targetCount; };

    // And get pointer to the targets array
    TargetData* getTargets();

  private:

    // RD03D module constants
    const uint16_t TIMEOUT_RX = 500;  // Timeout to receive a response from module

    const uint8_t CMD_TARGET_DETECTION_SINGLE[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
    const uint8_t CMD_TARGET_DETECTION_MULTI[12]  = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};   

    // Module serial comms
    uint8_t   _rxPin;
    uint8_t   _txPin;
    uint32_t  _baudRate;
    size_t    _bufferSize;

    RD03DMode _mode;  // Operational mode SINGLE / MULTITARGET

    HardwareSerial* _serial;  // Module serial interface

    uint8_t*  _bufferRx;
    size_t    _bufferRxIndex;
    uint8_t*  _bufferTx;
    size_t    _bufferTxIndex;

    // For storing target(s)
    TargetData  _targets[MAX_TARGETS];     // Pointer for multi-target storage
    size_t      _targetCount;  // Number of targets detected

    // Parses a complete frame from the internal buffer.
    bool processFrame();

    void    cmd_send_buffer(const uint8_t *buffer, size_t size);   // Send a buffer command
    uint8_t cmd_receive_ack();    // Wait for the reception of a ACK frame. Return the number of bytes read.     
    void    cmd_buffer_rx_clean(); // Clear rx buffer

    void printHex(const uint8_t *buffer, size_t size);   // Print the current serial buffer

   
};

#endif
