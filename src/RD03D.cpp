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

#include <Arduino.h>
#include <math.h>
#include "RD03D.h"

//
// --- TargetData Class Implementation ---
//
TargetData::TargetData() : idNum(0), x(0), y(0), speed(0), distanceRes(0), distance(0), angle(0), _isValid(false) {}

void TargetData::clearValues(){
  x = 0;
  y = 0;
  speed = 0;
  distanceRes = 0;
  distance = 0;
  angle = 0;
  timestamp = 0;
  _isValid = false;
}

bool TargetData::setValues(int16_t _x, int16_t _y, int16_t _speed, uint16_t _distanceRes) {

  // If distanceResolution is zero, the measurement is not valid
  if(_distanceRes == 0){
    clearValues();
    return false;
  }else
    _isValid = true;

  // Save values
  x = _x;
  y = _y;
  speed = _speed;
  distanceRes = _distanceRes;
  timestamp = millis();

  // Compute distance and angle based on x and y values.
  distance = sqrt(sq(x) + sq(y));
  angle = atan2((float)y, (float)x) * 180.0 / PI;

  // If distance is more than theoretical, measurement not valid
  if( distance > MAX_DISTANCE){
    clearValues();
    return false;
  }

  return _isValid;
}

void TargetData::printInfo(){

  Serial.print("Target-");
  Serial.print(idNum);
  Serial.print(": ");

  if (!isValid() ){
    Serial.println("-- no valid data --");
    return;
  }

  // Serial.print("Distance: ");
  // Serial.print(distance / 10.0);
  // Serial.print(" cm, Angle: ");
  // Serial.print(angle);
  // Serial.print("°, X: ");
  // Serial.print(x);
  // Serial.print(" mm, Y: ");
  // Serial.print(y);
  // Serial.print(" mm, Speed: ");
  // Serial.print(speed);
  // Serial.print(" cm/s, Resolution: ");
  // Serial.println(distanceRes);

  Serial.print(distance / 10.0);
  Serial.print(" cm, ");
  Serial.print(angle);
  Serial.print("°, (");
  Serial.print(x / 10);
  Serial.print(",");
  Serial.print(y / 10);
  Serial.print("), ");
  Serial.print(speed);
  Serial.println(" cm/s");

}


//
// --- RD03D Class Implementation ---
//
RD03D::RD03D(uint8_t rxPin, uint8_t txPin, uint32_t baudRate, HardwareSerial* serial, size_t bufferSize)
  : _rxPin(rxPin), _txPin(txPin), _baudRate(baudRate), _bufferSize(bufferSize), _mode(SINGLE_TARGET),
    _serial(serial), _bufferRxIndex(0), _bufferTxIndex(0), _targetCount(0)
{
  // Allocate the buffer for incoming data.
  _bufferRx = new uint8_t[_bufferSize];
  _bufferTx = new uint8_t[30];

  // Assign target ID incrementally
  for(uint8_t i = 0 ; i < MAX_TARGETS; i++){
    _targets[i].idNum = i + 1;
  }
}

bool RD03D::initialize( RD03DMode mode ) {

  uint8_t res;

  // Begin serial communication on the given port with the specified baud rate.
  _serial->begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);
  delay(10); // Allow time for the serial port and module to initialize

  // Set mode
  _mode = mode;

  // Send configuration commands to the radar module.
  cmd_buffer_rx_clean();
  if (_mode == SINGLE_TARGET) {
    cmd_send_buffer(CMD_TARGET_DETECTION_SINGLE, sizeof(CMD_TARGET_DETECTION_SINGLE));
  }else if (_mode == MULTI_TARGET) {
    cmd_send_buffer(CMD_TARGET_DETECTION_MULTI, sizeof(CMD_TARGET_DETECTION_MULTI));
  }else{
    return false;
  }

  // Wait for a response
  res = cmd_receive_ack();

  // TODO we will need to check the response, but since there is no clear documentation, we assume correct command.

  // Clear rx buffer
  cmd_buffer_rx_clean();

  if(res > 0)
    return true;
  else
    return false;
}

bool RD03D::tasks() {

  bool res = false;

  // Continuously read bytes from the serial port.
  while (_serial->available()) {

    uint8_t incomingByte = _serial->read();
    _bufferRx[_bufferRxIndex++] = incomingByte;

    // Prevent buffer overflow.
    if (_bufferRxIndex >= _bufferSize) {
      _bufferRxIndex = _bufferSize - 1;
    }

    // Check if the last two bytes match the frame terminator (0x55, 0xCC).
    if (_bufferRxIndex > 1 && _bufferRx[_bufferRxIndex - 2] == 0x55 && _bufferRx[_bufferRxIndex - 1] == 0xCC) {
      res = processFrame();
#ifdef RD03D_LOGGER_DEBUG
      Serial.print("# RX FRAME: ");
      printHex(_bufferRx, _bufferRxIndex);      
#endif
      _bufferRxIndex = 0; // Reset the buffer after processing.
    }
  }

  return res; 
}

bool RD03D::processFrame() {

  // Example assumes a frame structure with a header (first 4 bytes) then target data.
  // Ensure the frame is long enough (adjust the minimum length as necessary).
  if (_bufferRxIndex < 12) return false;
  
  // Save the target data based on the operating mode.
  uint8_t _targetPtr = 0;
  _targetCount = 0;

  // For multi-target, assume each target occupies 8 bytes and parse them sequentially.
  // Here we start from index 4 and step through the buffer.
  for (size_t i = 4; i + 7 < _bufferRxIndex - 2; i += 8) {

    int16_t tx = (int16_t)(_bufferRx[i] | (_bufferRx[i + 1] << 8)) - 0x200;
    int16_t ty = (int16_t)(_bufferRx[i + 2] | (_bufferRx[i + 3] << 8)) - 0x8000;
    int16_t tspeed = (int16_t)(_bufferRx[i + 4] | (_bufferRx[i + 5] << 8)) - 0x10;
    uint16_t tdistRes = (uint16_t)(_bufferRx[i + 6] | (_bufferRx[i + 7] << 8));

    if(_targets[_targetPtr++].setValues(tx, ty, tspeed, tdistRes))
      _targetCount++;

    // Limit the number of targets to the allocated size.
    if (_targetPtr >= MAX_TARGETS) break;
  }

  return _targetCount > 0;
}

TargetData* RD03D::getTarget(uint8_t target_num) {

  if(target_num >= MAX_TARGETS)
    target_num = MAX_TARGETS -1;

  return &_targets[target_num];
}


// Function to print buffer contents
void RD03D::printHex(const uint8_t *buffer, size_t size) {

  for (int i = 0; i < size; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");  // Add leading zero for single-digit hex values
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
  }

  Serial.println();
}

void RD03D::cmd_send_buffer(const uint8_t *buffer, size_t size){

  _serial->write(buffer, size);

#ifdef RD03D_LOGGER_DEBUG
  Serial.print("# SEND CMD: ");
  printHex(buffer, size);
#endif

}

// Wait for the reception of a frame (ends with 0x04030201). Return the number of bytes read.     
uint8_t RD03D::cmd_receive_ack(){

  unsigned long int timeout = millis() + TIMEOUT_RX;

  _bufferRxIndex = 0;

  // Continuously read bytes from the serial port.
  while (millis() < timeout) {

    if (_serial->available()){

      uint8_t incomingByte = _serial->read();
      _bufferRx[_bufferRxIndex++] = incomingByte;
  
      // Prevent buffer overflow.
      if (_bufferRxIndex >= _bufferSize) {
  #ifdef RD03D_LOGGER_DEBUG
        Serial.println("# RX ACK - BUFFER OVERFLOW");
  #endif
        _bufferRxIndex = 0;
        return 0;
      }
  
      // Check if the bytes match the frame terminator (0x04030201).
      if (_bufferRxIndex > 5 && _bufferRx[_bufferRxIndex - 4] == 0x04 && _bufferRx[_bufferRxIndex - 3] == 0x03 && _bufferRx[_bufferRxIndex - 2] == 0x02 && _bufferRx[_bufferRxIndex - 1] == 0x01) {
  #ifdef RD03D_LOGGER_DEBUG
        Serial.print("# RX ACK: ");
        printHex(_bufferRx, _bufferRxIndex);
  #endif
        return _bufferRxIndex;
      }

    }
  }

#ifdef RD03D_LOGGER_DEBUG
  Serial.println("# RX ACK - TIMEOUT");
#endif
  return 0;
}


void RD03D::cmd_buffer_rx_clean(){
  unsigned long int t = millis();
  _bufferRxIndex = 0;
  while (_serial->available() && millis() - t < TIMEOUT_RX)
    _serial->read();
  _bufferRxIndex = 0;
#ifdef RD03D_LOGGER_DEBUG
  Serial.println("# CLEAR RX BUFFER");
#endif
}