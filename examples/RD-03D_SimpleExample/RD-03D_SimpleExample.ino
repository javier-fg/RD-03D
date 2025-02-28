#include <Arduino.h>
#include "RD03D.h"

// Define pins and baud rate parameters.
#define RX_PIN 26
#define TX_PIN 27

// Create an instance of the RD03D class, and assign the module connection pins
RD03D radar(RX_PIN, TX_PIN);

void setup() {

  Serial.begin(115200);    // Initialize debugging Serial port.

  Serial.println("\n\n--- RD-03D Radar Module - Single Target Example ---");

  // Initialize the radar module.
  if( radar.initialize(RD03D::RD03DMode::SINGLE_TARGET) ){
    Serial.println("Module Initialized");
  }else{
    Serial.println("ERROR - Module not Initialized");
    while(1){
      Serial.print(".");
      delay(1000);
    }
  }   
}

void loop() {

  static TargetData*  ptrTarget = radar.getTarget();    // get pointer to first target ( SINGLE DETECTION )
  static uint64_t     next_screen_update = 0;
  static bool         detected = false;

  // Call the task method frequently to check for new frames.
  radar.tasks();

  // Plot information, We display data a bit less oftern
  if ( millis() > next_screen_update){

    next_screen_update = millis() + 1000;   // Update next tick every second

    // Check if Target is detected, then display the values.
    if(ptrTarget->isValid()){

      // If previous not detected: new line in serial console, for pretty prints
      if(!detected){
        detected = true;
        Serial.print("\n");
      }

      ptrTarget->printInfo();   // Display target information over serial

      // You can also access the targetData values, for specific functions
      // ptrTarget->distance, ptrTarget->angle . . .

    }else{
      detected = false;
      Serial.print(".");
    }
  }

  delay(10); // Some delay
}
