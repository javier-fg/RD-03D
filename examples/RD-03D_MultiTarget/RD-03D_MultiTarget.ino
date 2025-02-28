#include <Arduino.h>
#include "RD03D.h"

// Define pins and baud rate parameters.
#define RX_PIN 26
#define TX_PIN 27

// Create an instance of the RD03D class, and assign its pins
RD03D radar(RX_PIN, TX_PIN);

void setup() {

  Serial.begin(115200);    // Initialize debugging Serial port.

  Serial.println("\n\n--- RD-03D Radar Module - Multi Target Example ---");

  // Initialize the radar module.
  if( radar.initialize(RD03D::RD03DMode::MULTI_TARGET) ){
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

  static TargetData*  ptrTarget;
  static uint64_t     next_screen_update = 0;
  static bool         detected = false;

  // Call the task method frequently to check for new frames.
  radar.tasks();

  // Plot information, We display data a bit less oftern
  if ( millis() > next_screen_update){

    next_screen_update = millis() + 1000;   // Update next tick

    if( radar.getTargetCount() == 0){
      Serial.println("-NO TARGETS");
    }else{
      Serial.println("-TARGETS: ");

      // Iterate over the targets and look for valid
      for(uint8_t i = 0 ; i < RD03D::MAX_TARGETS; i++){
        ptrTarget = radar.getTarget(i);   // Get reference to the target

        if(ptrTarget->isValid()){
          Serial.print("  ");
          ptrTarget->printInfo();
        }
      }
    }
  }

  delay(10); // Some delay
}
