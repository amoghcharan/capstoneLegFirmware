#include "Arduino.h"
#include "SimpleFOC.h"
#include "motorSetup.h"

void setup() {  
    openLoopAngleSetup();
}

void loop() {
    openLoopTrajectoryL1(); 
}