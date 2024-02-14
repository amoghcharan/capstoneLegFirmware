#include "Arduino.h"
#include "SimpleFOC.h"
#include "motorSetup.h"

void setup() {
    
    // openLoopAngleSetup();
    closedLoopVelocitySetup();

}

void loop() {

    // openLoopAngleLoop();
    closedLoopVelocityLoop();

}