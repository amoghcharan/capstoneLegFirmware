#include "Arduino.h"
#include "SimpleFOC.h"
#include "motorSetup.h"

void setup() {
   
    // openLoopAngleSetup();
    // closedLoopVelocitySetup();
    closedLoopCustomSetup();
    // closedLoopPositionSetup();

}

void loop() {

    // openLoopAngleLoop();
    // openLooopTrajectoryL2(); 
    // openLoopHopLoop();
    closedLoopVelocityLoop();

}