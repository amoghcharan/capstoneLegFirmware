#include "Arduino.h"
#include "SimpleFOC.h"
#include "constants.h"

#define POLE_PAIRS                  7
#define PHASE_RESISTANCE            0.500f // 0.087 = Internal Resistance

void doTarget(char* cmd); 
void doMotor(char* cmd); 
void doP(char* cmd);

void openLoopAngleSetup();
void openLoopAngleLoop();

void closedLoopVelocitySetup();
void closedLoopPositionSetup();
void closedLoopVelocityLoop();

void closedLoopCustomSetup();
void closedLoopCustomLoop();
float torqueControlPIDLoop(float motor_target);

void openLoopTrajectoryL1();
void openLoopTrajectoryL2();

void sensorOffsetCalibration();
void seriaDebugPrints();
void closedLoopAngleLoop();










// float target = 0.0f;

// inline void serialLoop() {
//     static String received_chars;

//     while (Serial.available()) {
//         char inChar = (char) Serial.read();
//         received_chars += inChar;

//         if (inChar == '\n'){
//             target = received_chars.toFloat();
//             Serial.print("Target = "); Serial.println(target);
//             received_chars = "";
//         }
//     }
// }


// void openLoopVelocitySetup() {
    
//     Serial.begin(115200);
//     // Set up encoder
//     sensor.init();
//     Serial.println("Sensor Ready!");

//     // Link Motor and Encoder
//     motor.linkSensor(&sensor);
//     Serial.println("Motor and Sensor Linked!");

//     // Set up motor driver
//     driver.pwm_frequency = 30000  ;      // 30 kHz
//     driver.voltage_power_supply = 12;   // Voltage Supply
//     driver.init();
//     motor.linkDriver(&driver);
//     Serial.println("Driver Ready!");

//     // Set up Current Sensing
//     // currentSense.init();
//     // currentSense.skip_align = true;     // no need for aligning
//     // motor.linkCurrentSense(&currentSense);

//     // Set up system limits
//     motor.voltage_limit = 2;            // Volts
//     motor.velocity_limit = 20;          // rad/s
//     motor.voltage_sensor_align = 1;     // Limits volts during FOC calibration
//     motor.current_limit = 2.0;          // Amps


//     // Set up Motor
//     motor.controller = MotionControlType::velocity_openloop;
//     motor.init();
//     Serial.println("Motor Ready!");

// }

// void openLoopVelocityLoop() {

//     // Update system
//     sensor.update();
//     motor.move();
//     // motor.monitor();

//     // Print statements
//     Serial.print(sensor.getAngle());
//     Serial.print("\t");
//     Serial.println(sensor.getVelocity());            // Radians??

// }

// void closedLoopVoltageSetup() {

//     // initialize encoder sensor hardware
//     sensor.init();
//     // link the motor to the sensor
//     motor.linkSensor(&sensor);

//     // driver config
//     // power supply voltage [V]
//     driver.voltage_power_supply = 12;
//     driver.init();
//     // link driver
//     motor.linkDriver(&driver);

//     // aligning voltage
//     motor.voltage_sensor_align = 1;

//     // current sense init hardware
//     currentSense.linkDriver(&driver);
//     currentSense.init();
//     // link the current sense to the motor
//     motor.linkCurrentSense(&currentSense);

//     // set motion control loop to be used
//     motor.torque_controller = TorqueControlType::voltage;
//     motor.controller = MotionControlType::torque;

//     // add current limit
//     // motor.phase_resistance = 3.52 // [Ohm]
//     motor.current_limit = 2;   // [Amps] - if phase resistance defined

//     // use monitoring with serial 
//     Serial.begin(115200);
//     // comment out if not needed
//     motor.useMonitoring(Serial);
//     motor.monitor_downsample = 500; // set downsampling can be even more > 100
//     motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

//     // initialize motor
//     motor.init();
//     // align sensor and start FOC
//     motor.initFOC();

//     // set the initial motor target
//     // motor.target = 0.2; // Amps - if phase resistance defined  
//     motor.target = 0.2; // Volts 

//     // add target command T
//     // command.add('T', doTarget, "target current"); // - if phase resistance defined
//     // command.add('T', doTarget, "target voltage");

//     Serial.println(F("Motor ready."));
//     Serial.println(F("Set the target using serial terminal:"));

// }

// void closedLoopCurrentSetup() {

//     // initialize encoder sensor hardware
//     sensor.init();
//     // link the motor to the sensor
//     motor.linkSensor(&sensor);

//     // driver config
//     // power supply voltage [V]
//     driver.voltage_power_supply = 12;
//     driver.init();
//     // link driver
//     motor.linkDriver(&driver);

//     // aligning voltage
//     motor.voltage_sensor_align = 1;
    

//     // current sense init hardware
//     currentSense.linkDriver(&driver);
//     currentSense.init();
//     // link the current sense to the motor
//     motor.linkCurrentSense(&currentSense);

//     // set motion control loop to be used
//     motor.torque_controller = TorqueControlType::foc_current;
//     motor.controller = MotionControlType::torque;

//     // controller configuration based on the control type 
//     motor.PID_velocity.P = 0.05;
//     motor.PID_velocity.I = 1;
//     motor.PID_velocity.D = 0;
//     // default voltage_power_supply
//     motor.voltage_limit = 12;
    
//     // velocity low pass filtering time constant
//     motor.LPF_velocity.Tf = 0.01;

//     // angle loop controller
//     motor.P_angle.P = 20;
//     // angle loop velocity limit
//     motor.velocity_limit = 20;

//     // add current limit
//     // motor.phase_resistance = 3.52 // [Ohm]
//     motor.current_limit = 2;   // [Amps] - if phase resistance defined

//     // use monitoring with serial 
//     Serial.begin(115200);
//     // comment out if not needed
//     motor.useMonitoring(Serial);
//     // motor.monitor_downsample = 500; // set downsampling can be even more > 100
//     // motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

//     // initialize motor
//     motor.init();
//     // align sensor and start FOC
//     motor.initFOC();

//     // set the initial motor target
//     // motor.target = 0.2; // Amps - if phase resistance defined  
//     motor.target = 0.6; // Volts 

//     // add target command T
//     // command.add('T', doTarget, "target current"); // - if phase resistance defined
//     // command.add('T', doTarget, "target voltage");

//     Serial.println(F("Motor ready."));
//     Serial.println(F("Set the target using serial terminal:"));

// }


// void closedLoopVoltageLoop() {
//     // main FOC algorithm function
//     motor.loopFOC();

//     // Motion control function
//     motor.move();

//     // Monitor motor variables
//     motor.monitor();

// }


// // MOT: Monitor enabled!
// // MOT: Init
// // MOT: Enable driver.
// // MOT: Align sensor.
// // MOT: sensor_direction==CCW
// // MOT: PP check: OK!
// // MOT: Zero elec. angle: 6.05
// // MOT: Align current sense.
// // MOT: Success: 3
// // MOT: Ready.
// // Motor ready.
// // Set the target using serial terminal:
// // 68.8181 114.3584
// // 1.5501  -26.8709
// // -9.9560 -46.1393
// // -3.9446 -22.8058
// // 2.2701  -22.6536
// // -10.4813        -21.2390
// // 6.1216  -23.2499
// // -1.9511 -13.3735
// // 3.7866  -24.4429
// // 0.0071  -30.9136
// // -8.9286 -25.9408
// // 4.6099  -26.1994
// // 4.7297  -33.3517
// // -1.7001 -35.8470
// // 13.0015 -17.5711
// // 8.9401  -24.6279
// // 14.8115 -26.1173
// // 4.7014  -42.7163
// // 12.0624 -33.3105
// // 4.2626  -26.0164
// // -9.4826 -15.9670
// // 3.8726  -17.3185
// // -1.6563 -45.4073
// // -11.0680        -38.0319
// // -5.2310 -43.8621