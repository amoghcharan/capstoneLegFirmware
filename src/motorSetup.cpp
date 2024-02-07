// #include "SimpleFOC.h"
// #include "motorSetup.h"

// void openLoopVelocitySetup() {
// // Set up encoder
//     sensor.init();
//     // sensor.enableInterrupts(doA, doB); // For 'encoder' does not work
//     Serial.println("Sensor Ready!");

//     // Link Motor and Encoder
//     motor.linkSensor(&sensor);
//     Serial.println("Motor and Sensor Linked!");

//     // Set up motor driver
//     driver.voltage_power_supply = 12;   // Voltage Supply
//     driver.init();
//     motor.linkDriver(&driver);
//     Serial.println("Driver Ready!");

//     // Set up Current Sensing
//     // currentSense.init();
//     // currentSense.skip_align = true;     // no need for aligning
//     // motor.linkCurrentSense(&currentSense);

//     // Set up system limits
//     motor.voltage_limit = 1;            // Volts
//     motor.velocity_limit = 20;          // rad/s
//     motor.voltage_sensor_align = 1;     // Limits volts during FOC calibration
//     // motor.current_limit = 0.6;          // Amps

//     // Set up Velocity PID Coefficients
//     // motor.PID_velocity.P = 0.2f;
//     // motor.PID_velocity.I = 20.0;
//     // motor.PID_velocity.D = 0;
//     // motor.LPF_velocity.Tf = 0.01f;

//     // Set up Position PID Coefficients
//     // motor.P_angle.P = 10.0;
//     // motor.P_angle.I = 0.0;
//     // motor.P_angle.D = 0;
//     // motor.P_angle.output_ramp = 100;
//     // motor.LPF_angle.Tf = 0.03f;

//     // Set up Motor
//     // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
//     motor.controller = MotionControlType::velocity_openloop;
//     // motor.controller = MotionControlType::angle;      // For position control test
//     motor.useMonitoring(Serial);
//     // motor.monitor_variables = _MON_ANGLE | _MON_TARGET | _MON_VEL | _MON_CURR_Q | _MON_VOLT_Q;
    
//     motor.init();
//     // motor.initFOC();
//     Serial.println("Motor Ready!");

// }

// void openLoopVelocityLoop(float target) {

//     // Update system
//     sensor.update();

//     // motor.PID_velocity.I = target;      // Use to tune PID

//     // motor.loopFOC();
//     motor.move(target);
//     motor.monitor();

//     // Print statements
//     // Serial.print(sensor.getAngle());
//     // Serial.print("\t");
//     // Serial.println(sensor.getVelocity());            // Radians??
//     // Serial.print("\t");
//     // Serial.println(motor.PID_velocity.I);
// }