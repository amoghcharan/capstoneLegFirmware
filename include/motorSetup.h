#include "SimpleFOC.h"
#include "constants.h"

#define PHASE_RESISTANCE            0.800f // 0.087 = Internal Resistance

// Init Motor
BLDCMotor motor(7, PHASE_RESISTANCE, 900);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
// LowsideCurrentSense current_sense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);        // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Init Encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// Encoder sensor = Encoder(A_HALL2, A_HALL3, 2048, A_HALL1);


void openLoopVelocitySetup() {
// Set up encoder
    sensor.init();
    Serial.println("Sensor Ready!");

    // Link Motor and Encoder
    motor.linkSensor(&sensor);
    Serial.println("Motor and Sensor Linked!");

    // Set up motor driver
    driver.pwm_frequency = 30000  ;      // 30 kHz
    driver.voltage_power_supply = 12;   // Voltage Supply
    driver.init();
    motor.linkDriver(&driver);
    Serial.println("Driver Ready!");

    // Set up Current Sensing
    // currentSense.init();
    // currentSense.skip_align = true;     // no need for aligning
    // motor.linkCurrentSense(&currentSense);

    // Set up system limits
    motor.voltage_limit = 2;            // Volts
    motor.velocity_limit = 20;          // rad/s
    motor.voltage_sensor_align = 1;     // Limits volts during FOC calibration
    // motor.current_limit = 0.6;          // Amps

    // Set up Motor
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();
    Serial.println("Motor Ready!");

}

void closedLoopVoltageSetup() {

    // initialize encoder sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    driver.init();
    // link driver
    motor.linkDriver(&driver);

    // aligning voltage
    motor.voltage_sensor_align = 1;

    // current sense init hardware
    currentSense.linkDriver(&driver);
    currentSense.init();
    // link the current sense to the motor
    motor.linkCurrentSense(&currentSense);

    // set motion control loop to be used
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;

    // add current limit
    // motor.phase_resistance = 3.52 // [Ohm]
    motor.current_limit = 2;   // [Amps] - if phase resistance defined

    // use monitoring with serial 
    Serial.begin(115200);
    // comment out if not needed
    motor.useMonitoring(Serial);
    motor.monitor_downsample = 500; // set downsampling can be even more > 100
    motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // set the initial motor target
    // motor.target = 0.2; // Amps - if phase resistance defined  
    motor.target = 0.2; // Volts 

    // add target command T
    // command.add('T', doTarget, "target current"); // - if phase resistance defined
    // command.add('T', doTarget, "target voltage");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target using serial terminal:"));

}

void closedLoopCurrentSetup() {

    // initialize encoder sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    driver.init();
    // link driver
    motor.linkDriver(&driver);

    // aligning voltage
    motor.voltage_sensor_align = 1;

    // current sense init hardware
    currentSense.linkDriver(&driver);
    currentSense.init();
    // link the current sense to the motor
    motor.linkCurrentSense(&currentSense);

    // set motion control loop to be used
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::torque;

    // controller configuration based on the control type 
    motor.PID_velocity.P = 0.05;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.D = 0;
    // default voltage_power_supply
    motor.voltage_limit = 12;
    
    // velocity low pass filtering time constant
    motor.LPF_velocity.Tf = 0.01;

    // angle loop controller
    motor.P_angle.P = 20;
    // angle loop velocity limit
    motor.velocity_limit = 20;

    // add current limit
    // motor.phase_resistance = 3.52 // [Ohm]
    motor.current_limit = 2;   // [Amps] - if phase resistance defined

    // use monitoring with serial 
    Serial.begin(115200);
    // comment out if not needed
    motor.useMonitoring(Serial);
    motor.monitor_downsample = 500; // set downsampling can be even more > 100
    motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

    // initialize motor
    motor.init();
    // align sensor and start FOC
    motor.initFOC();

    // set the initial motor target
    // motor.target = 0.2; // Amps - if phase resistance defined  
    motor.target = 0.2; // Volts 

    // add target command T
    // command.add('T', doTarget, "target current"); // - if phase resistance defined
    // command.add('T', doTarget, "target voltage");

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target using serial terminal:"));

}

void openLoopVelocityLoop(float target) {

    // Update system
    sensor.update();
    motor.move(target);

    // Print statements
    Serial.print(sensor.getAngle());
    Serial.print("\t");
    Serial.println(sensor.getVelocity());            // Radians??

}

void closedLoopVoltageLoop() {
    // main FOC algorithm function
    motor.loopFOC();

    // Motion control function
    motor.move();

    // Monitor motor variables
    motor.monitor();

}