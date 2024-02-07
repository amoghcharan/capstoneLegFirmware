#include "SimpleFOC.h"
#include "constants.h"

// Init Motor
BLDCMotor motor(14, 0.087, 900);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
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

void openLoopVelocityLoop(float target) {

    // Update system
    sensor.update();
    motor.move(target);

    // Print statements
    Serial.print(sensor.getAngle());
    Serial.print("\t");
    Serial.println(sensor.getVelocity());            // Radians??

}