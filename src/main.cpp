#include "Arduino.h"
#include "SimpleFOC.h"
#include "motorSetup.h"

#define CONSTANT_VELOCITY             5.00f
#define GOAL_POSITION                -2.50f
float target = 0.00;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void serialLoop() {
    static String received_chars;

    while (Serial.available()) {
        char inChar = (char) Serial.read();
        received_chars += inChar;

        if (inChar == '\n'){
            target = received_chars.toFloat();
            Serial.print("Target = "); Serial.println(target);
            received_chars = "";
        }
    }
}

void setup() {
    // Set up Serial + I2C
    // Serial.begin(115200);
    closedLoopCurrentSetup();
    Wire.setClock(400000);

    command.add('T', doTarget, "target current"); // - if phase resistance defined

    delay(1000);
}

void loop() {

    // Update system
    // serialLoop();
    // openLoopVelocityLoop(target);
    closedLoopVoltageLoop();
    // sensor.update();

    // motor.loopFOC();
    // motor.move(target);

    // // Print statements
    // motor.monitor();
    // command.run();

    // user communication
    command.run();


}

    // openLoopVelocitySetup();

    // // Set up encoder
    // sensor.init();
    // Serial.println("Sensor Ready!");

    // // // Link Motor and Encoder
    // motor.linkSensor(&sensor);
    // Serial.println("Motor and Sensor Linked!");

    // // // Set up motor driver
    // driver.voltage_power_supply = 12;   // Voltage Supply
    // driver.init();
    // motor.linkDriver(&driver);
    // Serial.println("Driver Ready!");

    // // link current sense and the driver
    // currentSense.linkDriver(&driver);

    // // // Set up Current Sensing
    // // currentSense.init();
    // // currentSense.skip_align = true;     // no need for aligning
    // // motor.linkCurrentSense(&currentSense);

    // // // Set up system limits
    // motor.voltage_limit = 6.0;            // Volts
    // motor.velocity_limit = 20;          // rad/s
    // motor.voltage_sensor_align = 1;     // Limits volts during FOC calibration
    // motor.current_limit = 5.50;          // Amps
    // motor.PID_velocity.output_ramp = 1000;

    // // // Set up Velocity PID Coefficients
    // motor.PID_velocity.P = 0.20f;
    // motor.PID_velocity.I = 20.0f;
    // motor.PID_velocity.D = 0;
    // motor.LPF_velocity.Tf = 0.01f;

    // // // Set up Motor
    // motor.torque_controller = TorqueControlType::voltage;
    // motor.controller = MotionControlType::velocity;      // For position control test
    // motor.useMonitoring(Serial);                            // Target | Voltage | Current | Velocity | Angle
    // motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_ANGLE | _MON_VEL | _MON_CURR_Q ;

    // // command.add('T', doTarget, "target angle");
    
    // motor.init();
    // motor.initFOC();
    // Serial.println("Motor Ready!");