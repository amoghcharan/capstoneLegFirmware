#include "Arduino.h"
#include "SimpleFOC.h"
#include "motorSetup.h"

#define CONSTANT_VELOCITY             5.00f
float target = 0.0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target, cmd); }

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
    Serial.begin(115200);
    Wire.setClock(400000);

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

    // Set up system limits
    motor.voltage_limit = 4;            // Volts
    motor.velocity_limit = 30;          // rad/s
    motor.voltage_sensor_align = 1;     // Limits volts during FOC calibration

    // Set up Velocity PID Coefficients
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0.03f;

    // Set up Position PID Coefficients
    motor.P_angle.P = 0.5;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    motor.LPF_angle.Tf = 0.03f;

    // Set up Motor
    // motor.controller = MotionControlType::velocity;
    motor.controller = MotionControlType::angle;      // For position control test
    motor.useMonitoring(Serial);
    motor.monitor_variables = _MON_ANGLE | _MON_TARGET | _MON_VEL;
    motor.init();
    motor.initFOC();
    Serial.println("Motor Ready!");


    delay(1000);
}

void loop() {

    // Update system
    serialLoop();
    sensor.update();

    // motor.PID_velocity.P = target;
    // motor.PID_velocity.I = target;      // Use to tune PID
    // motor.PID_velocity.D = target;      // Use to tune PID

    // motor.P_angle.P = target;
    // motor.P_angle.I = target;
    // motor.P_angle.D = target;
    // motor.LPF_angle.Tf = target;

    motor.loopFOC();
    motor.move(target);

    // Print statements
    motor.monitor();
    // Serial.print(sensor.getAngle()*360.0/(6.28));
    // Serial.print(sensor.getAngle());            // Radians??
    // Serial.print("\t");
    // Serial.println(sensor.getVelocity());

}

