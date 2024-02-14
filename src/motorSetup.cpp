#include "SimpleFOC.h"
#include "motorSetup.h"

// Init Motor
BLDCMotor motor(7, 0.500f, 900);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Init Encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void openLoopAngleSetup() {

    Serial.begin(115200);
    Wire.setClock(40000);

    sensor.init();
    driver.init();
    motor.linkDriver(&driver);

    driver.voltage_power_supply = 12;
    motor.voltage_limit = 4;
    motor.current_limit = 1.0;
    motor.velocity_limit = 3;

    motor.controller = MotionControlType::angle_openloop;
    motor.init();

    command.add('T', doTarget, "Target Open Loop Position = ");
    Serial.println("Motor Setup Complete!");

    delay(1000);

}

void closedLoopVelocitySetup(){

    Serial.begin(115200);
    Wire.setClock(40000);

    sensor.init();
    motor.linkSensor(&sensor);

    driver.init();
    motor.linkDriver(&driver);

    // currentSense.init();
    // motor.linkCurrentSense(&currentSense);

    driver.voltage_power_supply = 12;
    motor.voltage_sensor_align = 1;
    motor.voltage_limit = 4;
    motor.current_limit = 1.0;
    motor.velocity_limit = 3;
    motor.useMonitoring(Serial);

    motor.controller = MotionControlType::velocity;
    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 10;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10;
    motor.LPF_velocity.Tf = 0.01;


    motor.init();
    motor.initFOC();

    command.add('T', doTarget, "Target Closed Loop Position = ");
    Serial.println("Motor Setup Complete!");

    delay(1000);

}

void openLoopAngleLoop() {
    
    sensor.update();
    Serial.println(sensor.getAngle());

    motor.move();

    command.run();

}

void closedLoopVelocityLoop(){

    motor.loopFOC();

    motor.move();

    motor.monitor();

    command.run();

}