#include "SimpleFOC.h"
#include "motorSetup.h"

#define TIMESTAMP                       5e5

// Init Motor
BLDCMotor motor(7, 0.500f, 900);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Init Encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }

// Init Low Pass Filter
LowPassFilter positionFilter = LowPassFilter(0.001);
LowPassFilter velocityFilter = LowPassFilter(0.005);

// temp
long timestamp_us = _micros();
float target_angle = 0.5;
float current_angle = 0.0;
float filtered_angle = positionFilter(current_angle);
int counter = 0;
float trajectory[30];
float zeroOffset;

void openLoopAngleSetup() {

    Serial.begin(115200);
    Wire.setClock(40000);

    sensor.init();
    driver.init();
    motor.linkDriver(&driver);

    driver.voltage_power_supply = 12;
    // motor.voltage_limit = 4;
    motor.current_limit = 1.5;
    motor.velocity_limit = 20;

    // motor.controller = MotionControlType::angle_openloop;
    motor.controller = MotionControlType::velocity_openloop;
    motor.useMonitoring(Serial);
    motor.init();

    command.add('T', doTarget, "Target Open Loop Position = ");
    command.add('M', doMotor, "Enable = ME 1, Disable ME 0");

    Serial.println("Motor Setup Complete!");

    delay(1000);

}

void closedLoopCustomSetup(){

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
    // motor.voltage_limit = 4;
    motor.current_limit = 1.5;
    motor.velocity_limit = 20;
    motor.useMonitoring(Serial);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    motor.init();
    motor.initFOC();

    command.add('T', doTarget, "Target Closed Loop Position = ");
    command.add('M', doMotor, "Enable = ME 1, Disable ME 0");
    command.add('P', doP, "P = ");
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
    // motor.voltage_limit = 4;
    motor.current_limit = 1.5;
    motor.velocity_limit = 20;
    motor.useMonitoring(Serial);

    motor.controller = MotionControlType::velocity;
    motor.PID_velocity.P = 2.0;          // need to play with this
    motor.PID_velocity.I = 1.0;          // need to play with this
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 1.0; // need to increase this 
    // motor.PID_velocity.limit = 0.5;

    motor.LPF_velocity.Tf = 0.01 ;
    // motor.motion_downsample = 1000;

    motor.init();
    motor.initFOC();

    command.add('T', doTarget, "Target Closed Loop Position = ");
    command.add('M', doMotor, "Enable = ME 1, Disable ME 0");
    command.add('P', doP, "P = ");
    Serial.println("Motor Setup Complete!");

    delay(1000);

}

void closedLoopPositionSetup(){

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
    // motor.voltage_limit = 4;
    motor.current_limit = 2.0;
    motor.velocity_limit = 5;
    motor.useMonitoring(Serial);

    motor.controller = MotionControlType::angle;
    motor.PID_velocity.P = 1.0;          // need to play with this
    motor.PID_velocity.I = 0.0;          // need to play with this
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10; // need to increase this 
    motor.LPF_velocity.Tf = 1.0;
    motor.motion_downsample = 1000;

    motor.P_angle.P = 1;

    motor.init();
    motor.initFOC();

    command.add('T', doTarget, "Target Closed Loop Position = ");
    command.add('M', doMotor, "Enable = ME 1, Disable ME 0");
    Serial.println("Motor Setup Complete!");

    delay(1000);

}

void closedLoopCustomLoop(){

    motor.loopFOC();
    motor.move();   // call PID loop output as move input
    motor.monitor();
    command.run();

}

void openLoopAngleLoop() {
    
    sensor.update();
    // Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());
    // Serial.print(positionFilter(sensor.getAngle())); Serial.print("\t");Serial.print(sensor.getVelocity()); Serial.print("\t"); Serial.println(velocityFilter(sensor.getVelocity()));
    // Serial.print("\t"); Serial.println(positionFilter(sensor.getVelocity()));
    sensorData data = sensor.getNewVelocity();
    float velocity = sensor.getVelocity();
    Serial.print(sensor.getNewVelocity().currentAngle); 
    Serial.print("\t"); 
    Serial.print(sensor.getNewVelocity().previousAngle); 
    Serial.print("\t"); 
    Serial.print(sensor.getNewVelocity().timeStep, 4); 
    Serial.print("\t"); 
    Serial.print(velocity, 4); 
    Serial.print("\t"); 
    Serial.println((sensor.getNewVelocity().currentVel), 4); 

    // motor.monitor();
    motor.move();

    command.run();

}

void openLoopHopLoop() {
    
    sensor.update();
    Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());

    // each one second
    if(_micros() - timestamp_us > TIMESTAMP) {
        timestamp_us = _micros();
        // inverse angle
        target_angle = -target_angle;   
    }
    motor.move(target_angle);

    command.run();

}

void openLooopTrajectoryL1(){
    
    sensor.update();
    // Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());
    float trajectory[30] = {3.2597, 3.3582, 3.444,  3.5156, 3.5727, 3.6131, 3.631,  3.6178, 3.5669, 3.4782,
                            3.3607, 3.2291, 3.0981, 2.9783, 2.8761, 2.8761, 2.9339, 2.9861, 3.033,  3.0747,
                            3.1115, 3.1435, 3.1709, 3.1942, 3.2134, 3.2289, 3.2409, 3.2499, 3.256,  3.2597};
    // each one second
    if(_micros() - timestamp_us > TIMESTAMP) {
        timestamp_us = _micros();
        counter++;
        // inverse angle
        target_angle = trajectory[counter];
        // if (CCW){
        //     target_angle = zeroOffset + trajectory[counter];   
        // }
        // else {
        //     target_angle = zeroOffset - trajectory[counter];   
        // }
    }
    motor.move(target_angle);

    command.run();

}

void openLooopTrajectoryL2(){
    
    sensor.update();
    // Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());
    float trajectory[30] = {2.2775, 2.2773, 2.2359, 2.1546, 2.0425, 1.9144, 1.7842, 1.6621, 1.5534, 1.46,
                            1.3823, 1.3202, 1.2736, 1.2432, 1.2299, 1.2299, 1.3054, 1.379,  1.4515, 1.5232,
                            1.5946, 1.6662, 1.7382, 1.8109, 1.8847, 1.9598, 2.0364, 2.1148, 2.1951, 2.2775};
    // each one second
    if(_micros() - timestamp_us > TIMESTAMP) {
        timestamp_us = _micros();
        counter++;
        // inverse angle
        target_angle = (trajectory[counter]);

        // if (CCW){
        //     target_angle = zeroOffset + trajectory[counter];   
        // }
        // else {
        //     target_angle = zeroOffset - trajectory[counter];   
        // }   
    }
    motor.move(target_angle);

    command.run();

}

void closedLoopVelocityLoop(){

    motor.loopFOC();

    motor.move();

    motor.monitor();
    // Serial.print("Error: "); 
    // Serial.print("\t");
    // Serial.print(motor.target - motor.shaft_velocity);
    // Serial.print("\t");
    // Serial.print(sensor.getAngle());
    // Serial.print("\t");
    // Serial.println(sensor.getVelocity());  

    command.run();

}