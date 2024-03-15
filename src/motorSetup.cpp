#include "SimpleFOC.h"
#include "motorSetup.h"
#include "./communication/SimpleFOCDebug.h"

#define TIMESTAMP                       2e6
#define TIMESTEP_GAIN                   1e-6
#define MIN_ELAPSED_TIME                0.2
#define ANGULAR_RESOLUTION              0.0015

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
float motor_target = 0.0;
// void doP(char* cmd) { command.scalar(&motor_target, cmd); }
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
struct PIDTorque {
    float P = 0.15;
    float I = 0.0;
    float D = 0.0;
    float limit = 0.0;
    float errorCurrent;
    float errorPrevious;
    float integral;
    float derivative;
    long anglePrevTS;
    long angleNewTS;
    float motorCommand;
} torquePID;

struct sensorOffset {
    float angleOffset;
    int direction;
} sensorOffset;

void openLoopAngleSetup() {

    Serial.begin(115200);
    Wire.setClock(40000);

    sensor.init();
    driver.init();
    motor.linkDriver(&driver);

    driver.voltage_power_supply = 12;
    // motor.voltage_limit = 4;
    motor.current_limit = 0.5;
    motor.velocity_limit = 10;
    motor.voltage_sensor_align = 0.5;

    motor.controller = MotionControlType::angle_openloop;
    // motor.controller = MotionControlType::velocity_openloop;
    motor.useMonitoring(Serial);
    motor.init();
    // motor.initFOC();

    command.add('T', doTarget, "Target Open Loop Position = ");
    command.add('M', doMotor, "Enable = ME 1, Disable ME 0");

    Serial.println("Motor Setup Complete!");

    sensorOffsetCalibration();
    delay(3000);

}

void sensorOffsetCalibration(){
    sensorOffset.angleOffset = sensor.getAngle() - motor.shaft_angle;
    sensorOffset.direction = motor.sensor_direction;
    // SIMPLEFOC_DEBUG('Sensor Offset: ', sensorOffset.angleOffset);
    // SIMPLEFOC_DEBUG('Sensor Direction: ', sensorOffset.direction);
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
    motor.voltage_limit = 2;
    motor.current_limit = 1.5;
    motor.velocity_limit = 40;
    motor.useMonitoring(Serial);

    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.PID_velocity.P = 0.01;          // need to play with this
    motor.PID_velocity.I = 0.001;          // need to play with this
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

void closedLoopCustomSetup(){

    Serial.begin(115200);
    Wire.setClock(40000);

    sensor.init();
    motor.linkSensor(&sensor);

    driver.init();
    motor.linkDriver(&driver);
    currentSense.linkDriver(&driver);
    
    // currentSense.init();
    // currentSense.skip_align = true;
    // motor.linkCurrentSense(&currentSense);

    driver.voltage_power_supply = 12;
    motor.voltage_sensor_align = 1;
    motor.voltage_limit = 4;
    motor.current_limit = 1.5;
    motor.velocity_limit = 20;
    motor.useMonitoring(Serial);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    // motor.PID_current_q.P = 5;
    // motor.PID_current_q.I= 300;
    // motor.PID_current_d.P= 5;
    // motor.PID_current_d.I = 300;
    // motor.LPF_current_q.Tf = 0.01; 
    // motor.LPF_current_d.Tf = 0.01; 

    motor.init();
    motor.initFOC();

    motor_target = sensor.getAngle();

    command.add('T', doTarget, "Target Closed Loop Position = ");
    command.add('M', doMotor, "Enable = ME 1, Disable ME 0");
    command.add('P', doP, "P = ");
    Serial.println("Motor Setup Complete!");

    delay(1000);

}

void closedLoopCustomLoop(){

    motor.loopFOC();
    float motorCommand = torqueControlPIDLoop(motor_target);
    Serial.print(sensor.getVelocity());
    Serial.print("\t"); 
    Serial.println(motorCommand, 4);
    motor.move();   // call PID loop output as move input
    // motor.monitor();
    command.run();

}

float torqueControlPIDLoop(float motor_target){
    
    torquePID.angleNewTS = micros();
    float Ts = (torquePID.angleNewTS - torquePID.anglePrevTS)*TIMESTEP_GAIN;
    if (Ts >= MIN_ELAPSED_TIME){
        if ((motor_target - sensor.getAngle()) < ANGULAR_RESOLUTION && (motor_target - sensor.getAngle()) > -ANGULAR_RESOLUTION) {
            torquePID.errorCurrent = 0;
        }
        else {
            torquePID.errorCurrent = motor_target - sensor.getAngle();
        }
        torquePID.integral = torquePID.integral + (torquePID.errorCurrent*Ts);
        if (torquePID.integral > torquePID.limit){torquePID.integral = 0;}
        torquePID.derivative =  (torquePID.errorCurrent - torquePID.errorPrevious)/Ts;
        torquePID.motorCommand = _constrain(torquePID.P*torquePID.errorCurrent + torquePID.I*torquePID.integral + torquePID.D*torquePID.derivative, -0.75, 0.75);
        // torquePID.P*torquePID.errorCurrent + torquePID.I*torquePID.integral + torquePID.D*torquePID.derivative;
        
        // if (torquePID.motorCommand >= (motor.current_limit - 0.2)){torquePID.motorCommand = (motor.current_limit - 0.2);}

        torquePID.errorPrevious = torquePID.errorCurrent;
        torquePID.anglePrevTS = torquePID.angleNewTS;
    }
    return torquePID.motorCommand;
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
    Serial.print(motor.sensor_direction); 
    Serial.print("\t"); 
    Serial.print(motor.shaft_angle, 4); 
    Serial.print("\t"); 
    Serial.println(sensorOffset.angleOffset, 4); 
    // Serial.print("\t"); 
    // Serial.println((sensor.getNewVelocity().currentVel), 4); 

    // motor.monitor();
    motor.move();

    command.run();

}

void openLoopHopLoop() {
    
    sensor.update();
    sensorData data = sensor.getNewVelocity();
    float velocity = sensor.getVelocity();
    Serial.print(sensor.getNewVelocity().currentAngle); 
    Serial.print("\t"); 
    Serial.print(motor.sensor_direction); 
    Serial.print("\t"); 
    Serial.print(motor.shaft_angle, 4); 
    Serial.print("\t"); 
    Serial.println(sensorOffset.angleOffset, 4);
    // Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());
    // float setpoints[4] = {0.0, 1.57, 3.14, 1.57};
    // float setpoints[2] = {0.5797, 0.40478};
    float setpoints[2] = {3.53064, 3.02524};

    // each one second
    if(_micros() - timestamp_us > TIMESTAMP) {
        timestamp_us = _micros();
        target_angle = setpoints[counter] - (sensorOffset.direction*sensorOffset.angleOffset);
        counter++;
        if (counter > 1){counter = 0;}
        // inverse angle
        // target_angle = -target_angle;   
    }
    motor.move(target_angle);

    command.run();

}

void openLooopTrajectoryL1(){
    
    sensor.update();
    // Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());
    float trajectory[30] = {3.0235, 2.9249, 2.8392, 2.7676, 2.7105, 2.67,   2.6522, 2.6654, 2.7163, 2.805,
                            2.9225, 3.054,  3.1851, 3.3048, 3.407,  3.407,  3.3493, 3.2971, 3.2502, 3.2085,
                            3.1717, 3.1397, 3.1122, 3.089,  3.0698, 3.0543, 3.0423, 3.0333, 3.0272, 3.0235};
    // each one second
    if(_micros() - timestamp_us > TIMESTAMP) {
        timestamp_us = _micros();
        target_angle = trajectory[counter] - (sensorOffset.direction*sensorOffset.angleOffset);
        counter++;
        if (counter > 29){counter = 0;}

        // inverse angle

    }
    motor.move(target_angle);

    command.run();

}

void openLooopTrajectoryL2(){
    
    sensor.update();
    // Serial.print(sensor.getAngle()); Serial.print("\t"); Serial.println(sensor.getVelocity());
    float trajectory[30] = {0.864,  0.8643, 0.9057, 0.987,  1.0991, 1.2272, 1.3574, 1.4795, 1.5882, 1.6816,
                            1.7593, 1.8214, 1.868,  1.8984, 1.9117, 1.9117, 1.8362, 1.7625, 1.6901, 1.6184,
                            1.547,  1.4754, 1.4034, 1.3306, 1.2569, 1.1818, 1.1052, 1.0268, 0.9465, 0.864};
    // each one second
    if(_micros() - timestamp_us > TIMESTAMP) {
        timestamp_us = _micros();
        target_angle = trajectory[counter] - (sensorOffset.direction*sensorOffset.angleOffset);
        counter++;
        if (counter > 29){counter = 0;}

        // inverse angle

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