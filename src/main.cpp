#include "Arduino.h"
#include "SimpleFOC.h"
// #include "motorSetup.h"

#define CONSTANT_VELOCITY             5.00f
#define GOAL_POSITION                -2.50f
float target = 0.00;

// Init Motor
BLDCMotor motor(14, 0.087, 900);                                                               
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); 
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Init Encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

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

    // // Set up encoder
    sensor.init();
    Serial.println("Sensor Ready!");

    // // Link Motor and Encoder
    motor.linkSensor(&sensor);
    Serial.println("Motor and Sensor Linked!");

    // // Set up motor driver
    driver.voltage_power_supply = 6;   // Voltage Supply
    driver.init();
    motor.linkDriver(&driver);
    Serial.println("Driver Ready!");

    // link current sense and the driver
    currentSense.linkDriver(&driver);

    // // Set up Current Sensing
    // currentSense.init();
    // currentSense.skip_align = true;     // no need for aligning
    // motor.linkCurrentSense(&currentSense);

    // // Set up system limits
    motor.voltage_limit = 4.0;            // Volts
    motor.velocity_limit = 20;          // rad/s
    motor.voltage_sensor_align = 1;     // Limits volts during FOC calibration
    motor.current_limit = 2.50;          // Amps
    motor.PID_velocity.output_ramp = 1000;

    // // Set up Velocity PID Coefficients
    motor.PID_velocity.P = 0.20f;
    motor.PID_velocity.I = 20.0f;
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0.01f;

    // // Set up Motor
    // motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity;      // For position control test
    motor.useMonitoring(Serial);                            // Target | Voltage | Current | Velocity | Angle
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_ANGLE | _MON_VEL | _MON_CURR_Q ;

    // command.add('T', doTarget, "target angle");
    
    motor.init();
    motor.initFOC();
    Serial.println("Motor Ready!");


    delay(1000);
}

void loop() {

    // Update system
    serialLoop();
    // sensor.update();

    motor.loopFOC();
    motor.move(target);

    // Print statements
    motor.monitor();
    // command.run();


}



// void setup() {

//   // initialise magnetic sensor hardware
//   sensor.init();
//   // link the motor to the sensor
//   motor.linkSensor(&sensor);

//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 12;
//   driver.init();
//   // link the motor and the driver
//   motor.linkDriver(&driver);

//   // set motion control loop to be used
//   motor.controller = MotionControlType::velocity;

//   // contoller configuration
//   // default parameters in defaults.h

//   // velocity PI controller parameters
//   motor.PID_velocity.P = 0.2f;
//   motor.PID_velocity.I = 20;
//   motor.PID_velocity.D = 0;
//   // default voltage_power_supply
//   motor.voltage_limit = 6;
//   // jerk control using voltage voltage ramp
//   // default value is 300 volts per sec  ~ 0.3V per millisecond
//   motor.PID_velocity.output_ramp = 1000;

//   // velocity low pass filtering
//   // default 5ms - try different values to see what is the best.
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.01f;

//   // use monitoring with serial
//   Serial.begin(115200);
//   // comment out if not needed
//   motor.useMonitoring(Serial);

//   // initialize motor
//   motor.init();
//   // align sensor and start FOC
//   motor.initFOC();

//   // add target command T
//   command.add('T', doTarget, "target velocity");

//   Serial.println(F("Motor ready."));
//   Serial.println(F("Set the target velocity using serial terminal:"));
//   _delay(1000);
// }

// void loop() {
//   // main FOC algorithm function
//   // the faster you run this function the better
//   // Arduino UNO loop  ~1kHz
//   // Bluepill loop ~10kHz
//   motor.loopFOC();

//   // Motion control function
//   // velocity, position or voltage (defined in motor.controller)
//   // this function can be run at much lower frequency than loopFOC() function
//   // You can also use motor.move() and set the motor.target in the code
//   motor.move(target);

//   // function intended to be used with serial plotter to monitor motor variables
//   // significantly slowing the execution down!!!!
//   motor.monitor();

//   // user communication
//   command.run();
// }


