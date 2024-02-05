#include "Arduino.h"
#include "SimpleFOC.h"
#include "motorSetup.h"


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

    // Set up PID Coefficients
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0.01f;

    // Set up Motor
    motor.controller = MotionControlType::velocity;
    motor.useMonitoring(Serial);
    motor.init();
    motor.initFOC();
    Serial.println("Motor Ready!");


    delay(1000);
}

void loop() {

    // Update system
    serialLoop();
    sensor.update();
    motor.loopFOC();
    motor.move();


    // Print statements
    Serial.print(sensor.getAngle());
    Serial.print("\t");
    Serial.println(sensor.getVelocity());

}



// #include "Arduino.h"
// #include "SimpleFOC.h"
// #include "motorSetup.h"

// // angle set point variable
// float target_angle = 120.0;
// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

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

//   // choose FOC modulation (optional)
//   motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

//   // set motion control loop to be used
//   motor.controller = MotionControlType::angle;

//   // contoller configuration
//   // default parameters in defaults.h

//   // velocity PI controller parameters
//   motor.PID_velocity.P = 0.2f;
//   motor.PID_velocity.I = 20;
//   motor.PID_velocity.D = 0;
//   // maximal voltage to be set to the motor
//   motor.voltage_limit = 12;

//   // velocity low pass filtering time constant
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.02f;

//   // angle P controller
//   motor.P_angle.P = 20;
//   // maximal velocity of the position control
//   motor.velocity_limit = 20;

//   // use monitoring with serial
//   Serial.begin(115200);
//   // comment out if not needed
//   motor.useMonitoring(Serial);


//   // initialize motor
//   motor.init();
//   // align sensor and start FOC
//   motor.initFOC();

//   // add target command T
//   command.add('T', doTarget, "target angle");

//   Serial.println(F("Motor ready."));
//   Serial.println(F("Set the target angle using serial terminal:"));
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
//   motor.move(target_angle);


//   // function intended to be used with serial plotter to monitor motor variables
//   // significantly slowing the execution down!!!!
//   // motor.monitor();

//   // user communication
//   command.run();
// }




























// // using a little, sensorless outrunner. Should be capable of at least 300W.
// // BLDCMotor motor(14, 0.087, 900);                                                               // hmm let's mess around with KV. they say it's usually 150%-170% of datasheet value but they also say it's 100%-200%.
// // BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL); // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
// // LowsideCurrentSense current_sense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);        // got this from here: https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/B_G431B_ESC1/B_G431B_ESC1.ino
// // MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// // Commander commander(Serial);
// // void doMotor(char *cmd) { commander.motion(&motor, cmd); }
// // void doTarget(char *cmd) { commander.motion(&motor, cmd); }
// // void doLimitCurrent(char *cmd) { commander.scalar(&motor.current_limit, cmd); }

// // float target = 300; //rpm. this needs to be low to start. i'm almost certain your motor won't be able to make big jumps in rpm. to go higher, ramp it up. maybe write a ramp function that sets motor.target += 20 every few seconds. can definitely just start with commander "T??"

// // void setup()
// // {
// //   Serial.begin(115200);

// //   // configure i2C
// //   Wire.setClock(400000);
// //   // initialise magnetic sensor hardware
// //   sensor.init();

// //   Serial.println("Sensor ready");
// //   // _delay(1000);


// //   delay(3000); //NEED THIS, at least for some using platformio, so that the IDE serial monitor has time to boot up before the MCU tries to send stuff to it. at least for me, if i don't do this, I miss some or all SimpleFOCDebug statements at the beginning, which are SO useful.
// //   motor.useMonitoring(Serial); //this doesn't slow anything down. need this if you want to see the SimpleFOCDebug statements at the beginning, which are so useful. 
// //   motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // specifies which variables get output. there's a whole bunch of them you can monitor. check this out if interested: https://docs.simplefoc.com/monitoring
// //   motor.monitor_downsample = 500;                                // downsampling, or how many loops between each communication with Serial. default 10. Higher number here means less frequent samples, so less latency, and less crazy serial output.

// //   driver.voltage_power_supply = 12; //
// //   // driver.voltage_limit = 5;   //makes no visible changes. motor.current_limit is the limiting factor.
// //   driver.pwm_frequency = 30000; // 20kHz is the standard for many boards, but the B-G431B-ESC1 can do 30kHz. i don't see why one would lower this if hardware can handle it. the higher, the smoother.
// //   driver.init(); //essential
// //   motor.linkDriver(&driver); //essential

// //   // current_sense.skip_align; //don't do this unless...idk, you'd need some good reason.
// //   current_sense.linkDriver(&driver); //essential
// //   current_sense.init(); //essential
// //   motor.linkCurrentSense(&current_sense); //essential

// //   motor.current_limit = 2.0; // measured in Amps. IF THIS IS TOO LOW, YOUR MOTOR WON'T SPIN. REGARDLESS, START LOW (5% of rated current and move up to 10%, 20%, 30%, etc.) REQUIRES motor.phase_resistance to be set, which it is. 
// //   // motor.voltage_limit = 0.05f; // do this OR motor.current_limit. current limit superior, in my humble opinion.
// //   motor.torque_controller = TorqueControlType::foc_current; //voltage mode should work. dc_current better. foc_current best. Thankfully, the STM32G4 on the B-G431B-ESC1 can do 170MHz so it can do foc_current, which needs lots of computation power.
// //   motor.controller = MotionControlType::velocity_openloop; //it's the name of this example and the identifying part of this setup/algorithm.
// //   // motor.foc_modulation = FOCModulationType::SinePWM; // default or...
// //   // motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // not default. 15% more power, but only matters once everything else is tuned, i think.
// //   motor.initFOC(); //essential
// //   motor.init(); //essential

// //   motor.target = target * 6.28 / 60;              // converting rpm to rev/s
// //   commander.add('M', doMotor, (char *)"motor");   // this is the connection control command SimpleFOCStudio (GUI) needs to send to connect and work...haven't gotten it to work yet :) 
// //   commander.add('T', doTarget, (char *)"target"); // type in "T20" to set the target to 20. Type in "T50" to set the target to 50, etc.
// //   commander.add('C', doLimitCurrent, (char *)"current"); //type in "C1" to set the current limit to 1A. Type in "C10" to set the current limit to 10A, etc.

// //   Serial.println("setup done");
// // }

// // void loop()
// // {
// //   sensor.update();
  
// //   // display the angle and the angular velocity to the terminal
// //   Serial.print(sensor.getAngle());
// //   Serial.print("\t");
// //   Serial.println(sensor.getVelocity());
  
  
// //   motor.loopFOC(); //this needs to run. always.
// //   motor.move(); //this needs to run. always.
// //   motor.monitor(); //enable if you wish. with downsampling of 500, shouldn't be too epileptic and shouldn't affect performance. lower downsampling if you want more data (and more latency)
// //   commander.run(); //this never hurts. much neater than writing one's own "if(Serial.available()>0) { String command = Serial.readString(); ...}"
// // }

// // // Bonus: if you want to kill the program from the Serial monitor for some reason, define this before setup: 
// // // void(* resetFunc) (void) = 0; //declare reset function at address 0
// // // Then you need your own "if(Serial.available()>0) { String command = Serial.readString(); ...}" kind of thing going on in loop();
// // // Then, in loop, send something via Serial Monitor to make resetFunc() execute.
// // // That being said, "T0" works fine, and the reset approach will not actually reset the board. you'll have to upload again or something to restart the program.