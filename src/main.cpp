#include <Arduino.h>

// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }

#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(14);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);


// encoder instance
Encoder encoder = Encoder(A_HALL2, A_HALL3, 2048, A_HALL1);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motion(&motor, cmd); }

void setup() {
  
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 

  // link the motor to the sensor
  motor.linkSensor(&encoder);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
 
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 4;


  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  
  // user communication
  command.run();
}





























// #include <SimpleFOC.h>

// // MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
// //  chip_address  I2C chip address
// //  bit_resolution  resolution of the sensor
// //  angle_register_msb  angle read register msb
// //  bits_used_msb  number of used bits in msb register
// // 
// // make sure to read the chip address and the chip angle register msb value from the datasheet
// // also in most cases you will need external pull-ups on SDA and SCL lines!!!!!
// //
// // For AS5058B
// // MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// // Example of AS5600 configuration 
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// void setup() {
//   // monitoring port
//   Serial.begin(115200);

//   // configure i2C
//   Wire.setClock(400000);
//   // initialise magnetic sensor hardware
//   sensor.init();

//   Serial.println("Sensor ready");
//   _delay(1000);
// }

// void loop() {
//   // iterative function updating the sensor internal variables
//   // it is usually called in motor.loopFOC()
//   // this function reads the sensor hardware and 
//   // has to be called before getAngle nad getVelocity
//   sensor.update();
  
//   // display the angle and the angular velocity to the terminal
//   Serial.print(sensor.getAngle());
//   Serial.print("\t");
//   Serial.println(sensor.getVelocity());
// }
