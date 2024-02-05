// #include "Arduino.h"
// #include "SimpleFOC.h"
// #include "motorSetup.h"

// float target = 0.0;

// void serialLoop() {
//     static String received_chars;

//     while (Serial.available()) {
//         char inChar = (char) Serial.read();
//         received_chars += inChar;

//         if (inChar == '/n'){
//             target = received_chars.toFloat();
//             Serial.print('Target = '); Serial.println(target);
//             received_chars = "";
//         }
//     }
// }

// void setup() {
//     Serial.begin(115200);

//     sensor.init();
//     Serial.println("Sensor Ready!");

//     delay(1000);
// }

// void loop() {
//     serialLoop();

//     Serial.print(sensor.getAngle());
//     Serial.print("/t");
//     Serial.print(sensor.getVelocity());

// }