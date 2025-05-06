#include <Servo.h>

Servo servo[4];
const int servoMinAngle = 0;
const int servoMaxAngle = 180;

void setup() {
  Serial.begin(9600);  // Match this baud rate with your Python script
  
  // Attach each servo to its respective pin
  servo[0].attach(8);  // Motor attached to pin 8
  servo[1].attach(6);  // Motor attached to pin 6
  servo[2].attach(7);  // Motor attached to pin 7
  servo[3].attach(5);  // Motor attached to pin 5

  // Initialize servos to their middle positions
  for (int i = 0; i < 4; i++) {
    servo[i].write(90);
  }

  Serial.println("Arduino ready to receive servo angles");
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();  // Remove any whitespace

    // Split the input string into individual angle values
    int angleValues[4] = {0};
    int angleIndex = 0;
    int commaIndex = 0;
    
    while (commaIndex > -1 && angleIndex < 4) {
      commaIndex = inputString.indexOf(',');
      if (commaIndex > -1) {
        angleValues[angleIndex] = inputString.substring(0, commaIndex).toInt();
        inputString = inputString.substring(commaIndex + 1);
      } else {
        angleValues[angleIndex] = inputString.toInt();
      }
      angleIndex++;
    }

    // Move servos based on received angles
    for (int i = 0; i < 4; i++) {
      int angle = constrain(angleValues[i], servoMinAngle, servoMaxAngle);
      servo[i].write(angle);
      
      Serial.print("Servo ");
      Serial.print(i);
      Serial.print(" Angle: ");
      Serial.println(angle);
    }
  }
}