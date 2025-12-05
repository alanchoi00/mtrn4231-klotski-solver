#include <Servo.h>

Servo myServo;  // Create a Servo object
int servoPin = 23;
int angle = 5;

void setup() {
  Serial.begin(9600);           // Start serial communication
  myServo.attach(servoPin);     // Attach the servo
  myServo.write(0);             // Move to 0 degrees initially
  Serial.println("Ready. Type an angle (0–40):");  
  
               
}

void loop() {
  //int angle = 5;
  int prev_angle = angle;

  if (Serial.available() > 0) {
    angle = Serial.parseInt();
  }

  if ((angle > 0) && (angle <= 40)) {
    myServo.write(angle);
    if (prev_angle != angle) {
      Serial.print("Moved servo to ");
      Serial.print(angle);
      Serial.println(" degrees.");
    }
  } else if (angle != 0) {
    Serial.println("Invalid angle! Please enter a value between 0 and 40.");
  }              // Wait again before repeating
}

