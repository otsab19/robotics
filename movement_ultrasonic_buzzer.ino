// Buzzer connection
const int buzzer = 14; // Buzzer connected to pin 14

// Motor A connections (left motor)
int enA = 25;
int in1 = 33;
int in2 = 32;

// Motor B connections (right motor)
int enB = 19;
int in3 = 18;
int in4 = 21;

// Ultrasonic sensor Pins:
int trigPin = 23;    // Trigger pin
int echoPin = 22;    // Echo pin
long duration, cm, inches;

void setup() {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);

  // Initialize random seed
  randomSeed(analogRead(0));

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Buzzer setup
  pinMode(buzzer, OUTPUT); // Set buzzer as an output
}

void loop() {
  // Ultrasonic sensor code to measure distance
  // Ensure a clean HIGH pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (max distance ~5m)

  // Check if a valid pulse was received
  if (duration == 0) {
    Serial.println("No pulse received");
    cm = 500; // Assume no obstacle within 5 meters
  } else {
    // Convert the time into a distance
    cm = (duration / 2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    inches = (duration / 2) / 74;   // Divide by 74 or multiply by 0.0135
  }

  // Print the results to the Serial Monitor
  Serial.print(inches);
  Serial.print(" in, ");
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();

  // Control the buzzer and motors based on the distance
  if (cm > 0 && cm < 20) { // If object is closer than 20 cm
    tone(buzzer, 1000);  // Sound the buzzer at 1KHz
    motorOff();          // Stop the motors
    Serial.println("Object detected! Stopping motors and sounding buzzer.");
    delay(500);          // Short delay before moving backward
    noTone(buzzer);      // Stop the buzzer

    // Move backward for a short duration
    moveBackward();
    Serial.println("Moving backward.");
    delay(1000);         // Adjust the time as needed
    motorOff();
    delay(500);

    // Randomly decide to turn left or right
    int turnDirection = random(0, 2); // Generates 0 or 1
    if (turnDirection == 0) {
      turnLeft();
      Serial.println("Turning left.");
    } else {
      turnRight();
      Serial.println("Turning right.");
    }
    delay(1000);         // Adjust the time as needed
    motorOff();
    delay(500);
  } else {
    noTone(buzzer);      // Ensure the buzzer is off
    moveForward();       // Continue moving forward
    Serial.println("Moving forward.");
  }

  delay(100);            // Short delay before next sensor reading
}

// Move both motors forward
void moveBackward() {
  analogWrite(enA, 255);  // Full speed for Motor A (left)
  analogWrite(enB, 255);  // Full speed for Motor B (right)

  digitalWrite(in1, HIGH);  // Motor A forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);  // Motor B forward
  digitalWrite(in4, LOW);
}

// Move both motors backward
void moveForward() {
  analogWrite(enA, 255);  // Full speed for Motor A (left)
  analogWrite(enB, 255);  // Full speed for Motor B (right)

  digitalWrite(in1, LOW);  // Motor A backward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);  // Motor B backward
  digitalWrite(in4, HIGH);
}

// Turn left by slowing/stopping the left motor and keeping right motor forward
void turnLeft() {
  analogWrite(enA, 0);   // Slow down or stop Motor A (left)
  analogWrite(enB, 255);   // Full speed for Motor B (right)

  digitalWrite(in1, HIGH);  // Motor A forward (slow)
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);  // Motor B forward
  digitalWrite(in4, LOW);
}

// Turn right by slowing/stopping the right motor and keeping left motor forward
void turnRight() {
  analogWrite(enA, 255);   // Full speed for Motor A (left)
  analogWrite(enB, 0);   // Slow down or stop Motor B (right)

  digitalWrite(in1, HIGH);  // Motor A forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);  // Motor B forward (slow)
  digitalWrite(in4, LOW);
}

// Turn off both motors
void motorOff() {
  analogWrite(enA, 0);  // Stop PWM
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
