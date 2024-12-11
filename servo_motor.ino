#include <Servo.h>

Servo servo1;  // Create first servo object for tilt
Servo servo2;  // Create second servo object for pan

unsigned long MOVING_TIME = 3000; // Moving time in milliseconds (3 seconds)
unsigned long moveStartTime;
int startTiltAngle = 90;  // Starting angle for tilt (servo1)
int stopTiltAngle  = 180;  // Stopping angle for tilt (servo1)
int startPanAngle = 0;    // Starting angle for pan (servo2)
int stopPanAngle  = 180;  // Stopping angle for pan (servo2)

void setup() {
  servo1.attach(25);  // Attach the first servo to pin 14 (for tilt)
  servo2.attach(14);  // Attach the second servo to pin 25 (for pan)
  
  servo1.write(startTiltAngle);  // Initialize the tilt servo to start angle
  servo2.write(startPanAngle);   // Initialize the pan servo to start angle
  
  moveStartTime = millis();  // Start timing the movement
}

void loop() {
  unsigned long progress = millis() - moveStartTime;  // Time passed since movement started
  
  if (progress <= MOVING_TIME) {
    // Calculate the current angle based on the progress for both servos
    long currentTiltAngle = map(progress, 0, MOVING_TIME, startTiltAngle, stopTiltAngle);
    long currentPanAngle = map(progress, 0, MOVING_TIME, startPanAngle, stopPanAngle);
    
    // Set the servo positions
    servo1.write(currentTiltAngle);
    servo2.write(currentPanAngle);
  } else {
    // When movement is complete, reverse the direction for the next loop
    delay(500);  // Wait before reversing direction
    moveStartTime = millis();  // Restart movement timer
    int tempTilt = startTiltAngle;
    startTiltAngle = stopTiltAngle;
    stopTiltAngle = tempTilt;
    
    int tempPan = startPanAngle;
    startPanAngle = stopPanAngle;
    stopPanAngle = tempPan;
  }
}
