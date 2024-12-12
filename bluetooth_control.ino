#include "BluetoothSerial.h"
#include <Servo.h>

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it.
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
String device_name = "My";

Servo servo1;  // Create first servo object for tilt
Servo servo2;  // Create second servo object for pan

// Default servo angles
int tiltAngle = 90;  // Default tilt angle
int panAngle = 90;   // Default pan angle

unsigned long MOVING_TIME = 3000; // Moving time in milliseconds (3 seconds)
unsigned long moveStartTime;
int startTiltAngle = 90;  // Starting angle for tilt (servo1)
int stopTiltAngle  = 180;  // Stopping angle for tilt (servo1)
int startPanAngle = 0;    // Starting angle for pan (servo2)
int stopPanAngle  = 180;  // Stopping angle for pan (servo2)
bool runCommandActive = false;
bool runTiltOnly = false;
bool runPanOnly = false;


float tiltSpeedDegPerSec = 30.0;
float panSpeedDegPerSec = 30.0;


void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);  // Bluetooth device name
  // Uncomment the next line to delete paired devices (debug only)
  // SerialBT.deleteAllBondedDevices();
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  // Attach the servos to the designated pins
  servo1.attach(25);  // Tilt servo on pin 25
  servo2.attach(14);  // Pan servo on pin 14

  // Set initial positions
  servo1.write(tiltAngle);
  servo2.write(panAngle);
}

void loop() {
  // Check if data is available from Bluetooth
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n'); // Read the incoming command
    command.trim(); // Remove any leading/trailing whitespace
    command.toLowerCase(); // Convert command to lowercase

    // Parse and execute the command
    if (command.startsWith("tilt")) {
      int angle = command.substring(5).toInt(); // Extract angle value
      if (angle >= 0 && angle <= 180) {
        tiltAngle = angle;
        servo1.write(tiltAngle);
        SerialBT.printf("Tilt angle set to %d\n", tiltAngle);
      } else {
        SerialBT.println("Invalid tilt angle! Use a value between 0 and 180.");
      }
    } else if (command.startsWith("pan")) {
      int angle = command.substring(4).toInt(); // Extract angle value
      if (angle >= 0 && angle <= 180) {
        panAngle = angle;
        servo2.write(panAngle);
        SerialBT.printf("Pan angle set to %d\n", panAngle);
      } else {
        SerialBT.println("Invalid pan angle! Use a value between 0 and 180.");
      }
    } else if (command.startsWith("run")) {
      int timeInSeconds = command.substring(4).toInt(); // Extract time value in seconds
      if (timeInSeconds > 0) {
        MOVING_TIME = timeInSeconds * 1000; // Convert to milliseconds
        moveStartTime = millis();
        runCommandActive = true;
        runTiltOnly = false;
        runPanOnly = false;
        SerialBT.printf("Running movement for %d seconds\n", timeInSeconds);
      } else {
        SerialBT.println("Invalid time! Use a positive value in seconds.");
      }
    } else if (command.startsWith("move_tilt")) {
      int timeInSeconds = command.substring(9).toInt(); // Extract time value in seconds
      if (timeInSeconds > 0) {
        MOVING_TIME = timeInSeconds * 1000; // Convert to milliseconds
        moveStartTime = millis();
        runCommandActive = false;
        runTiltOnly = true;
        SerialBT.printf("Running tilt movement for %d seconds\n", timeInSeconds);
      } else {
        SerialBT.println("Invalid time! Use a positive value in seconds.");
      }
    } else if (command.startsWith("move_pan")) {
      int timeInSeconds = command.substring(9).toInt(); // Extract time value in seconds
      if (timeInSeconds > 0) {
        MOVING_TIME = timeInSeconds * 1000; // Convert to milliseconds
        moveStartTime = millis();
        runCommandActive = false;
        runPanOnly = true;
        SerialBT.printf("Running pan movement for %d seconds\n", timeInSeconds);
      } else {
        SerialBT.println("Invalid time! Use a positive value in seconds.");
      }
    } else {
      SerialBT.println("Unknown command! Use tilt <angle>, pan <angle>, run <time>, run_tilt <time>, or run_pan <time>.");
    }
  }

  if (runCommandActive) {
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

      runCommandActive = false; // Stop the run command after one complete cycle
      SerialBT.println("Movement complete.");
    }
  }

  if (runTiltOnly) {
    unsigned long progress = millis() - moveStartTime;  // Time passed since movement started

    if (progress <= MOVING_TIME) {
      // Calculate the current angle based on the progress for tilt servo
      long currentTiltAngle = map(progress, 0, MOVING_TIME, startTiltAngle, stopTiltAngle);

      // Set the tilt servo position
      servo1.write(currentTiltAngle);
    } else {
      // When movement is complete, reverse the direction for the next loop
      delay(500);  // Wait before reversing direction
      moveStartTime = millis();  // Restart movement timer
      int tempTilt = startTiltAngle;
      startTiltAngle = stopTiltAngle;
      stopTiltAngle = tempTilt;

      runTiltOnly = false; // Stop the run_tilt command after one complete cycle
      SerialBT.println("Tilt movement complete.");
    }
  }

  if (runPanOnly) {
    unsigned long progress = millis() - moveStartTime;  // Time passed since movement started

    if (progress <= MOVING_TIME) {
      // Calculate the current angle based on the progress for pan servo
      long currentPanAngle = map(progress, 0, MOVING_TIME, startPanAngle, stopPanAngle);

      // Set the pan servo position
      servo2.write(currentPanAngle);
    } else {
      // When movement is complete, reverse the direction for the next loop
      delay(500);  // Wait before reversing direction
      moveStartTime = millis();  // Restart movement timer
      int tempPan = startPanAngle;
      startPanAngle = stopPanAngle;
      stopPanAngle = tempPan;

      runPanOnly = false; // Stop the run_pan command after one complete cycle
      SerialBT.println("Pan movement complete.");
    }
  }

  delay(20); // Small delay for stability
}
