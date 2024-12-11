#include <LiquidCrystal_I2C.h>
#include <SimpleDHT.h>

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT11 sensor
int pinDHT11 = 26;
SimpleDHT11 dht11;

// PIR motion sensor and LED
int led = 13;                // GPIO 13 for LED
int sensor = 2;              // GPIO 2 for PIR sensor
int state = LOW;             // by default, no motion detected
int val = 0;                 // variable to store the sensor status (value)

// Buzzer
const int buzzer = 14;       // GPIO 14 for buzzer

void setup() {
  // LCD setup
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  delay(1000);

  Serial.begin(9600);
  Serial.println("Simple DHT11 Test");

  // Initialize pins
  pinMode(led, OUTPUT);      // LED as an output
  pinMode(sensor, INPUT);    // PIR sensor as an input
  pinMode(buzzer, OUTPUT);   // Buzzer as an output
}

void loop() {
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;

  // Read temperature and humidity from DHT11
  if ((err = dht11.read(pinDHT11, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err=");
    Serial.println(err);
    delay(2000);
    return;
  }

  Serial.print("Sample OK: ");
  Serial.print((int)temperature);
  Serial.print(" *C, ");
  Serial.print((int)humidity);
  Serial.println(" RH%");
  delay(1000);

  // Display temperature and humidity on the LCD
  lcd.setCursor(2, 0);   
  lcd.print(temperature);
  lcd.print(" *C, ");
  delay(1000);
  lcd.setCursor(2, 1);
  lcd.print(humidity);
  lcd.print(" RH%");
  delay(1000);

  // Buzzer sounds if temperature exceeds 20 degrees Celsius
  if (temperature > 20) {
    tone(buzzer, 1000);  // Send a 1KHz sound signal
    delay(1000);         // Sound duration
    noTone(buzzer);      // Stop the buzzer
    delay(1000);
  } else {
    noTone(buzzer);      // Ensure the buzzer is off if temperature is normal
  }

  // PIR motion sensor logic
  val = digitalRead(sensor);   // Read sensor value
  if (val == HIGH) {           // Check if the sensor is HIGH
    digitalWrite(led, HIGH);   // Turn LED ON
    delay(100);                // Delay 100 milliseconds 
    
    if (state == LOW) {
      Serial.println("Motion detected!"); 
      state = HIGH;       // Update variable state to HIGH
    }
  } else {
    digitalWrite(led, LOW);    // Turn LED OFF
    delay(200);                // Delay 200 milliseconds 
    
    if (state == HIGH) {
      Serial.println("Motion stopped!");
      state = LOW;        // Update variable state to LOW
    }
  }
}
